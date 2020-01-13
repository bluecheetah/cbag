// SPDX-License-Identifier: BSD-3-Clause AND Apache-2.0
/*
Copyright (c) 2018, Regents of the University of California
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.


Copyright 2019 Blue Cheetah Analog Design Inc.

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
*/

#include <fmt/core.h>

#include <cbag/common/box_t.h>
#include <cbag/common/param_map_util.h>
#include <cbag/schematic/cellview.h>
#include <cbag/schematic/cellview_info.h>
#include <cbag/schematic/instance.h>
#include <cbag/spirit/ast.h>
#include <cbag/util/io.h>
#include <cbag/util/name_convert.h>
#include <cbag/util/overload.h>
#include <cbag/yaml/cellviews.h>

namespace cbag {
namespace sch {

void validate_sch_cellview(const cellview &cv) {
    // check terminals have legal type and name
    for (auto const & [ term_name, term_fig ] : cv.terminals) {
        if (term_fig.ttype != term_type::input && term_fig.ttype != term_type::output &&
            term_fig.ttype != term_type::inout) {
            throw std::runtime_error("Unsupported terminal type: " +
                                     std::to_string(static_cast<enum_t>(term_fig.ttype)));
        }
        try {
            auto ast = util::parse_cdba_name_unit(term_name);
        } catch (const std::invalid_argument &) {
            throw std::runtime_error("Unsupported schematic terminal name: " + term_name);
        }
    }
}

cellview::cellview() = default;

cellview::cellview(const std::string &yaml_fname, const std::string &sym_view) {
    if (!util::is_file(yaml_fname)) {
        throw std::invalid_argument(yaml_fname + " is not a file.");
    }

    auto n = YAML::LoadFile(yaml_fname);

    *this = n.as<cellview>();
    validate_sch_cellview(*this);

    if (!sym_view.empty()) {
        // load symbol cellview
        auto yaml_path = cbag::util::get_canonical_path(yaml_fname);
        yaml_path.replace_extension(fmt::format(".{}{}", sym_view, yaml_path.extension().c_str()));
        if (cbag::util::is_file(yaml_path)) {
            sym_ptr = std::make_unique<cellview>(yaml_path.string(), "");
        }
    }
}

cellview::cellview(std::string lib_name, std::string cell_name, std::string view_name, coord_t xl,
                   coord_t yl, coord_t xh, coord_t yh)
    : lib_name(std::move(lib_name)), cell_name(std::move(cell_name)),
      view_name(std::move(view_name)), bbox(xl, yl, xh, yh) {}

void cellview::to_file(const std::string &fname) const {
    // create root directory if not exist.
    auto outfile = util::open_file_write(fname);
    YAML::Node node(*this);
    YAML::Emitter emitter;
    emitter << node;
    outfile << emitter.c_str() << std::endl;
    outfile.close();
}

std::unique_ptr<cellview> cellview::get_copy() const {
    // copy easily copiable attributes
    cellview ans(lib_name, cell_name, view_name, xl(bbox), yl(bbox), xh(bbox), yh(bbox));
    ans.terminals = terminals;
    ans.shapes = shapes;
    ans.props = props;
    ans.app_defs = app_defs;

    // copy instances
    for (const auto &pair : instances) {
        ans.instances.emplace_back(pair.first, pair.second->get_copy());
    }
    // copy symbol pointer
    if (sym_ptr != nullptr) {
        ans.sym_ptr = sym_ptr->get_copy();
    }

    return std::make_unique<cellview>(std::move(ans));
}

sig_type cellview::get_sig_type(const std::string &term_name) const {
    auto iter = terminals.find(term_name);
    if (iter == terminals.end())
        throw std::out_of_range("Cannot find terminal " + term_name);
    return iter->second.stype;
}

void cellview::clear_params() { props.clear(); }

void cellview::set_param(std::string name, const param_t &val) {
    cbag::set_param(props, std::move(name), val);
}

void cellview::rename_pin(const std::string &old_name, const std::string &new_name,
                          bool is_symbol) {
    // check the new pin name does not exist already
    if (terminals.find(new_name) != terminals.end()) {
        throw std::invalid_argument(fmt::format("Terminal {} already exists.", new_name));
    }
    // check the new name is legal.  Parse will throw exception if not passed
    auto ast = util::parse_cdba_name_unit(new_name);

    if (terminals.replace_key(old_name, new_name) == terminals.end()) {
        throw std::invalid_argument("Cannot find terminal: " + old_name);
    }

    if (is_symbol) {
        // rename pin text
        for (auto &obj : shapes) {
            auto success = std::visit(
                overload{
                    [&old_name, &new_name](text_t &v) {
                        if (v.text == old_name) {
                            v.text = new_name;
                            return true;
                        }
                        return false;
                    },
                    [](auto &v) { return false; },
                },
                obj);
            if (success)
                break;
        }

        // NOTE: no need to update port order, it'll be corrected by
        // the cellview writer.
    } else if (sym_ptr) {
        // rename the corresponding symbol pin
        sym_ptr->rename_pin(old_name, new_name, true);
    }
}

void cellview::add_pin(const std::string &new_name, enum_t term_type_code, enum_t sig_type_code,
                       bool is_symbol) {
    // check the pin name is legal.  Parse will throw exception if not passed
    auto ast = util::parse_cdba_name_unit(new_name);

    // check the pin name does not exist already
    if (terminals.find(new_name) != terminals.end()) {
        throw std::invalid_argument(fmt::format("Terminal {} already exists.", new_name));
    }
    auto ttype = static_cast<term_type>(term_type_code);

    // find lowest BBox
    auto ybot = COORD_MAX;
    const pin_figure *ref_ptr = nullptr;
    const std::string *name_ptr = nullptr;
    for (const auto & [ term_name, pin_fig ] : terminals) {
        if (pin_fig.ttype == ttype) {
            auto ycur = yl(pin_fig.get_bbox());
            if (ybot > ycur) {
                ybot = ycur;
                ref_ptr = &pin_fig;
                name_ptr = &term_name;
            }
        }
    }

    if (!ref_ptr) {
        throw std::runtime_error(
            fmt::format("Cannot create new pin {} because there are no other pins "
                        "with that type.",
                        new_name));
    }

    // NOTE: magical number that corresponds to one grid space in schematic
    auto dy = -20;
    auto old_name = *name_ptr;
    auto new_pin = ref_ptr->get_move_by(0, dy);
    new_pin.stype = static_cast<sig_type>(sig_type_code);
    terminals.emplace(new_name, std::move(new_pin));
    if (is_symbol) {
        // add new pin text
        auto num_shapes = shapes.size();
        for (std::size_t idx = 0; idx < num_shapes; ++idx) {
            auto found = std::visit(
                overload{
                    [this, &new_name, &old_name, dy](text_t &v) {
                        if (v.text == old_name) {
                            // copy pin text
                            auto new_text = v.get_move_by(0, dy);
                            new_text.text = new_name;
                            shapes.emplace_back(new_text);
                            return true;
                        }
                        return false;
                    },
                    [](auto &v) { return false; },
                },
                shapes[idx]);
            if (found)
                break;
        }

    } else if (sym_ptr) {
        sym_ptr->add_pin(new_name, term_type_code, true);
    }
}

void cellview::set_pin_attribute(const std::string &pin_name, const std::string &key,
                                 const std::string &val) {
    auto iter = terminals.find(pin_name);
    if (iter == terminals.end())
        throw std::invalid_argument(fmt::format("pin {} not found.", pin_name));
    iter->second.attrs.emplace(key, val);
}

bool cellview::remove_pin(const std::string &name, bool is_symbol) {
    auto success = terminals.erase(name) > 0;
    if (success) {
        if (is_symbol) {
            // rename pin text
            auto num_shapes = shapes.size();
            for (std::size_t idx = 0; idx < num_shapes; ++idx) {
                auto found = std::visit(
                    overload{
                        [&name](text_t &v) { return v.text == name; },
                        [](auto &v) { return false; },
                    },
                    shapes[idx]);
                if (found) {
                    shapes.erase(shapes.begin() + idx);
                    break;
                }
            }
        } else if (sym_ptr) {
            sym_ptr->remove_pin(name, true);
        }
    }
    return success;
}

void cellview::rename_instance(const std::string &old_name, std::string new_name) {
    // check the new name does not exist
    if (instances.find(new_name) != instances.end()) {
        throw std::invalid_argument(fmt::format("instance {} already exists.", new_name));
    }
    // check the new name is legal.  Parse will throw exception if not passed
    auto new_ast = util::parse_cdba_name_rep(new_name);

    auto iter = instances.replace_key(old_name, new_name);
    if (iter == instances.end())
        throw std::invalid_argument("Cannot find instance: " + old_name);

    // resize nets if necessary
    auto old_ast = util::parse_cdba_name_rep(old_name);
    auto old_size = old_ast.size();
    auto new_size = new_ast.size();
    if (old_size != new_size) {
        iter->second->resize_nets(old_size, new_size);
    }
}

bool cellview::remove_instance(const std::string &name) { return instances.erase(name) > 0; }

} // namespace sch
} // namespace cbag
