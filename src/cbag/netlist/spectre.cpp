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

#include <map>
#include <utility>
#include <variant>

#include <fmt/core.h>

#include <cbag/spirit/ast.h>

#include <cbag/schematic/cellview.h>
#include <cbag/schematic/cellview_info.h>
#include <cbag/schematic/instance.h>

#include <cbag/netlist/lstream.h>
#include <cbag/netlist/spectre.h>
#include <cbag/spirit/util.h>
#include <cbag/util/io.h>
#include <cbag/util/name_convert.h>

namespace cbag {
namespace netlist {

// spectre netlisting helper functions
namespace spectre {

using term_net_vec_t = std::vector<std::pair<cnt_t, std::vector<std::string>>>;

void get_cv_pins(lstream &b, const std::vector<std::string> &names) {
    for (auto const &name : names) {
        spirit::ast::name_unit ast = cbag::util::parse_cdba_name_unit(name);
        auto n = ast.size();
        for (decltype(n) idx = 0; idx < n; ++idx) {
            std::string tmp = ast.get_name_bit(idx, false, spirit::namespace_cdba{});
            b << tmp;
        }
    }
}

void append_nets(lstream &b, const std::string &inst_name, const sch::instance &inst,
                 const std::vector<std::string> &terms) {
    for (auto const &term : terms) {
        auto term_iter = inst.connections.find(term);
        if (term_iter == inst.connections.end()) {
            throw std::invalid_argument(fmt::format(
                "Cannot find net connected to instance {} terminal {}", inst_name, term));
        }
        if (term_iter->second == "gnd!") {
            // replace gnd! by 0 in netlist
            b << "0";
        } else {
            spirit::ast::name ast = cbag::util::parse_cdba_name(term_iter->second);
            spirit::util::get_name_bits(ast, b.get_back_inserter(), spirit::namespace_cdba{});
        }
    }
}

template <class OutIter>
void write_instance_cell_name(OutIter &&iter, const sch::instance &inst,
                              const sch::cellview_info &info, spectre_stream &stream) {
    if (inst.lib_name == "analogLib") {
        // change CDF names to Spectre names for analogLib cells
        auto itr = stream.cdf2spectre_names.find(inst.cell_name);
        if (itr == stream.cdf2spectre_names.end()) {
            *iter = inst.cell_name;
        } else {
            *iter = itr->second;
        }
    } else {
        *iter = inst.cell_name;
    }

    // get default parameter values
    param_map par_map(info.props);
    // update with instance parameters
    for (auto const &pair : inst.params) {
        par_map.insert_or_assign(pair.first, pair.second);
    }

    // write instance parameters
    if (inst.lib_name == "analogLib") {
        for (auto const &pair : par_map) {
            // map CDF parameters to Spectre parameters
            auto itr2 = stream.cdf2spectre_props.find(pair.first);
            if (itr2 == stream.cdf2spectre_props.end()) {
                std::visit(write_param_visitor(iter, pair.first, stream.precision), pair.second);
            } else {
                std::visit(write_param_visitor(iter, itr2->second, stream.precision), pair.second);
            }
        }
    } else {
        for (auto const &pair : par_map) {
            std::visit(write_param_visitor(iter, pair.first, stream.precision), pair.second);
        }
    }
}

void get_term_net_pairs(term_net_vec_t &term_net_vec, const std::string &inst_name,
                        const sch::instance &inst, const std::vector<std::string> &terms) {
    for (const auto &term : terms) {
        auto term_iter = inst.connections.find(term);
        if (term_iter == inst.connections.end()) {
            throw std::invalid_argument(fmt::format(
                "Cannot find net connected to instance {} terminal {}", inst_name, term));
        }
        spirit::ast::name_unit ast_term = cbag::util::parse_cdba_name_unit(term);
        spirit::ast::name ast = cbag::util::parse_cdba_name(term_iter->second);
        std::vector<std::string> net_vec;
        net_vec.reserve(ast.size());
        spirit::util::get_name_bits(ast, std::back_inserter(net_vec), spirit::namespace_cdba{});
        term_net_vec.emplace_back(ast_term.size(), std::move(net_vec));
    }
}

} // namespace spectre

spectre_stream::spectre_stream(const std::string &fname, cnt_t rmin, cnt_t precision)
    : nstream_file(fname), rmin(rmin), precision(precision) {
    // map CDF cell names to Spectre cell names
    cdf2spectre_names["cap"] = "capacitor";
    cdf2spectre_names["idc"] = "isource";
    cdf2spectre_names["ind"] = "inductor";
    cdf2spectre_names["ipulse"] = "isource";
    cdf2spectre_names["isin"] = "isource";
    cdf2spectre_names["res"] = "resistor";
    cdf2spectre_names["switch"] = "relay";
    cdf2spectre_names["vdc"] = "vsource";
    cdf2spectre_names["vpulse"] = "vsource";
    cdf2spectre_names["vpwlf"] = "vsource";
    cdf2spectre_names["vsin"] = "vsource";

    // map CDF properties to Spectre properties
    cdf2spectre_props["acm"] = "mag";
    cdf2spectre_props["acp"] = "phase";
    cdf2spectre_props["egain"] = "gain";
    cdf2spectre_props["fileName"] = "file";
    cdf2spectre_props["i1"] = "val0";
    cdf2spectre_props["i2"] = "val1";
    cdf2spectre_props["ia"] = "ampl";
    cdf2spectre_props["idc"] = "dc";
    cdf2spectre_props["maxmm"] = "max";
    cdf2spectre_props["minm"] = "min";
    cdf2spectre_props["per"] = "period";
    cdf2spectre_props["pw"] = "width";
    cdf2spectre_props["rc"] = "rclosed";
    cdf2spectre_props["ro"] = "ropen";
    cdf2spectre_props["srcType"] = "type";
    cdf2spectre_props["td"] = "delay";
    cdf2spectre_props["tf"] = "fall";
    cdf2spectre_props["tr"] = "rise";
    cdf2spectre_props["v1"] = "val0";
    cdf2spectre_props["v2"] = "val1";
    cdf2spectre_props["va"] = "ampl";
    cdf2spectre_props["vdc"] = "dc";
}

void traits::nstream<spectre_stream>::close(type &stream) { stream.close(); }

void traits::nstream<spectre_stream>::write_header(type &stream,
                                                   const std::vector<std::string> &inc_list,
                                                   bool shell) {
    // set language
    stream.out_file << "simulator lang=spectre" << std::endl;

    if (!shell) {
        if (!inc_list.empty()) {
            for (auto const &fname : inc_list) {
                stream.out_file << "include \"" << util::get_canonical_path(fname).c_str() << '"'
                                << std::endl;
            }
            stream.out_file << std::endl;
        }
    }
}

void traits::nstream<spectre_stream>::write_end(type &stream) {}

void traits::nstream<spectre_stream>::write_cv_header(type &stream, const std::string &name,
                                                      const sch::cellview_info &info, bool shell,
                                                      bool write_subckt) {
    stream.out_file << std::endl << std::endl;
    if (write_subckt) {
        lstream b;
        b << "subckt";
        b << name;
        spectre::get_cv_pins(b, info.in_terms);
        spectre::get_cv_pins(b, info.out_terms);
        spectre::get_cv_pins(b, info.io_terms);

        // write definition line
        b.to_file(stream.out_file, spirit::namespace_cdba{});
    }
}

void traits::nstream<spectre_stream>::write_cv_end(type &stream, const std::string &name,
                                                   bool write_subckt) {
    if (write_subckt) {
        stream.out_file << "ends " << name << std::endl;
    } else {
        stream.out_file << std::endl;
    }
}

void traits::nstream<spectre_stream>::write_instance(type &stream, const std::string &name,
                                                     const sch::instance &inst,
                                                     const sch::cellview_info &info) {
    spirit::ast::name_unit inst_ast = cbag::util::parse_cdba_name_unit(name);
    auto n = inst_ast.size();

    if (n == 1) {
        // normal instance, just write normally
        lstream b;
        b << name;
        spectre::append_nets(b, name, inst, info.in_terms);
        spectre::append_nets(b, name, inst, info.out_terms);
        spectre::append_nets(b, name, inst, info.io_terms);
        spectre::write_instance_cell_name(b.get_back_inserter(), inst, info, stream);
        b.to_file(stream.out_file, spirit::namespace_cdba{});
    } else {
        // arrayed instance, need to split up
        // get name bits of terminals/nets
        auto term_net_vec = spectre::term_net_vec_t();
        spectre::get_term_net_pairs(term_net_vec, name, inst, info.in_terms);
        spectre::get_term_net_pairs(term_net_vec, name, inst, info.out_terms);
        spectre::get_term_net_pairs(term_net_vec, name, inst, info.io_terms);
        // get cell name tokens
        std::vector<std::string> tokens;
        tokens.reserve(2);
        spectre::write_instance_cell_name(std::back_inserter(tokens), inst, info, stream);
        // array instance
        for (decltype(n) inst_idx = 0; inst_idx < n; ++inst_idx) {
            lstream b;
            // write instance name
            b << inst_ast.get_name_bit(inst_idx, true, spirit::namespace_cdba{});
            // write instance nets
            for (const auto &pair : term_net_vec) {
                auto term_n = pair.first;
                auto net_idx = inst_idx * term_n;
                auto stop_idx = net_idx + term_n;
                for (; net_idx < stop_idx; ++net_idx) {
                    b << pair.second[net_idx];
                }
            }
            // write instance cell name
            b << tokens;
            b.to_file(stream.out_file, spirit::namespace_cdba{});
        }
    }
}

void traits::nstream<spectre_stream>::append_netlist(type &stream, const std::string &netlist) {
    stream.out_file << std::endl << std::endl;
    stream.out_file << netlist;
}

} // namespace netlist
} // namespace cbag
