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

#include <cbag/util/overload.h>

#include <cbag/gdsii/parse_map.h>
#include <cbag/gdsii/read.h>
#include <cbag/gdsii/read_util.h>

namespace cbag {
namespace gdsii {

gds_rlookup::gds_rlookup() = default;

gds_rlookup::gds_rlookup(const std::string &lay_fname, const std::string &obj_fname,
                         const layout::tech &t) {
    process_layer_map(lay_fname, t, [this](layer_t &&lay, gds_layer_t &&glay) {
        lay_map_.emplace(std::move(glay), std::move(lay));
    });

    process_object_map(obj_fname, t,
                       [this](boundary_type btype, gds_layer_t &&glay) {
                           bnd_map_.emplace(std::move(glay), btype);
                       },
                       [this](blockage_t &&btype, gds_layer_t &&glay) {
                           blk_map_.emplace(std::move(glay), std::move(btype));
                       });
}

std::variant<layer_t, boundary_type, blockage_t, bool>
gds_rlookup::get_mapping(gds_layer_t key) const {
    auto iter = lay_map_.find(key);
    if (iter != lay_map_.end())
        return iter->second;

    auto iter2 = bnd_map_.find(key);
    if (iter2 != bnd_map_.end())
        return iter2->second;

    auto iter3 = blk_map_.find(key);
    if (iter3 != blk_map_.end())
        return iter3->second;

    return false;
}

layer_t gds_rlookup::get_layer_t(gds_layer_t key) const {
    auto iter = lay_map_.find(key);
    if (iter == lay_map_.end())
        throw std::runtime_error(fmt::format("Cannot find mapping for GDS layer/purpose: ({}, {})",
                                             key.first, key.second));
    return iter->second;
}

std::string read_gds_start(spdlog::logger &logger, std::istream &stream) {
    read_header(logger, stream);
    read_lib_begin(logger, stream);
    auto ans = read_lib_name(logger, stream);
    read_units(logger, stream);
    return ans;
}

void add_object(spdlog::logger &logger, layout::cellview &ans, gds_layer_t &&gds_key,
                layout::poly_t &&poly, const gds_rlookup &rmap) {
    auto map_val = rmap.get_mapping(gds_key);
    std::visit(
        overload{
            [&ans, &poly](layer_t k) { ans.add_shape(k, poly); },
            [&ans, &poly](boundary_type k) {
                auto bnd = layout::boundary(k);
                bnd.set(poly.begin(), poly.end());
                ans.add_object(std::move(bnd));
            },
            [&ans, &poly](blockage_t k) {
                auto blk = layout::blockage(k.second, k.first);
                blk.set(poly.begin(), poly.end());
                ans.add_object(std::move(blk));
            },
            [&logger, &gds_key](bool k) {
                logger.warn("Cannot find mapping for GDS layer/purpose: ({}, {}), skipping.",
                            gds_key.first, gds_key.second);
            },

        },
        map_val);
}

std::tuple<std::string, std::shared_ptr<layout::cellview>> read_lay_cellview(
    spdlog::logger &logger, std::istream &stream, const std::string &lib_name,
    const std::shared_ptr<const layout::routing_grid> &g,
    const std::shared_ptr<const layout::track_coloring> &colors, const gds_rlookup &rmap,
    const std::unordered_map<std::string, std::shared_ptr<const layout::cellview>> &master_map) {
    auto cell_name = read_struct_name(logger, stream);

    logger.info("GDS cellview name: " + cell_name);

    auto cv_ptr = std::make_shared<layout::cellview>(g, colors, cell_name);
    auto resolution = g->get_tech()->get_resolution();
    auto inst_cnt = static_cast<std::size_t>(0);
    while (true) {
        auto[rtype, rsize] = read_record_header(stream);
        switch (rtype) {
        case record_type::TEXT: {
            logger.info("Reading layout text.");
            auto[gds_key, xform, text, text_h_dbl] = read_text(logger, stream);
            auto text_h = static_cast<offset_t>(text_h_dbl / resolution);
            cv_ptr->add_label(rmap.get_layer_t(gds_key), std::move(xform), std::move(text), text_h);
            break;
        }
        case record_type::SREF:
            logger.info("Reading layout instance.");
            cv_ptr->add_object(read_instance(logger, stream, inst_cnt, master_map));
            break;
        case record_type::AREF:
            logger.info("Reading layout array instance.");
            cv_ptr->add_object(read_arr_instance(logger, stream, inst_cnt, master_map));
            break;
        case record_type::BOX: {
            logger.info("Reading layout box.");
            auto[gds_key, poly] = read_box(logger, stream);
            add_object(logger, *cv_ptr, std::move(gds_key), std::move(poly), rmap);
            break;
        }
        case record_type::BOUNDARY: {
            logger.info("Reading layout boundary.");
            auto[gds_key, poly] = read_boundary(logger, stream);
            add_object(logger, *cv_ptr, std::move(gds_key), std::move(poly), rmap);
            break;
        }
        case record_type::PATH: {
            auto gds_key = read_path(logger, stream);
            auto lay_t = rmap.get_layer_t(gds_key);
            logger.warn("Found PATH on layer ({}, {}) in gds, which is not supported.  Skipping.",
                        lay_t.first, lay_t.second);
            break;
        }
        case record_type::ENDSTR:
            logger.info("Finish reading GDS cellview.");
            return {std::move(cell_name), std::move(cv_ptr)};
        default: {
            auto rec_str = to_string(rtype);
            throw std::runtime_error("Unsupported record type while reading cellview: " + rec_str);
        }
        }
    }
}

} // namespace gdsii
} // namespace cbag
