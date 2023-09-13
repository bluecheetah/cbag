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

#include <yaml-cpp/yaml.h>

#include "yaml-cpp/tuple.h"
#include "yaml-cpp/unordered_map.h"

#include <cbag/layout/tech_util.h>
#include <cbag/layout/track_coloring.h>
#include <cbag/layout/wire_width.h>
#include <cbag/yaml/common.h>
#include <cbag/yaml/int_array.h>
#include <cbag/yaml/len_info.h>
#include <cbag/yaml/via_info.h>

namespace cbag {
namespace layout {

namespace yaml {

std::vector<std::pair<offset_t, offset_t>> make_w_sp_vec(const YAML::Node &node) {
    std::vector<std::pair<offset_t, offset_t>> ans;
    ans.reserve(node.size());
    for (const auto &val : node) {
        ans.emplace_back(cbagyaml::get_int<offset_t>(val[0]), cbagyaml::get_int<offset_t>(val[1]));
    }
    return ans;
}

sp_map_t make_space_map(const YAML::Node &parent_node, const char *node_name,
                        const lp_lookup &lp_map) {
    auto node = parent_node[node_name];
    if (!node.IsDefined())
        throw std::runtime_error(
            fmt::format("Cannot find YAML node name {} in tech YAML file.", node_name));
    sp_map_t ans;
    for (const auto &pair : node) {
        auto[lay_name, purp_name] = pair.first.as<std::pair<std::string, std::string>>();
        ans.emplace(layer_t_at(lp_map, lay_name, purp_name), make_w_sp_vec(pair.second));
    }
    return ans;
}

len_map_t make_len_map(const YAML::Node &parent_node, const char *node_name,
                       const lp_lookup &lp_map) {
    auto node = parent_node[node_name];
    if (!node.IsDefined())
        throw std::runtime_error(
            fmt::format("Cannot find YAML node name {} in tech YAML file.", node_name));
    len_map_t ans;
    for (const auto &pair : node) {
        auto[lay_name, purp_name] = pair.first.as<std::pair<std::string, std::string>>();
        ans.emplace(layer_t_at(lp_map, lay_name, purp_name), pair.second.as<len_info>());
    }
    return ans;
}

lp_lookup make_lp_lookup(const YAML::Node &node) {
    auto lay_map = node["layer"].as<lay_map_t>();
    auto purp_map = node["purpose"].as<purp_map_t>();
    auto def_purp = node["default_purpose"].as<std::string>();
    auto pin_purp = node["pin_purpose"].as<std::string>();
    auto label_purp = pin_purp;
    if (YAML::Node label_purpose = node["label_purpose"]) {
        label_purp = label_purpose.as<std::string>();
    }
    return {node["layer"].as<lay_map_t>(), node["purpose"].as<purp_map_t>(), def_purp, pin_purp, label_purp};
}

void _add_flipped_enclosures(vinfo_map_t &info_map, const std::string &name) {
    for (auto &val : info_map) {
        for (auto &vinfo : val.second) {
            if (vinfo.get_name() == name) {
                vinfo.add_flipped_enclosures();
            }
        }
    }
}

void _add_flipped_vias(vinfo_map_t &info_map, const std::string &old_name,
                       const std::string &new_name) {
    for (auto &val : info_map) {
        auto &vinfo_list = val.second;
        auto n = vinfo_list.size();
        for (decltype(n) i = 0; i < n; ++i) {
            if (vinfo_list[i].get_name() == old_name) {
                vinfo_list.push_back(vinfo_list[i].get_flip_xy(new_name));
            }
        }
    }
}

vlayers_t _parse_via_layers(const YAML::Node &node, const lp_lookup &lp) {
    return {layer_t_at(lp, node[0][0].as<std::string>(), node[0][1].as<std::string>()),
            layer_t_at(lp, node[1][0].as<std::string>(), node[1][1].as<std::string>())};
}

via_lookup make_via_lookup(const YAML::Node &node, const lp_lookup &lp) {
    auto lp_map = node["via_layers"].as<vlp_map_t>();
    auto info_map = node["via"].as<vinfo_map_t>();
    auto id_map = vid_map_t();

    // update enclosure rules for square vias
    for (const auto &entry : node["via_square_list"]) {
        _add_flipped_enclosures(info_map, entry.as<std::string>());
    }
    // add symmetric vias
    for (const auto &entry : node["via_symmetry_list"]) {
        _add_flipped_vias(info_map, entry[0].as<std::string>(), entry[1].as<std::string>());
    }

    for (const auto &entry : node["via_id"]) {
        id_map.emplace(_parse_via_layers(entry.first, lp), entry.second.as<std::string>());
    }

    auto flip_set = std::unordered_set<std::string>();
    for (const auto &entry : node["flipped_vias"]) {
        flip_set.insert(entry.as<std::string>());
    }

    // correct via layer order
    auto flip_end = flip_set.end();
    for (auto & [ key, via_lp ] : lp_map) {
        if (flip_set.find(key) != flip_end) {
            std::swap(std::get<0>(via_lp), std::get<2>(via_lp));
        }
    }

    return {std::move(lp_map), std::move(id_map), std::move(info_map), std::move(flip_set)};
}

} // namespace yaml

lp_list_t _make_lp_list(const lp_lookup &lp_map,
                        const std::vector<std::vector<std::pair<std::string, std::string>>> &vec,
                        level_map_t &lev_map, level_t bot_layer) {
    lp_list_t lp_list;
    lp_list.reserve(vec.size());
    for (const auto &cur_lp_vec : vec) {
        std::vector<layer_t> new_lp_vec;
        new_lp_vec.reserve(cur_lp_vec.size());
        for (const auto & [ lay_str, purp_str ] : cur_lp_vec) {
            auto lid = lp_map.get_layer_id(lay_str);
            if (!lid)
                throw std::out_of_range("Cannot find layer ID for layer: " + lay_str);

            auto pid = lp_map.get_purpose_id(purp_str);
            if (!pid)
                throw std::out_of_range("Cannot find purpose ID for purpose: " + purp_str);
            new_lp_vec.push_back(std::make_pair(*lid, *pid));
        }
        lp_list.push_back(new_lp_vec);
    }

    for (const auto &lay_purp_vec : lp_list) {
        for (const auto &lp : lay_purp_vec) {
            lev_map.emplace(lp, bot_layer);
        }
        ++bot_layer;
    }
    return lp_list;
}

tech make_tech(const std::string &fname) {
    if (fname.empty())
        return {};

    auto node = YAML::LoadFile(fname);

    auto tech_lib = node["tech_lib"].as<std::string>();
    auto layout_unit = node["layout_unit"].as<double_t>();
    auto resolution = node["resolution"].as<double_t>();
    auto use_track_coloring = node["use_track_coloring"].as<bool>();
    auto make_pin_obj = node["make_pin_obj"].as<bool>();

    auto gds_resolution = resolution;
    auto gds_res_node = node["gds_resolution"];
    if (gds_res_node.IsDefined()) {
        gds_resolution = gds_res_node.as<double_t>();
    }

    auto lp_map = yaml::make_lp_lookup(node);
    auto vlookup = yaml::make_via_lookup(node, lp_map);

    // populate space map
    sp_map_grp_t sp_map_grp;
    sp_map_grp.emplace(space_type::DIFF_COLOR, yaml::make_space_map(node, "sp_min", lp_map));
    sp_map_grp.emplace(space_type::LINE_END, yaml::make_space_map(node, "sp_le_min", lp_map));

    space_type sp_sc_type = space_type::DIFF_COLOR;
    if (node["sp_sc_min"].IsDefined()) {
        sp_map_grp.emplace(space_type::SAME_COLOR, yaml::make_space_map(node, "sp_sc_min", lp_map));
        sp_sc_type = space_type::SAME_COLOR;
    }

    space_type sp_meta_type = sp_sc_type;
    if (node["sp_meta_min"].IsDefined()) {
        sp_map_grp.emplace(space_type::META_WIRE,
                           yaml::make_space_map(node, "sp_meta_min", lp_map));
        sp_meta_type = space_type::META_WIRE;
    }

    // populate len_map
    auto len_map = yaml::make_len_map(node, "len_min", lp_map);

    // get level-to-layer/purpose mapping
    auto[grid_bot_layer, lp_vec] =
        cbagyaml::int_map_to_vec<std::vector<std::pair<std::string, std::string>>>(
            node["lay_purp_list"]);
    auto[dum_bot_layer, dum_lp_vec] =
        cbagyaml::int_map_to_vec<std::vector<std::pair<std::string, std::string>>>(
            node["dum_lay_purp_list"]);
    auto exc_lp_vec = std::get<1>(
        cbagyaml::int_map_to_vec<std::pair<std::string, std::string>>(node["metal_exclude_table"]));

    auto exc_lp_list = std::vector<layer_t>();
    exc_lp_list.reserve(exc_lp_vec.size());
    for (const auto & [ lay_str, purp_str ] : exc_lp_vec) {
        auto lid = lp_map.get_layer_id(lay_str);
        if (!lid)
            throw std::out_of_range("Cannot find layer ID for layer: " + lay_str);

        auto pid = lp_map.get_purpose_id(purp_str);
        if (!pid)
            throw std::out_of_range("Cannot find purpose ID for purpose: " + purp_str);
        exc_lp_list.push_back(std::make_pair(*lid, *pid));
    }

    auto lev_map = level_map_t();
    auto lp_list = _make_lp_list(lp_map, lp_vec, lev_map, grid_bot_layer);
    auto dum_lp_list = _make_lp_list(lp_map, dum_lp_vec, lev_map, dum_bot_layer);

    auto num_layers = lp_list.size();
    wintv_list_t wintv_list;
    auto w_node = node["width_intervals"];
    if (!w_node) {
        // no width intervals; default to all widths are legal
        auto w_intvs = w_intvs_t(
            std::vector<std::array<offset_t, 2>>(1, std::array<offset_t, 2>{1, COORD_MAX}));
        wintv_list.resize(num_layers, std::array<w_intvs_t, 2>{w_intvs, w_intvs});
    } else {
        auto wintv_tmp = std::get<1>(
            cbagyaml::int_map_to_vec<std::array<std::vector<std::array<offset_t, 2>>, 2>>(w_node));
        for (const auto & [ w_intvs_x, w_intvs_y ] : wintv_tmp) {
            wintv_list.push_back(std::array<w_intvs_t, 2>{w_intvs_t(), w_intvs_t()});
            for (const auto &intv : w_intvs_x) {
                if (!wintv_list.back()[0].emplace(false, false, false, intv)) {
                    throw std::runtime_error("invalid width interval list");
                }
            }
            for (const auto &intv : w_intvs_y) {
                if (!wintv_list.back()[1].emplace(false, false, false, intv)) {
                    throw std::runtime_error("invalid width interval list");
                }
            }
        }
    }

    // color mapping
    auto color_map = color_map_t();
    auto color_map_node = node["colors"];
    if (color_map_node.IsDefined()) {
        color_map = color_map_node.as<color_map_t>();
    }

    auto exc_blockage = false;
    auto exc_blk_node = node["exclude_is_blockage"];
    if (exc_blk_node.IsDefined()) {
        exc_blockage = exc_blk_node.as<bool>();
    }

    return {std::move(tech_lib),
            layout_unit,
            resolution,
            gds_resolution,
            use_track_coloring,
            make_pin_obj,
            sp_sc_type,
            sp_meta_type,
            grid_bot_layer,
            std::move(lp_map),
            std::move(vlookup),
            std::move(sp_map_grp),
            std::move(len_map),
            std::move(lp_list),
            std::move(dum_lp_list),
            std::move(wintv_list),
            std::move(lev_map),
            std::move(color_map),
            std::move(exc_lp_list),
            exc_blockage};
}

track_coloring make_track_coloring(const tech &t) {
    auto n_color_lev = t.get_num_color_levels();
    auto bot_level = t.get_bot_level();
    std::vector<color_info> color_data;
    color_data.reserve(n_color_lev);
    for (decltype(n_color_lev) idx = 0; idx < n_color_lev; ++idx) {
        auto modulus = t.get_num_colors(idx + bot_level);
        color_data.emplace_back(modulus, 1, 0);
    }
    return track_coloring(bot_level, std::move(color_data));
}

lay_t layer_id_at(const tech &t, const std::string &layer) {
    auto ans = t.get_layer_id(layer);
    if (!ans)
        throw std::out_of_range(fmt::format("Cannot find layer: {}", layer));
    return *ans;
}

purp_t purpose_id_at(const tech &t, const std::string &purpose) {
    auto ans = t.get_purpose_id(purpose);
    if (!ans)
        throw std::out_of_range(fmt::format("Cannot find purpose: {}", purpose));
    return *ans;
}

lay_t layer_id_at(const lp_lookup &lp, const std::string &layer) {
    auto ans = lp.get_layer_id(layer);
    if (!ans)
        throw std::out_of_range(fmt::format("Cannot find layer: {}", layer));
    return *ans;
}

purp_t purpose_id_at(const lp_lookup &lp, const std::string &purpose) {
    auto ans = lp.get_purpose_id(purpose);
    if (!ans)
        throw std::out_of_range(fmt::format("Cannot find purpose: {}", purpose));
    return *ans;
}

layer_t layer_t_at(const tech &t, const std::string &layer, const std::string &purpose) {
    return {layer_id_at(t, layer), purpose_id_at(t, purpose)};
}

layer_t layer_t_at(const lp_lookup &lp, const std::string &layer, const std::string &purpose) {
    return {layer_id_at(lp, layer), purpose_id_at(lp, purpose)};
}

layer_t get_test_lay_purp(const tech &t, level_t level) { return t.get_lay_purp_list(level)[0]; }

layer_t get_layer_t(const tech &t, const track_coloring &tr_colors, level_t level, htr_t htr,
                    bool is_dummy) {
    auto idx = tr_colors.get_htr_parity(level, htr);
    return t.get_lay_purp_list(level, is_dummy)[idx];
}

const std::string &get_label_purpose_name(const tech &t) {
    return t.get_purpose_name(t.get_label_purpose());
}

const std::string &get_pin_purpose_name(const tech &t) {
    return t.get_purpose_name(t.get_pin_purpose());
}

const std::string &get_default_purpose_name(const tech &t) {
    return t.get_purpose_name(t.get_default_purpose());
}

offset_t get_next_length(const tech &t, const std::string &layer, const std::string &purpose,
                         orientation_2d tr_dir, offset_t width, offset_t cur_len, bool even) {
    return t.get_next_length(layer_t_at(t, layer, purpose), tr_dir, width, cur_len, even);
}

offset_t get_prev_length(const tech &t, const std::string &layer, const std::string &purpose,
                         orientation_2d tr_dir, offset_t width, offset_t cur_len, bool even) {
    return t.get_prev_length(layer_t_at(t, layer, purpose), tr_dir, width, cur_len, even);
}

offset_t get_min_space(const tech &t, level_t level, const wire_width &wire_w, space_type sp_type,
                       bool even) {
    auto key = get_test_lay_purp(t, level);

    switch (sp_type) {
    case space_type::LINE_END: {
        offset_t ans = 0;
        auto iter_stop = wire_w.end_width();
        for (auto iter = wire_w.begin_width(); iter != iter_stop; ++iter) {
            ans = std::max(ans, t.get_min_space(key, *iter, space_type::LINE_END, even));
        }
        return ans;
    }
    default:
        return t.get_min_space(key, wire_w.get_edge_wire_width(), sp_type, even);
    }
}

em_specs_t get_metal_em_specs(const tech &t, layer_t key, offset_t width, offset_t length,
                              bool vertical, temp_t dc_temp, temp_t rms_dt) {
    auto &layer = t.get_layer_name(key.first);
    auto &purpose = t.get_purpose_name(key.second);
    return t.get_metal_em_specs(layer, purpose, width, length, vertical, dc_temp, rms_dt);
}

em_specs_t get_metal_em_specs(const tech &t, level_t level, const wire_width &wire_w,
                              offset_t length, bool vertical, temp_t dc_temp, temp_t rms_dt) {
    auto key = get_test_lay_purp(t, level);

    double idc = 0, iac_rms = 0, iac_peak = 0;
    for (auto witer = wire_w.begin_width(), wend = wire_w.end_width(); witer != wend; ++witer) {
        auto[idc_cur, iac_rms_cur, iac_peak_cur] =
            cbag::layout::get_metal_em_specs(t, key, *witer, length, vertical, dc_temp, rms_dt);
        idc += idc_cur;
        iac_rms += iac_rms_cur;
        iac_peak += iac_peak_cur;
    }

    return {idc, iac_rms, iac_peak};
}

em_specs_t get_via_em_specs(const tech &t, direction_1d vdir, layer_t key, layer_t adj_key,
                            std::array<offset_t, 2> cut_dim, std::array<offset_t, 2> m_dim,
                            std::array<offset_t, 2> adj_m_dim, bool array, temp_t dc_temp,
                            temp_t rms_dt) {
    auto &layer = t.get_layer_name(key.first);
    auto &purpose = t.get_purpose_name(key.second);
    auto &adj_layer = t.get_layer_name(adj_key.first);
    auto &adj_purpose = t.get_purpose_name(adj_key.second);
    return t.get_via_em_specs(to_int(vdir), layer, purpose, adj_layer, adj_purpose, cut_dim[0],
                              cut_dim[1], m_dim[0], m_dim[1], adj_m_dim[0], adj_m_dim[1], array,
                              dc_temp, rms_dt);
}

} // namespace layout
} // namespace cbag
