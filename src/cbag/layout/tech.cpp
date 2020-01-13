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

#include <algorithm>
#include <limits>

#include <fmt/core.h>

#include <cbag/layout/tech.h>
#include <cbag/layout/tech_util.h>
#include <cbag/layout/wire_width.h>
#include <cbag/util/binary_iterator.h>

namespace cbag {
namespace layout {

tech::tech() = default;

tech::tech(std::string &&tech_lib, double_t layout_unit, double_t resolution,
           bool use_track_coloring, bool make_pin_obj, space_type sp_sc_type,
           space_type sp_meta_type, level_t grid_bot_layer, lp_lookup &&lp_map,
           via_lookup &&vlookup, sp_map_grp_t &&sp_map_grp, len_map_t &&len_map,
           lp_list_t &&lp_list, lp_list_t &&dum_lp_list, wintv_list_t &&wintv_list,
           level_map_t &&lev_map, color_map_t &&color_map, std::vector<layer_t> &&exc_lp_list,
           bool exc_blockage)
    : tech_lib(std::move(tech_lib)), layout_unit(layout_unit), resolution(resolution),
      use_track_coloring(use_track_coloring), make_pin_obj(make_pin_obj), sp_sc_type(sp_sc_type),
      sp_meta_type_(sp_meta_type), grid_bot_layer(grid_bot_layer), lp_map(std::move(lp_map)),
      vlookup(std::move(vlookup)), sp_map_grp(std::move(sp_map_grp)), len_map(std::move(len_map)),
      lp_list(std::move(lp_list)), dum_lp_list_(std::move(dum_lp_list)),
      wintv_list(std::move(wintv_list)), lev_map(std::move(lev_map)),
      color_map(std::move(color_map)), exc_lp_list_(std::move(exc_lp_list)),
      exc_blockage_(exc_blockage) {}

const std::string &tech::get_tech_lib() const noexcept { return tech_lib; }

double_t tech::get_layout_unit() const noexcept { return layout_unit; }

double_t tech::get_resolution() const noexcept { return resolution; }

bool tech::get_use_track_coloring() const noexcept { return use_track_coloring; }

bool tech::get_make_pin() const noexcept { return make_pin_obj; }

level_t tech::get_bot_level() const noexcept { return grid_bot_layer; }

purp_t tech::get_default_purpose() const { return lp_map.get_default_purpose(); }

purp_t tech::get_pin_purpose() const { return lp_map.get_pin_purpose(); }

bool tech::exclude_is_blockage() const { return exc_blockage_; }

const color_map_t &tech::get_color_map() const { return color_map; }

const std::string &tech::get_layer_name(lay_t lay_id) const {
    return lp_map.get_layer_name(lay_id);
}

const std::string &tech::get_purpose_name(purp_t purp_id) const {
    return lp_map.get_purpose_name(purp_id);
}

std::optional<lay_t> tech::get_layer_id(const std::string &layer) const {
    return lp_map.get_layer_id(layer);
}

std::optional<purp_t> tech::get_purpose_id(const std::string &purpose) const {
    return lp_map.get_purpose_id(purpose);
}

std::optional<level_t> tech::get_level(layer_t key) const {
    std::optional<level_t> ans;
    auto iter = lev_map.find(key);
    if (iter == lev_map.end())
        return ans;
    ans = iter->second;
    return ans;
}

const std::vector<layer_t> &tech::get_lay_purp_list(level_t level, bool is_dummy) const {
    auto idx = static_cast<std::size_t>(level - grid_bot_layer);
    auto &cur_lp_list = is_dummy ? dum_lp_list_ : lp_list;
    if (idx >= cur_lp_list.size())
        throw std::out_of_range("Undefined routing grid level: " + std::to_string(level) +
                                ", is_dummy: " + std::to_string(is_dummy));
    return cur_lp_list[idx];
}

layer_t tech::get_exclude_layer(level_t level) const {
    auto idx = static_cast<std::size_t>(level - grid_bot_layer);
    if (idx >= exc_lp_list_.size())
        throw std::out_of_range("Undefined exclude layer level: " + std::to_string(level));
    return exc_lp_list_[idx];
}

const w_intvs_t &tech::get_width_intervals(level_t level, orientation_2d tr_dir) const {
    auto idx = static_cast<std::size_t>(level - grid_bot_layer);
    if (idx >= wintv_list.size())
        throw std::out_of_range("Undefined routing grid level: " + std::to_string(level));
    return wintv_list[idx][to_int(tr_dir)];
}

cnt_t tech::get_num_colors(level_t level) const { return get_lay_purp_list(level).size(); }

cnt_t tech::get_num_color_levels() const {
    auto n = lp_list.size();
    for (cnt_t idx = 0; idx < n; ++idx) {
        if (lp_list[idx].size() == 1)
            return idx;
    }
    return n;
}

offset_t tech::get_min_space(layer_t key, offset_t width, space_type sp_type, bool even) const {
    space_type search_sp_type = sp_type;
    if (sp_type == space_type::SAME_COLOR) {
        search_sp_type = sp_sc_type;
    } else if (sp_type == space_type::META_WIRE) {
        search_sp_type = sp_meta_type_;
    }
    auto map_iter = sp_map_grp.find(search_sp_type);
    if (map_iter == sp_map_grp.end())
        throw std::out_of_range("Min space not defined for space type: " +
                                std::to_string(static_cast<enum_t>(search_sp_type)));

    const auto &cur_map = map_iter->second;
    auto vec_iter = cur_map.find(key);
    if (vec_iter == cur_map.end())
        return 0;

    const auto &w_sp_list = vec_iter->second;

    for (const auto & [ w_spec, sp ] : w_sp_list) {
        if (width <= w_spec)
            return sp + (sp & static_cast<offset_t>(even));
    }
    auto ans = w_sp_list[w_sp_list.size() - 1].second;
    return ans + (ans & static_cast<offset_t>(even));
}

offset_t tech::get_next_length(layer_t key, orientation_2d tr_dir, offset_t width, offset_t cur_len,
                               bool even) const {
    // first fix area/min length rule
    auto iter = len_map.find(key);
    if (iter == len_map.end())
        throw std::runtime_error(
            fmt::format("Cannot find min length for layer ({}, {})", key.first, key.second));
    cur_len = std::max(cur_len, iter->second.get_min_length(width, even));

    // check valid discrete widths
    auto lev_opt = get_level(key);
    if (!lev_opt) {
        throw std::invalid_argument(
            fmt::format("({}, {}) is not on the routing grid.", key.first, key.second));
    }
    for (const auto & [ wl, wu ] : get_width_intervals(*lev_opt, tr_dir)) {
        if (cur_len < wu) {
            cur_len = std::max(cur_len, wl);
            cur_len += (cur_len & even);
            if (cur_len < wu)
                return cur_len;
        }
    }
    // NOTE: should never get here
    throw std::runtime_error(
        fmt::format("length {} is not in any width intervals on level {}", cur_len, *lev_opt));
}

offset_t tech::get_next_length(level_t level, orientation_2d tr_dir, const wire_width &wire_w,
                               offset_t cur_len, bool even) const {
    auto key = get_test_lay_purp(*this, level);

    // first fix area/min length rule
    auto iter = len_map.find(key);
    if (iter == len_map.end())
        throw std::runtime_error(
            fmt::format("Cannot find min length for layer ({}, {})", key.first, key.second));
    for (auto witer = wire_w.begin_width(), wend = wire_w.end_width(); witer != wend; ++witer) {
        cur_len = std::max(cur_len, iter->second.get_min_length(*witer, even));
    }

    for (const auto & [ wl, wu ] : get_width_intervals(level, tr_dir)) {
        if (cur_len < wu) {
            cur_len = std::max(cur_len, wl);
            cur_len += (cur_len & even);
            if (cur_len < wu)
                return cur_len;
        }
    }
    // NOTE: should never get here
    throw std::runtime_error(
        fmt::format("length {} is not in any width intervals on level {}", cur_len, level));
}

offset_t tech::get_prev_length(layer_t key, orientation_2d tr_dir, offset_t width, offset_t cur_len,
                               bool even) const {
    auto good_len = cur_len + 1;
    auto bin_iter = util::binary_iterator(0, cur_len + 1);
    while (bin_iter.has_next()) {
        auto result = get_next_length(key, tr_dir, width, *bin_iter, even);
        if (result == cur_len)
            return result;
        else if (result < cur_len) {
            good_len = result;
            bin_iter.up();
        } else {
            bin_iter.down();
        }
    }
    if (good_len > cur_len) {
        throw std::runtime_error("ERROR: No solution.");
    }
    return good_len;
}

offset_t tech::get_prev_length(level_t level, orientation_2d tr_dir, const wire_width &wire_w,
                               offset_t cur_len, bool even) const {
    auto good_len = cur_len + 1;
    auto bin_iter = util::binary_iterator(0, cur_len + 1);
    while (bin_iter.has_next()) {
        auto result = get_next_length(level, tr_dir, wire_w, *bin_iter, even);
        if (result == cur_len)
            return result;
        else if (result < cur_len) {
            good_len = result;
            bin_iter.up();
        } else {
            bin_iter.down();
        }
    }
    if (good_len > cur_len) {
        throw std::runtime_error("ERROR: No solution.");
    }
    return good_len;
}

const std::string &tech::get_via_id(direction_1d vdir, layer_t layer, layer_t adj_layer) const {
    return vlookup.get_via_id(vdir, layer, adj_layer);
}

via_lay_purp_t tech::get_via_layer_purpose(const std::string &key) const {
    return vlookup.get_via_layer_purpose(key);
}

bool tech::via_layer_flipped(const std::string &key) const {
    return vlookup.via_layer_flipped(key);
}

via_param tech::get_via_param(vector dim, const std::string &via_id, direction_1d vdir,
                              orientation_2d ex_dir, orientation_2d adj_ex_dir, bool extend) const {
    return vlookup.get_via_param(dim, via_id, vdir, ex_dir, adj_ex_dir, extend);
}

em_specs_t tech::get_metal_em_specs(const std::string &layer, const std::string &purpose,
                                    offset_t width, offset_t length, bool vertical, temp_t dc_temp,
                                    temp_t rms_dt) const {
    constexpr auto inf = std::numeric_limits<double>::infinity();
    return {inf, inf, inf};
}

em_specs_t tech::get_via_em_specs(enum_t layer_dir, const std::string &layer,
                                  const std::string &purpose, const std::string &adj_layer,
                                  const std::string &adj_purpose, offset_t cut_w, offset_t cut_h,
                                  offset_t m_w, offset_t m_l, offset_t adj_m_w, offset_t adj_m_l,
                                  bool array, temp_t dc_temp, temp_t rms_dt) const {
    constexpr auto inf = std::numeric_limits<double>::infinity();
    return {inf, inf, inf};
}

} // namespace layout
} // namespace cbag
