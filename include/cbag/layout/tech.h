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

#ifndef CBAG_LAYOUT_TECH_H
#define CBAG_LAYOUT_TECH_H

#include <string>
#include <tuple>
#include <unordered_map>
#include <utility>
#include <vector>

#include <boost/container_hash/hash.hpp>

#include <cbag/common/layer_t.h>
#include <cbag/common/typedefs.h>
#include <cbag/enum/space_type.h>
#include <cbag/layout/len_info.h>
#include <cbag/layout/lp_lookup.h>
#include <cbag/layout/via_lookup.h>
#include <cbag/polygon/enum.h>

namespace cbag {
namespace layout {

using sp_map_t =
    std::unordered_map<layer_t, std::vector<std::pair<offset_t, offset_t>>, boost::hash<layer_t>>;
using sp_map_grp_t = std::unordered_map<space_type, sp_map_t>;
using len_map_t = std::unordered_map<layer_t, len_info, boost::hash<layer_t>>;
using lp_list_t = std::vector<std::vector<layer_t>>;
using w_intvs_t = std::vector<std::array<offset_t, 2>>;
using wintv_list_t = std::vector<std::array<w_intvs_t, 2>>;
using level_map_t = std::unordered_map<layer_t, level_t, boost::hash<layer_t>>;
using color_map_t = std::unordered_map<lay_t, std::pair<lay_t, scnt_t>>;

class tech {
  private:
    std::string tech_lib = "";
    double_t layout_unit = 1e-6;
    double_t resolution = 0.001;
    bool use_track_coloring = false;
    bool make_pin_obj = true;
    space_type sp_sc_type = space_type::DIFF_COLOR;
    level_t grid_bot_layer = 0;
    lp_lookup lp_map;
    via_lookup vlookup;
    sp_map_grp_t sp_map_grp;
    len_map_t len_map;
    lp_list_t lp_list;
    wintv_list_t wintv_list;
    level_map_t lev_map;
    color_map_t color_map;

  public:
    tech();

    tech(std::string &&tech_lib, double_t layout_unit, double_t resolution, bool use_track_coloring,
         bool make_pin_obj, space_type sp_sc_type, level_t grid_bot_layer, lp_lookup &&lp_map,
         via_lookup &&vlookup, sp_map_grp_t &&sp_map_grp, len_map_t &&len_map, lp_list_t &&lp_list,
         wintv_list_t &&wintv_list, level_map_t &&lev_map, color_map_t &&color_map);

    virtual ~tech() = default;
    tech(tech &&) = default;
    tech &operator=(tech &&) = default;
    // disable copying; tech object should be singleton
    tech(const tech &) = delete;
    tech &operator=(const tech &) = delete;

    const std::string &get_tech_lib() const noexcept;

    double_t get_layout_unit() const noexcept;

    double_t get_resolution() const noexcept;

    bool get_use_track_coloring() const noexcept;

    bool get_make_pin() const noexcept;

    purp_t get_default_purpose() const;

    purp_t get_pin_purpose() const;

    const color_map_t &get_color_map() const;

    const std::string &get_layer_name(lay_t lay_id) const;

    const std::string &get_purpose_name(purp_t purp_id) const;

    std::optional<lay_t> get_layer_id(const std::string &layer) const;

    std::optional<purp_t> get_purpose_id(const std::string &purpose) const;

    std::optional<level_t> get_level(layer_t key) const;

    const std::vector<layer_t> &get_lay_purp_list(level_t level) const;

    const std::vector<std::array<offset_t, 2>> &get_width_intervals(level_t level,
                                                                    orientation_2d tr_dir) const;

    cnt_t get_num_colors(level_t level) const;

    cnt_t get_num_color_levels() const;

    offset_t get_min_space(layer_t key, offset_t width, space_type sp_type, bool even) const;

    offset_t get_min_length(layer_t key, offset_t width, bool even) const;

    const std::string &get_via_id(direction_1d vdir, layer_t layer, layer_t adj_layer) const;

    via_lay_purp_t get_via_layer_purpose(const std::string &key) const;

    via_param get_via_param(vector dim, const std::string &via_id, direction_1d vdir,
                            orientation_2d ex_dir, orientation_2d adj_ex_dir, bool extend) const;

    virtual em_specs_t get_metal_em_specs(const std::string &layer, const std::string &purpose,
                                          offset_t width, offset_t length, bool vertical,
                                          temp_t dc_temp, temp_t rms_dt) const;

    virtual em_specs_t get_via_em_specs(enum_t layer_dir, const std::string &layer,
                                        const std::string &purpose, const std::string &adj_layer,
                                        const std::string &adj_purpose, offset_t cut_w,
                                        offset_t cut_h, offset_t m_w, offset_t m_l,
                                        offset_t adj_m_w, offset_t adj_m_l, bool array,
                                        temp_t dc_temp, temp_t rms_dt) const;
};

} // namespace layout
} // namespace cbag

#endif
