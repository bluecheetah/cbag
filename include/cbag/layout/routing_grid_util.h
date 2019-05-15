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

#ifndef CBAG_LAYOUT_ROUTING_GRID_UTIL_H
#define CBAG_LAYOUT_ROUTING_GRID_UTIL_H

#include <array>

#include <cbag/common/layer_t.h>
#include <cbag/common/typedefs.h>
#include <cbag/layout/polygons.h>
#include <cbag/layout/routing_grid.h>
#include <cbag/polygon/enum.h>

namespace cbag {

namespace layout {

class wire_width;

routing_grid make_grid(std::shared_ptr<const tech> tech_ptr, const std::string &fname);

std::array<offset_t, 2> get_margins(const routing_grid &grid, layer_t key, level_t lev,
                                    const rect_t &obj);
std::array<offset_t, 2> get_margins(const routing_grid &grid, layer_t key, level_t lev,
                                    const poly_90_t &obj);
std::array<offset_t, 2> get_margins(const routing_grid &grid, layer_t key, level_t lev,
                                    const poly_45_t &obj);
std::array<offset_t, 2> get_margins(const routing_grid &grid, layer_t key, level_t lev,
                                    const poly_t &obj);

std::array<offset_t, 2> get_via_extensions(const routing_grid &grid, direction_1d vdir,
                                           level_t level, cnt_t ntr, cnt_t adj_ntr);

std::array<offset_t, 2> get_via_extensions_dim(const routing_grid &grid, direction_1d vdir,
                                               level_t level, offset_t dim, offset_t adj_dim);

std::array<offset_t, 2> get_via_extensions_dim_tr(const routing_grid &grid, direction_1d vdir,
                                                  level_t level, offset_t dim, cnt_t adj_ntr);

level_t get_lower_orthogonal_level(const routing_grid &grid, level_t level);

bool block_defined_at(const routing_grid &grid, level_t level);

std::array<offset_t, 2> get_top_track_pitches(const routing_grid &grid, level_t level);

htr_t get_sep_htr(const routing_grid &grid, level_t level, cnt_t ntr1, cnt_t ntr2, bool same_color);

htr_t get_line_end_sep_htr(const routing_grid &grid, direction_1d vdir, level_t le_level, cnt_t ntr,
                           cnt_t adj_ntr);

std::array<offset_t, 2> get_blk_size(const routing_grid &grid, level_t level, bool include_private,
                                     std::array<bool, 2> half_blk);

layer_t get_layer_t(const routing_grid &grid, level_t level, htr_t htr);

std::array<offset_t, 2> get_wire_bounds(const routing_grid &grid, level_t level, htr_t htr,
                                        cnt_t ntr);

em_specs_t get_wire_em_specs(const routing_grid &grid, level_t level, cnt_t ntr, offset_t length,
                             bool vertical, temp_t dc_temp, temp_t rms_dt);

em_specs_t get_via_em_specs(const routing_grid &grid, direction_1d vdir, level_t level,
                            const wire_width &wire_w, offset_t length, const wire_width &adj_wire_w,
                            offset_t adj_length, temp_t dc_temp, temp_t rms_dt);

cnt_t get_min_num_tr(const routing_grid &grid, level_t level, double idc, double iac_rms,
                     double iac_peak, offset_t length, cnt_t bot_ntr, cnt_t top_ntr, temp_t dc_temp,
                     temp_t rms_dt);

} // namespace layout
} // namespace cbag

#endif
