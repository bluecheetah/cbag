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

#ifndef CBAG_LAYOUT_TECH_UTIL_H
#define CBAG_LAYOUT_TECH_UTIL_H

#include <string>

#include <cbag/common/layer_t.h>
#include <cbag/layout/tech.h>

namespace cbag {
namespace layout {

class wire_width;
class track_coloring;

tech make_tech(const std::string &fname);

track_coloring make_track_coloring(const tech &t);

lay_t layer_id_at(const tech &t, const std::string &layer);

purp_t purpose_id_at(const tech &t, const std::string &purpose);

layer_t layer_t_at(const tech &t, const std::string &layer, const std::string &purpose);

layer_t layer_t_at(const lp_lookup &lp, const std::string &layer, const std::string &purpose);

layer_t get_test_lay_purp(const tech &t, level_t level);

layer_t get_layer_t(const tech &t, const track_coloring &tr_colors, level_t level, htr_t htr,
                    bool is_dummy = false);

const std::string &get_pin_purpose_name(const tech &t);

const std::string &get_default_purpose_name(const tech &t);

offset_t get_next_length(const tech &t, const std::string &layer, const std::string &purpose,
                         orientation_2d tr_dir, offset_t width, offset_t cur_len, bool even);

offset_t get_prev_length(const tech &t, const std::string &layer, const std::string &purpose,
                         orientation_2d tr_dir, offset_t width, offset_t cur_len, bool even);

offset_t get_min_space(const tech &t, level_t level, const wire_width &wire_w, space_type sp_type,
                       bool even);

em_specs_t get_metal_em_specs(const tech &t, layer_t key, offset_t width, offset_t length,
                              bool vertical, temp_t dc_temp, temp_t rms_dt);

em_specs_t get_metal_em_specs(const tech &t, level_t level, const wire_width &wire_w,
                              offset_t length, bool vertical, temp_t dc_temp, temp_t rms_dt);

em_specs_t get_via_em_specs(const tech &t, direction_1d vdir, layer_t key, layer_t adj_key,
                            std::array<offset_t, 2> cut_dim, std::array<offset_t, 2> m_dim,
                            std::array<offset_t, 2> adj_m_dim, bool array, temp_t dc_temp,
                            temp_t rms_dt);

} // namespace layout
} // namespace cbag

#endif
