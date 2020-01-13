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

#ifndef CBAG_LAYOUT_CELLVIEW_UTIL_H
#define CBAG_LAYOUT_CELLVIEW_UTIL_H

#include <cbag/enum/min_len_mode.h>
#include <cbag/layout/cellview.h>

namespace cbag {
namespace layout {

class track_id;

template <typename T> class cv_obj_ref;

box_t get_bbox(const cellview &cv, const std::string &layer, const std::string &purpose);

void add_pin(cellview &cv, const std::string &layer, const std::string &net,
             const std::string &label, const box_t &bbox);

void add_pin_arr(cellview &cv, const std::string &net, const std::string &label,
                 const track_id &tid, coord_t lower, coord_t upper);

cv_obj_ref<via_wrapper> add_via(const std::shared_ptr<cellview> &cv_ptr, transformation xform,
                                std::string via_id, const via_param &params, bool add_layers,
                                bool commit);

void add_via_arr(cellview &cv, const transformation &xform, const std::string &via_id,
                 const via_param &params, bool add_layers, std::array<cnt_t, 2> num_arr,
                 std::array<offset_t, 2> sp_arr);

std::array<std::array<coord_t, 2>, 2> add_via_on_intersections(cellview &cv, const track_id &tid1,
                                                               const track_id &tid2,
                                                               std::array<coord_t, 2> coord1,
                                                               std::array<coord_t, 2> coord2,
                                                               bool extend, bool contain);

std::array<std::array<coord_t, 2>, 2>
connect_box_track(cellview &cv, direction_1d vdir, layer_t key, const box_t &box,
                  std::array<cnt_t, 2> num, std::array<offset_t, 2> sp, const track_id &tid,
                  const std::array<std::optional<coord_t>, 2> &w_ext,
                  const std::array<std::optional<coord_t>, 2> &tr_ext, min_len_mode mode);

std::array<std::array<coord_t, 2>, 2> connect_warr_track(cellview &cv, const track_id &w_tid,
                                                         const track_id &tid, coord_t w_lower,
                                                         coord_t w_upper);

void add_label(cellview &cv, const std::string &layer, const std::string &purpose,
               transformation xform, std::string label, offset_t text_h);

cv_obj_ref<instance> add_prim_instance(const std::shared_ptr<cellview> &cv_ptr, std::string lib,
                                       std::string cell, std::string view, std::string name,
                                       transformation xform, cnt_t nx, cnt_t ny, offset_t spx,
                                       offset_t spy, bool commit);

cv_obj_ref<instance> add_instance(const std::shared_ptr<cellview> &cv_ptr,
                                  const std::shared_ptr<const cellview> &master, std::string name,
                                  transformation xform, cnt_t nx, cnt_t ny, offset_t spx,
                                  offset_t spy, bool commit);

} // namespace layout
} // namespace cbag

#endif
