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

#include <cbag/common/box_t.h>
#include <cbag/layout/cellview_poly.h>
#include <cbag/layout/grid_object.h>
#include <cbag/layout/routing_grid_util.h>
#include <cbag/layout/tech_util.h>
#include <cbag/layout/track_info.h>

namespace cbag {
namespace layout {

shape_ref<box_t> add_rect(const std::shared_ptr<cellview> &cv_ptr, const std::string &layer,
                          const std::string &purpose, box_t bbox, bool commit) {

    return {cv_ptr, layer_t_at(*(cv_ptr->get_tech()), layer, purpose), std::move(bbox), commit};
}

void add_rect_arr(cellview &cv, layer_t &key, const box_t &box, std::array<cnt_t, 2> num,
                  std::array<offset_t, 2> sp) {
    box_t box_copy(box);
    for (cnt_t xidx = 0; xidx < num[0]; ++xidx, move_by(box_copy, sp[0], 0)) {
        auto tot_dy = num[1] * sp[1];
        for (cnt_t yidx = 0; yidx < num[1]; ++yidx, move_by(box_copy, 0, sp[1])) {
            cv.add_shape(key, box_copy);
        }
        move_by(box_copy, 0, -tot_dy);
    }
}

void add_rect_arr(cellview &cv, const std::string &layer, const std::string &purpose,
                  const box_t &box, cnt_t nx, cnt_t ny, offset_t spx, offset_t spy) {
    auto key = layer_t_at(*(cv.get_tech()), layer, purpose);
    add_rect_arr(cv, key, box, {nx, ny}, {spx, spy});
}

void add_warr(cellview &cv, const wire_array &warr) {
    cv.add_warr(warr.get_track_id_ref(), warr.get_coord());
}

} // namespace layout
} // namespace cbag
