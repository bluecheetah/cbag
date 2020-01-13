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
#include <cbag/common/transformation_util.h>
#include <cbag/layout/via_param_util.h>

namespace cbag {
namespace layout {

bool empty(const via_param &p) { return p.num[0] == 0; }

offset_t get_arr_dim(const via_param &p, orientation_2d orient) {
    auto dir_idx = to_int(orient);
    return p.num[dir_idx] * (p.cut_dim[dir_idx] + p.cut_spacing[dir_idx]) - p.cut_spacing[dir_idx];
}

offset_t get_metal_dim(const via_param &p, orientation_2d orient, direction_1d vdir) {
    return get_arr_dim(p, orient) + 2 * p.enc[to_int(vdir)][to_int(orient)];
}

box_t get_box(const via_param &p, const transformation &xform, direction_1d vdir) {
    auto w2 = get_metal_dim(p, orientation_2d::HORIZONTAL, vdir) / 2;
    auto h2 = get_metal_dim(p, orientation_2d::VERTICAL, vdir) / 2;
    auto [x, y] = xform.offset();
    return {x - w2, y - h2, x + w2, y + h2};
}

std::array<offset_t, 2> get_via_extensions(const via_param &p, vector dim, direction_1d vdir,
                                           orientation_2d ex_dir, orientation_2d adj_ex_dir) {

    std::array<offset_t, 2> ans;
    auto didx = to_int(vdir);
    ans[didx] = (get_metal_dim(p, ex_dir, vdir) - dim[to_int(ex_dir)]) / 2;
    ans[1 - didx] = (get_metal_dim(p, adj_ex_dir, flip(vdir)) - dim[to_int(adj_ex_dir)]) / 2;
    return ans;
}

} // namespace layout
} // namespace cbag
