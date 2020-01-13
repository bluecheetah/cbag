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
#include <cbag/common/vector.h>
#include <cbag/layout/via_param_util.h>
#include <cbag/layout/via_util.h>

namespace cbag {
namespace layout {

box_t get_via_box(const transformation &xform, const via_param &params, const vector &offset,
                  const vector &enc) {
    auto via_w = get_arr_dim(params, orientation_2d::HORIZONTAL);
    auto via_h = get_arr_dim(params, orientation_2d::VERTICAL);

    auto xl = static_cast<coord_t>(offset[0] - (via_w / 2) - enc[0]);
    auto yl = static_cast<coord_t>(offset[1] - (via_h / 2) - enc[1]);
    auto xh = static_cast<coord_t>(xl + via_w + 2 * enc[0]);
    auto yh = static_cast<coord_t>(yl + via_h + 2 * enc[1]);

    return get_transform(box_t(xl, yl, xh, yh), xform);
}

box_t get_bot_box(const via &v) {
    auto &param = v.get_params();
    return get_via_box(v.xform, param, param.off[0], param.enc[0]);
}

box_t get_top_box(const via &v) {
    auto &param = v.get_params();
    return get_via_box(v.xform, param, param.off[1], param.enc[1]);
}

} // namespace layout
} // namespace cbag
