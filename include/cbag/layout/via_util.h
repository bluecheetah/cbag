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

#ifndef CBAG_LAYOUT_VIA_UTIL_H
#define CBAG_LAYOUT_VIA_UTIL_H

#include <type_traits>

#include <cbag/common/box_t.h>
#include <cbag/common/transformation_util.h>
#include <cbag/layout/via.h>

namespace cbag {
namespace layout {

box_t get_bot_box(const via &v);

box_t get_top_box(const via &v);

template <typename It> void get_via_cuts(const via &v, It out_iter) {
    auto &params = v.get_params();
    auto &[nx, ny] = params.num;
    auto &[vw, vh] = params.cut_dim;
    auto &[spx, spy] = params.cut_spacing;

    auto xoff = (nx * (vw + spx) - spx) / 2;
    auto yoff = (ny * (vh + spy) - spy) / 2;

    offset_t dx = -xoff;
    box_t cut_box{0, 0, static_cast<coord_t>(vw), static_cast<coord_t>(vh)};
    for (std::decay_t<decltype(nx)> xidx = 0; xidx != nx; ++xidx, dx += spx) {
        offset_t dy = -yoff;
        for (std::decay_t<decltype(ny)> yidx = 0; yidx != ny; ++yidx, dy += spy) {
            *out_iter = get_move_by(get_transform(cut_box, v.xform), dx, dy);
            ++out_iter;
        }
    }
}

} // namespace layout
} // namespace cbag

#endif
