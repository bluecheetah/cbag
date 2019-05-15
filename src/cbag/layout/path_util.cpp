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
#include <fmt/ostream.h>

#include <cbag/layout/path_util.h>

namespace cbag {
namespace layout {

void add_path_points(pt_vector &vec, coord_t x, coord_t y, const vector45 &p, const vector45 &n,
                     bool is_45, end_style style, offset_t w_main, offset_t w_norm) {
    switch (style) {
    case end_style::truncate: {
        auto xw = n.dx * w_main;
        auto yw = n.dy * w_main;
        vec.emplace_back(point_t{x - xw, y - yw});
        vec.emplace_back(point_t{x + xw, y + yw});
        break;
    }
    case end_style::extend:
        vec.emplace_back(point_t{x - (p.dx + n.dx) * w_main, y - (p.dy + n.dy) * w_main});
        vec.emplace_back(point_t{x - (p.dx - n.dx) * w_main, y - (p.dy - n.dy) * w_main});
        break;
    case end_style::triangle: {
        auto xw = n.dx * w_main;
        auto yw = n.dy * w_main;
        vec.emplace_back(point_t{x - xw, y - yw});
        vec.emplace_back(point_t{x - w_main * p.dx, y - w_main * p.dy});
        vec.emplace_back(point_t{x + xw, y + yw});
        break;
    }
    default: {
        auto xnm = n.dx * w_main;
        auto ynm = n.dy * w_main;
        auto xpm = p.dx * w_main;
        auto ypm = p.dy * w_main;
        auto xnn = n.dx * w_norm;
        auto ynn = n.dy * w_norm;
        auto xpn = p.dx * w_norm;
        auto ypn = p.dy * w_norm;
        vec.emplace_back(point_t{x - xpn - xnm, y - ypn - ynm});
        vec.emplace_back(point_t{x - xpm - xnn, y - ypm - ynn});
        vec.emplace_back(point_t{x - xpm + xnn, y - ypm + ynn});
        vec.emplace_back(point_t{x - xpn + xnm, y - ypn + ynm});
    }
    }
} // namespace layout

end_style get_style(end_style ans, offset_t half_width, bool is_45) {
    if (ans == end_style::round) {
        // handle degenerate cases
        switch (half_width) {
        case 1:
            // triangle if 45 degrees, extend otherwise
            return static_cast<end_style>(1 + 2 * is_45);
        case 2:
            // extend if 45 degrees, triangle otherwise
            return static_cast<end_style>(3 - 2 * is_45);
        default:
            return end_style::round;
        }
    }
    return ans;
}

pt_vector path_to_poly45(coord_t x0, coord_t y0, coord_t x1, coord_t y1, offset_t half_width,
                         end_style sty0, end_style sty1) {
    pt_vector ans;
    vector45 p_norm{x1 - x0, y1 - y0};
    p_norm.normalize();

    // handle empty path
    if (half_width == 0 || (p_norm.dx == 0 && p_norm.dy == 0)) {
        return ans;
    }
    // handle invalid path
    if (!p_norm.valid()) {
        throw std::invalid_argument(fmt::format("path segment vector {} not valid", p_norm));
    }

    bool is_45 = p_norm.is_45_or_invalid();

    // initialize point array, reserve space for worst case
    ans.reserve(8);

    vector45 n_norm = p_norm.get_rotate90();
    auto half_diag = static_cast<offset_t>(round(half_width / root2));
    offset_t w_main, w_norm;
    if (is_45) {
        w_main = half_diag;
        w_norm = half_width - half_diag;
    } else {
        w_main = half_width;
        w_norm = 2 * half_diag - half_width;
    }

    add_path_points(ans, x0, y0, p_norm, n_norm, is_45, get_style(sty0, half_width, is_45), w_main,
                    w_norm);
    p_norm.invert();
    n_norm.invert();
    add_path_points(ans, x1, y1, p_norm, n_norm, is_45, get_style(sty1, half_width, is_45), w_main,
                    w_norm);

    return ans;
}

} // namespace layout
} // namespace cbag
