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

#ifndef CBAG_LAYOUT_PATH_UTIL_H
#define CBAG_LAYOUT_PATH_UTIL_H

#include <cbag/enum/end_style.h>
#include <cbag/layout/cellview_poly.h>
#include <cbag/layout/cv_obj_ref.h>
#include <cbag/layout/polygons.h>
#include <cbag/layout/pt_vector.h>
#include <cbag/layout/vector45.h>
#include <cbag/math/constexpr.h>

namespace cbag {
namespace layout {

constexpr double root2 = cbag::math::sqrt(2);

pt_vector path_to_poly45(coord_t x0, coord_t y0, coord_t x1, coord_t y1, offset_t half_width,
                         end_style sty0, end_style sty1);

template <typename T, typename = IsPtList<T>>
poly_set_t make_path(const T &data, offset_t half_width, end_style s0, end_style s1, end_style sm) {
    auto n = traits::pt_list<T>::size(data);
    if (n < 2) {
        throw std::invalid_argument("Cannot draw path with less than 2 points.");
    }

    poly_set_t ans;
    for (pt_vector::size_type istart = 0; istart < n - 1; ++istart) {
        end_style c0 = (istart == 0) ? s0 : sm;
        end_style c1 = (istart == n - 2) ? s1 : sm;
        auto inext = istart + 1;
        auto cx = traits::pt_list<T>::x(data, istart);
        auto cy = traits::pt_list<T>::y(data, istart);
        auto nx = traits::pt_list<T>::x(data, inext);
        auto ny = traits::pt_list<T>::y(data, inext);
        auto tmp = poly_45_t(path_to_poly45(cx, cy, nx, ny, half_width, c0, c1));
        ans.insert(tmp);
    }

    return ans;
}

template <typename T, typename L, typename = IsPtList<T>>
poly_set_t make_path45_bus(const T &data, const L &widths, const L &spaces, end_style style0,
                           end_style style1, end_style stylem) {
    auto n_pts = traits::pt_list<T>::size(data);
    if (n_pts < 2) {
        throw std::invalid_argument("Cannot draw path with less than 2 points.");
    }
    auto n_paths = widths.size();
    if (n_paths != spaces.size() + 1) {
        throw std::invalid_argument("invalid size for path bus widths/spaces.");
    }

    // compute total width
    offset_t tot_width = 0;
    for (offset_t val : widths) {
        tot_width += val;
    }
    for (offset_t val : spaces) {
        tot_width += val;
    }

    // compute deltas
    std::vector<offset_t> deltas;
    deltas.reserve(n_paths);
    deltas[0] = (-tot_width + widths[0]) / 2;
    for (std::size_t idx = 1; idx < n_paths; ++idx) {
        deltas[idx] = deltas[idx - 1] + spaces[idx - 1] + (widths[idx - 1] + widths[idx]) / 2;
    }

    // get initial points
    pt_vector prev_pts;
    prev_pts.reserve(n_paths);
    auto x0 = traits::pt_list<T>::x(data, 0);
    auto y0 = traits::pt_list<T>::y(data, 0);
    auto x1 = traits::pt_list<T>::x(data, 1);
    auto y1 = traits::pt_list<T>::y(data, 1);
    vector45 s0{x1 - x0, y1 - y0};
    s0.rotate90_norm();
    if (s0.is_45_or_invalid()) {
        for (decltype(n_paths) idx = 0; idx < n_paths; ++idx) {
            auto scale = static_cast<offset_t>(round(deltas[idx] / root2));
            prev_pts.emplace_back(point_t{x0 + s0.dx * scale, y0 + s0.dy * scale});
        }
    } else {
        for (decltype(n_paths) idx = 0; idx < n_paths; ++idx) {
            auto scale = deltas[idx];
            prev_pts.emplace_back(point_t{x0 + s0.dx * scale, y0 + s0.dy * scale});
        }
    }

    // add intermediate path segments
    poly_set_t ans;
    auto sty1 = stylem;
    for (decltype(n_pts) nidx = 2; nidx < n_pts; ++nidx) {
        auto sty0 = (nidx == 2) ? style0 : stylem;

        auto xc = traits::pt_list<T>::x(data, nidx);
        auto yc = traits::pt_list<T>::y(data, nidx);
        auto xp1 = traits::pt_list<T>::x(data, nidx - 1);
        auto yp1 = traits::pt_list<T>::y(data, nidx - 1);
        auto xp2 = traits::pt_list<T>::x(data, nidx - 2);
        auto yp2 = traits::pt_list<T>::y(data, nidx - 2);
        s0.dx = xp1 - xp2;
        s0.dy = yp1 - yp2;
        vector45 s1{xc - xp1, yc - yp1};
        s0.normalize();
        s1.normalize();
        vector45 dir1 = s1.get_rotate90();
        if (dir1.is_45_or_invalid()) {
            for (decltype(n_paths) idx = 0; idx < n_paths; ++idx) {
                auto scale = static_cast<offset_t>(round(deltas[idx] / root2));
                auto prevx = traits::pt_list<pt_vector>::x(prev_pts, idx);
                auto prevy = traits::pt_list<pt_vector>::y(prev_pts, idx);
                auto pdx = xc + dir1.dx * scale - prevx;
                auto pdy = yc + dir1.dy * scale - prevy;
                auto k = (pdx * s1.dy - pdy * s1.dx) / (s0.dx * s1.dy - s0.dy * s1.dx);
                auto newx = prevx + k * s0.dx;
                auto newy = prevy + k * s0.dy;
                auto tmp = poly_45_t(
                    path_to_poly45(prevx, prevy, newx, newy, widths[idx] / 2, sty0, sty1));
                ans.insert(tmp);
                traits::pt_list<pt_vector>::set_x(prev_pts, idx, newx);
                traits::pt_list<pt_vector>::set_y(prev_pts, idx, newy);
            }
        } else {
            for (decltype(n_paths) idx = 0; idx < n_paths; ++idx) {
                auto scale = deltas[idx];
                auto prevx = traits::pt_list<pt_vector>::x(prev_pts, idx);
                auto prevy = traits::pt_list<pt_vector>::y(prev_pts, idx);
                auto pdx = xc + dir1.dx * scale - prevx;
                auto pdy = yc + dir1.dy * scale - prevy;
                auto k = (pdx * s1.dy - pdy * s1.dx) / (s0.dx * s1.dy - s0.dy * s1.dx);
                auto newx = prevx + k * s0.dx;
                auto newy = prevy + k * s0.dy;
                auto tmp = poly_45_t(
                    path_to_poly45(prevx, prevy, newx, newy, widths[idx] / 2, sty0, sty1));
                ans.insert(tmp);
                traits::pt_list<pt_vector>::set_x(prev_pts, idx, newx);
                traits::pt_list<pt_vector>::set_y(prev_pts, idx, newy);
            }
        }
    }

    // add last path segment
    auto sty0 = (n_pts == 2) ? style0 : stylem;
    sty1 = style1;
    x0 = traits::pt_list<T>::x(data, n_pts - 1);
    y0 = traits::pt_list<T>::y(data, n_pts - 1);
    s0.dx = x0 - traits::pt_list<T>::x(data, n_pts - 2);
    s0.dy = y0 - traits::pt_list<T>::y(data, n_pts - 2);
    s0.rotate90_norm();
    if (s0.is_45_or_invalid()) {
        for (decltype(n_paths) idx = 0; idx < n_paths; ++idx) {
            auto scale = static_cast<offset_t>(round(deltas[idx] / root2));
            auto xt = traits::pt_list<pt_vector>::x(prev_pts, idx);
            auto yt = traits::pt_list<pt_vector>::y(prev_pts, idx);
            auto tmp = poly_45_t(path_to_poly45(xt, yt, x0 + s0.dx * scale, y0 + s0.dy * scale,
                                                widths[idx] / 2, sty0, sty1));
            ans.insert(tmp);
        }
    } else {
        for (decltype(n_paths) idx = 0; idx < n_paths; ++idx) {
            auto scale = deltas[idx];
            auto xt = traits::pt_list<pt_vector>::x(prev_pts, idx);
            auto yt = traits::pt_list<pt_vector>::y(prev_pts, idx);
            auto tmp = poly_45_t(path_to_poly45(xt, yt, x0 + s0.dx * scale, y0 + s0.dy * scale,
                                                widths[idx] / 2, sty0, sty1));
            ans.insert(tmp);
        }
    }

    return ans;
}

template <typename T, typename = IsPtList<T>>
shape_ref<poly_set_t> add_path(const std::shared_ptr<cellview> &cv_ptr, const std::string &layer,
                               const std::string &purpose, const T &data, offset_t half_width,
                               end_style style0, end_style style1, end_style stylem, bool commit) {
    return add_polygon(cv_ptr, layer, purpose, make_path(data, half_width, style0, style1, stylem),
                       commit);
}

template <typename T, typename L, typename = IsPtList<T>>
shape_ref<poly_set_t>
add_path45_bus(const std::shared_ptr<cellview> &cv_ptr, const std::string &layer,
               const std::string &purpose, const T &data, const L &widths, const L &spaces,
               end_style style0, end_style style1, end_style stylem, bool commit) {
    return add_polygon(cv_ptr, layer, purpose,
                       make_path45_bus(data, widths, spaces, style0, style1, stylem), commit);
}

} // namespace layout
} // namespace cbag

#endif
