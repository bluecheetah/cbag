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

#include <cstdlib>

#include <cbag/common/box_t.h>
#include <cbag/common/transformation_util.h>
#include <cbag/layout/cellview_poly.h>
#include <cbag/layout/cellview_util.h>
#include <cbag/layout/cv_obj_ref.h>
#include <cbag/layout/grid_object.h>
#include <cbag/layout/tech_util.h>
#include <cbag/layout/track_info_util.h>
#include <cbag/layout/via_param_util.h>
#include <cbag/layout/via_wrapper.h>
#include <cbag/layout/wire_width.h>
#include <cbag/polygon/polygon_set_data.h>

namespace cbag {
namespace layout {

box_t get_bbox(const cellview &cv, const std::string &layer, const std::string &purpose) {
    auto ans = box_t::get_invalid_bbox();
    // merge geometry bounding box
    auto iter = cv.find_geometry(layer_t_at(*cv.get_tech(), layer, purpose));
    if (iter != cv.end_geometry()) {
        ans |= get_bbox(iter->second);
    }
    // merge instance bounding box
    for (auto it = cv.begin_inst(); it != cv.end_inst(); ++it) {
        ans |= it->second.get_bbox(layer, purpose);
    }
    return ans;
}

void add_pin(cellview &cv, const std::string &layer, const std::string &net,
             const std::string &label, const box_t &bbox) {
    auto lay_id = layer_id_at(*cv.get_tech(), layer);
    cv.add_pin(lay_id, std::string(net), std::string(label), box_t(bbox));
}

void add_pin_arr(cellview &cv, const std::string &net, const std::string &label,
                 const wire_array &warr) {
    auto &tid = warr.get_track_id_ref();
    auto level = tid.get_level();
    auto cur_htr = tid.get_htr();
    auto ntr = tid.get_ntr();
    auto num = tid.get_num();
    auto pitch = tid.get_pitch();
    auto [lower, upper] = warr.get_coord();

    auto &tinfo = cv.get_grid()->track_info_at(level);
    auto winfo = tinfo.get_wire_width(ntr);
    auto winfo_end = winfo.end();
    auto tr_dir = tinfo.get_direction();
    for (decltype(num) idx = 0; idx < num; ++idx, cur_htr += pitch) {
        auto [lay, purp] = get_layer_t(*cv.get_grid(), level, cur_htr);
        for (auto witer = winfo.begin(); witer != winfo_end; ++witer) {
            auto &[rel_htr, wire_w] = *witer;
            auto half_w = wire_w / 2;
            auto center = htr_to_coord(tinfo, cur_htr + rel_htr);
            cv.add_pin(lay, std::string(net), std::string(label),
                       box_t(tr_dir, lower, upper, center - half_w, center + half_w));
        }
    }
}

cv_obj_ref<via_wrapper> add_via(const std::shared_ptr<cellview> &cv_ptr, transformation xform,
                                std::string via_id, const via_param &params, bool add_layers,
                                bool commit) {
    return {cv_ptr, via_wrapper(via(std::move(xform), std::move(via_id), params), add_layers),
            commit};
}

void add_via_arr(cellview &cv, const transformation &xform, const std::string &via_id,
                 const via_param &params, bool add_layers, std::array<cnt_t, 2> num_arr,
                 std::array<offset_t, 2> sp_arr) {
    offset_t dx = 0;
    for (cnt_t xidx = 0; xidx < num_arr[0]; ++xidx, dx += sp_arr[0]) {
        offset_t dy = 0;
        for (cnt_t yidx = 0; yidx < num_arr[1]; ++yidx, dy += sp_arr[1]) {
            cv.add_object(
                via_wrapper(via(polygon::get_move_by(xform, dx, dy), via_id, params), add_layers));
        }
    }
}

std::array<std::array<coord_t, 2>, 2> add_via_on_intersections(cellview &cv, const track_id &tid1,
                                                               const track_id &tid2,
                                                               std::array<coord_t, 2> coord1,
                                                               std::array<coord_t, 2> coord2,
                                                               bool extend, bool contain) {
    std::array<std::array<coord_t, 2>, 2> ans = {std::array<coord_t, 2>{COORD_MAX, COORD_MIN},
                                                 std::array<coord_t, 2>{COORD_MAX, COORD_MIN}};
    auto lev1 = tid1.get_level();
    auto lev2 = tid2.get_level();
    auto diff = lev1 - lev2;
    if (std::abs(diff) != 1)
        throw std::invalid_argument(
            fmt::format("Cannot create via between layers {} and {}", lev1, lev2));

    std::array<const track_id *, 2> tid_arr;
    std::array<std::array<coord_t, 2>, 2> coord_arr;
    auto idx1 = static_cast<int>(diff > 0);
    tid_arr[idx1] = &tid1;
    tid_arr[1 - idx1] = &tid2;
    coord_arr[idx1] = coord1;
    coord_arr[1 - idx1] = coord2;
    auto &grid = *cv.get_grid();
    auto &tech = *grid.get_tech();
    auto &tinfo0 = grid.track_info_at(tid_arr[0]->get_level());
    auto &tinfo1 = grid.track_info_at(tid_arr[1]->get_level());
    auto dir0 = tinfo0.get_direction();
    auto dir1 = tinfo1.get_direction();
    if (dir0 == dir1)
        throw std::invalid_argument("Cannot draw vias between layers with same direction.");

    for (auto i0 = begin_rect(grid, *tid_arr[0], coord_arr[0]),
              s0 = end_rect(grid, *tid_arr[0], coord_arr[0]);
         i0 != s0; ++i0) {
        auto [lay0, box0] = *i0;
        for (auto i1 = begin_rect(grid, *tid_arr[1], coord_arr[1]),
                  s1 = end_rect(grid, *tid_arr[1], coord_arr[1]);
             i1 != s1; ++i1) {
            auto [lay1, box1] = *i1;
            auto via_box = box0;
            via_box &= box1;
            auto box_dim = std::array<coord_t, 2>{width(via_box), height(via_box)};
            if (box_dim[to_int(dir0)] == dimension(box1, dir0) &&
                box_dim[to_int(dir1)] == dimension(box0, dir1)) {
                auto &via_id = tech.get_via_id(direction_1d::LOWER, lay0, lay1);
                auto via_param =
                    tech.get_via_param(box_dim, via_id, direction_1d::LOWER, dir0, dir1, extend);
                if (!empty(via_param)) {
                    auto via_ext =
                        get_via_extensions(via_param, box_dim, direction_1d::LOWER, dir0, dir1);
                    auto l0 = lower(via_box, dir0) - via_ext[0];
                    auto u0 = upper(via_box, dir0) + via_ext[0];
                    auto l1 = lower(via_box, dir1) - via_ext[1];
                    auto u1 = upper(via_box, dir1) + via_ext[1];
                    if (!contain || (lower(box0, dir0) <= l0 && upper(box0, dir0) >= u0 &&
                                     lower(box1, dir1) <= l1 && upper(box1, dir1) >= u1)) {
                        // we can draw via safely
                        cv.add_object(via_wrapper(
                            via(transformation(xm(via_box), ym(via_box)), via_id, via_param),
                            false));
                        // update bounds
                        ans[0][0] = std::min(ans[0][0], l0);
                        ans[0][1] = std::max(ans[0][1], u0);
                        ans[1][0] = std::min(ans[1][0], l1);
                        ans[1][1] = std::max(ans[1][1], u1);
                    }
                }
            }
        }
    }
    return ans;
}

std::array<std::array<coord_t, 2>, 2> add_via_on_intersections(cellview &cv,
                                                               const wire_array &warr1,
                                                               const wire_array &warr2, bool extend,
                                                               bool contain) {
    return add_via_on_intersections(cv, warr1.get_track_id_ref(), warr2.get_track_id_ref(),
                                    warr1.get_coord(), warr2.get_coord(), extend, contain);
}

std::array<std::array<coord_t, 2>, 2>
connect_box_track(cellview &cv, direction_1d vdir, layer_t key, const box_t &box,
                  std::array<cnt_t, 2> num, std::array<offset_t, 2> sp, const track_id &tid,
                  const std::array<std::optional<coord_t>, 2> &box_ext,
                  const std::array<std::optional<coord_t>, 2> &tr_ext, min_len_mode mode) {
    std::array<std::array<coord_t, 2>, 2> ans;

    auto &grid = *cv.get_grid();
    auto &tech = *grid.get_tech();
    auto &tinfo = grid.track_info_at(tid.get_level());
    auto tr_dir = tinfo.get_direction();
    auto p_dir = perpendicular(tr_dir);
    auto tr_didx = to_int(tr_dir);
    auto p_didx = to_int(p_dir);

    auto v_didx = to_int(vdir);
    auto tv_didx = v_didx ^ 1;
    auto num_box = num[tr_didx];
    auto sp_box = sp[tr_didx];
    decltype(sp_box) delta = sp_box * (num_box - 1);
    decltype(sp_box) p_delta = sp[p_didx] * (num[p_didx] - 1);
    ans[v_didx] = std::array<coord_t, 2>{lower(box, p_dir), upper(box, p_dir)};
    ans[v_didx][p_delta > 0] += p_delta;
    ans[tv_didx] = std::array<coord_t, 2>{lower(box, tr_dir), upper(box, tr_dir)};
    ans[tv_didx][delta > 0] += delta;

    for (auto iter = begin_rect(grid, tid, ans[1]), stop = end_rect(grid, tid, ans[1]);
         iter != stop; ++iter) {
        // compute via
        auto [tr_key, tr_box] = *iter;
        auto &via_id = tech.get_via_id(vdir, key, tr_key);
        auto via_box = box_t(tr_dir, lower(box, tr_dir), upper(box, tr_dir), lower(tr_box, p_dir),
                             upper(tr_box, p_dir));
        auto via_dim = std::array<coord_t, 2>{width(via_box), height(via_box)};
        auto via_param = tech.get_via_param(via_dim, via_id, vdir, p_dir, tr_dir, true);

        // add via
        auto xform = transformation(xm(via_box), ym(via_box));
        auto via_num = std::array<cnt_t, 2>{1, 1};
        auto via_sp = std::array<offset_t, 2>{0, 0};
        via_num[tr_didx] = num_box;
        via_sp[tr_didx] = sp_box;
        add_via_arr(cv, xform, via_id, via_param, false, via_num, via_sp);

        // get via coordinate boundaries
        auto via_ext = get_via_extensions(via_param, via_dim, vdir, p_dir, tr_dir);
        ans[v_didx][0] = std::min(ans[v_didx][0], lower(via_box, p_dir) - via_ext[p_didx]);
        ans[v_didx][1] = std::max(ans[v_didx][1], upper(via_box, p_dir) + via_ext[p_didx]);
        ans[tv_didx][0] = std::min(ans[tv_didx][0], lower(via_box, tr_dir) - via_ext[tr_didx]);
        ans[tv_didx][1] = std::max(ans[tv_didx][1], upper(via_box, tr_dir) + via_ext[tr_didx]);
    }

    // perform optional extensions
    if (box_ext[0])
        ans[v_didx][0] = std::min(ans[v_didx][0], *box_ext[0]);
    if (box_ext[1])
        ans[v_didx][1] = std::max(ans[v_didx][1], *box_ext[1]);
    if (tr_ext[0])
        ans[tv_didx][0] = std::min(ans[tv_didx][0], *tr_ext[0]);
    if (tr_ext[1])
        ans[tv_didx][1] = std::max(ans[tv_didx][1], *tr_ext[1]);

    // perform minimum length correction
    auto tr_len = ans[tv_didx][1] - ans[tv_didx][0];
    switch (mode) {
    case min_len_mode::LOWER:
        tr_len = std::max(tr_len, get_min_length(tech, tid.get_level(),
                                                 tinfo.get_wire_width(tid.get_ntr()), false));
        ans[tv_didx][0] = ans[tv_didx][1] - tr_len;
        break;
    case min_len_mode::UPPER:
        tr_len = std::max(tr_len, get_min_length(tech, tid.get_level(),
                                                 tinfo.get_wire_width(tid.get_ntr()), false));
        ans[tv_didx][1] = ans[tv_didx][0] + tr_len;
        break;
    case min_len_mode::MIDDLE:
        tr_len = std::max(tr_len, get_min_length(tech, tid.get_level(),
                                                 tinfo.get_wire_width(tid.get_ntr()), true));
        ans[tv_didx][0] = (ans[tv_didx][0] + ans[tv_didx][1] - tr_len) / 2;
        ans[tv_didx][1] = ans[tv_didx][0] + tr_len;
        break;
    default:
        break;
    }

    // add bbox wires
    auto new_box =
        box_t(tr_dir, lower(box, tr_dir), upper(box, tr_dir), ans[v_didx][0], ans[v_didx][1]);
    auto new_num_box = std::array<cnt_t, 2>{1, 1};
    auto new_sp_box = std::array<offset_t, 2>{0, 0};
    new_num_box[tr_didx] = num_box;
    new_sp_box[tr_didx] = sp_box;
    add_rect_arr(cv, key, new_box, new_num_box, new_sp_box);

    // add track wires and return track coordinates
    cv.add_warr(tid, ans[tv_didx]);
    return ans;
}

std::array<std::array<coord_t, 2>, 2>
connect_warr_track(cellview &cv, const wire_array &warr, const track_id &tid,
                   const std::array<std::optional<coord_t>, 2> &w_ext,
                   const std::array<std::optional<coord_t>, 2> &tr_ext) {

    // draw vias
    auto &w_tid = warr.get_track_id_ref();
    auto inf_bnds = std::array<coord_t, 2>{COORD_MIN, COORD_MAX};
    auto ans = add_via_on_intersections(cv, w_tid, tid, inf_bnds, inf_bnds, true, false);
    if (ans[0][1] <= ans[0][0]) {
        // add_via_on_intersections failed to add any vias
        throw std::runtime_error("Cannot draw vias from given WireArray to TrackID.");
    }
    // determine wire_array and track_id layer indices
    auto wlev = w_tid.get_level();
    auto tlev = tid.get_level();
    auto lev_diff = wlev - tlev;
    auto w_vidx = static_cast<int>(lev_diff > 0);
    auto t_vidx = 1 - w_vidx;

    // extend wires and track indicates
    auto &w_coords = warr.get_coord();
    if (w_ext[0]) {
        ans[w_vidx][0] = std::min({ans[w_vidx][0], w_coords[0], *w_ext[0]});
    } else {
        ans[w_vidx][0] = std::min(ans[w_vidx][0], w_coords[0]);
    }
    if (w_ext[1]) {
        ans[w_vidx][1] = std::max({ans[w_vidx][1], w_coords[1], *w_ext[1]});
    } else {
        ans[w_vidx][1] = std::max(ans[w_vidx][1], w_coords[1]);
    }
    if (tr_ext[0]) {
        ans[t_vidx][0] = std::min(ans[t_vidx][0], *tr_ext[0]);
    }
    if (tr_ext[1]) {
        ans[t_vidx][1] = std::max(ans[t_vidx][1], *tr_ext[1]);
    }

    // draw wires
    cv.add_warr(w_tid, ans[w_vidx]);
    cv.add_warr(tid, ans[t_vidx]);

    return ans;
}

void add_label(cellview &cv, const std::string &layer, const std::string &purpose,
               transformation xform, std::string label, offset_t text_h) {
    cv.add_label(layer_t_at(*cv.get_tech(), layer, purpose), std::move(xform), std::move(label),
                 text_h);
}

cv_obj_ref<instance> add_prim_instance(const std::shared_ptr<cellview> &cv_ptr, std::string lib,
                                       std::string cell, std::string view, std::string name,
                                       cbag::transformation xform, cnt_t nx, cnt_t ny, offset_t spx,
                                       offset_t spy, bool commit) {
    return {cv_ptr,
            instance(std::move(name), std::move(lib), std::move(cell), std::move(view),
                     std::move(xform), nx, ny, spx, spy),
            commit};
}

cv_obj_ref<instance> add_instance(const std::shared_ptr<cellview> &cv_ptr,
                                  const std::shared_ptr<const cellview> &master, std::string name,
                                  cbag::transformation xform, cnt_t nx, cnt_t ny, offset_t spx,
                                  offset_t spy, bool commit) {
    return {cv_ptr, instance(std::move(name), master, std::move(xform), nx, ny, spx, spy), commit};
}

} // namespace layout
} // namespace cbag
