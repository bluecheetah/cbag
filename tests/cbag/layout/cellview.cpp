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

#include <catch2/catch.hpp>

#include <cbag/common/box_t.h>
#include <cbag/common/transformation_util.h>
#include <cbag/enum/end_style.h>
#include <cbag/layout/cellview_poly.h>
#include <cbag/layout/cellview_util.h>
#include <cbag/layout/grid_object.h>
#include <cbag/layout/path_util.h>
#include <cbag/layout/polygons.h>
#include <cbag/layout/via_wrapper.h>

#include <cbag/tests/layout/util.h>

using c_cellview = cbag::layout::cellview;
using c_xform = cbag::transformation;
using c_tid = cbag::layout::track_id;
using c_warr = cbag::layout::wire_array;
using c_offset_t = cbag::offset_t;
using c_via_param = cbag::layout::via_param;
using c_box = cbag::box_t;
using c_pin = cbag::layout::pin;
using c_poly_set = cbag::layout::poly_set_t;

using pt_type = cbag::point_t;
using intv_type = std::array<cbag::coord_t, 2>;

std::shared_ptr<c_cellview> make_cv(const std::shared_ptr<const c_grid> &grid) {
    return std::make_shared<c_cellview>(grid, "CBAG_TEST");
}

std::ostream &operator<<(std::ostream &os, const c_pin &value) {
    auto box = value.bbox();
    return os << fmt::format("pin({}, {}, {}, {}, {}, {})", value.net(), value.label(), xl(box),
                             yl(box), xh(box), yh(box));
}

TEST_CASE("add wire array", "[layout::cellview]") {
    auto [tech_info, grid] = make_tech_grid();
    auto cv = make_cv(grid);
    auto [tid, lower, upper] = GENERATE(values<std::tuple<c_tid, c_offset_t, c_offset_t>>({
        // one track
        {c_tid(4, 0, 1, 1, 0), 0, 100},
        {c_tid(4, -2, 2, 1, 0), -200, 100},
        {c_tid(4, 5, 3, 1, 0), -50, 100},
        // two tracks
        {c_tid(4, 0, 1, 2, 1), 0, 100},
        {c_tid(4, -2, 2, 4, -4), -300, -100},
    }));

    auto warr = c_warr(std::make_shared<c_tid>(tid), lower, upper);

    cbag::layout::add_warr(*cv, warr);
}

TEST_CASE("add path", "[layout::cellview]") {
    using data_type = std::tuple<std::array<std::string, 2>, std::vector<pt_type>, cbag::offset_t,
                                 std::array<cbag::end_style, 3>>;
    auto [tech_info, grid] = make_tech_grid();
    auto cv = make_cv(grid);
    auto [lay_purp, pt_list, half_w, styles] = GENERATE(values<data_type>({
        {
            {"M2CA", ""},
            {pt_type{0, 0}, pt_type{2000, 0}, pt_type{3000, 1000}, pt_type{3000, 3000}},
            10,
            {cbag::end_style::truncate, cbag::end_style::round, cbag::end_style::round},
        },
    }));

    add_path(cv, lay_purp[0], lay_purp[1], pt_list, half_w, styles[0], styles[1], styles[2], true);

    // create expected geometry
    auto key = cbag::layout::layer_t_at(*(cv->get_tech()), lay_purp[0], lay_purp[1]);
    auto expect = cbag::layout::make_path(pt_list, half_w, styles[0], styles[1], styles[2]);

    auto geo_iter = cv->find_geometry(key);
    REQUIRE(geo_iter != cv->end_geometry());
    REQUIRE(geo_iter->second == expect);
}

TEST_CASE("add blockage", "[layout::cellview]") {
    using data_type = std::tuple<std::string, cbag::blockage_type, std::vector<pt_type>>;
    auto [tech_info, grid] = make_tech_grid();
    auto cv = make_cv(grid);
    auto [layer, blk_type, pt_list] = GENERATE(values<data_type>({
        {"", cbag::blockage_type::placement, {pt_type{0, 0}, pt_type{100, 0}, pt_type{0, 100}}},
    }));

    cbag::layout::add_blockage(cv, layer, blk_type, pt_list, true);
}

TEST_CASE("add via", "[layout::cellview]") {
    using data_type = std::tuple<c_xform, std::string, std::string, c_via_param, c_box, c_box>;
    auto [tech_info, grid] = make_tech_grid();
    auto cv = make_cv(grid);
    auto [xform, lay1, lay2, via_param, box1, box2] = GENERATE(values<data_type>({
        {cbag::transformation(0, 0), "M4", "M5",
         c_via_param(1, 1, 32, 32, 0, 0, 14, 14, 14, 14, 14, 14, 14, 14), c_box(-30, -30, 30, 30),
         c_box(-30, -30, 30, 30)},
    }));

    auto l1 = cbag::layout::layer_t_at(*tech_info, lay1, "");
    auto l2 = cbag::layout::layer_t_at(*tech_info, lay2, "");
    auto via_id = tech_info->get_via_id(cbag::direction_1d::LOWER, l1, l2);

    cv->add_object(cbag::layout::via_wrapper(cbag::layout::via(xform, via_id, via_param), true));

    auto expect1 = c_poly_set();
    auto expect2 = c_poly_set();
    expect1.insert(box1);
    expect2.insert(box2);

    auto iter_end = cv->end_geometry();
    auto iter1 = cv->find_geometry(l1);
    auto iter2 = cv->find_geometry(l2);
    REQUIRE(iter1 != iter_end);
    REQUIRE(iter2 != iter_end);
    REQUIRE(iter1->second == expect1);
    REQUIRE(iter2->second == expect2);
}

TEST_CASE("add via on intersections", "[layout::cellview]") {
    using data_type =
        std::tuple<c_tid, c_tid, intv_type, intv_type, bool, bool, std::array<intv_type, 2>>;
    auto [tech_info, grid] = make_tech_grid();
    auto cv = make_cv(grid);
    auto [tid1, tid2, coord1, coord2, extend, contain, expect] = GENERATE(values<data_type>({
        {c_tid(4, 1, 2, 1, 0),
         c_tid(5, 4, 1, 1, 0),
         {cbag::COORD_MIN, cbag::COORD_MAX},
         {cbag::COORD_MIN, cbag::COORD_MAX},
         true,
         false,
         {intv_type{842, 958}, intv_type{64, 320}}},
    }));

    auto ans =
        cbag::layout::add_via_on_intersections(*cv, tid1, tid2, coord1, coord2, extend, contain);

    REQUIRE(ans == expect);
}

TEST_CASE("connect warr to track", "[layout::cellview]") {
    using ext_type = std::optional<cbag::coord_t>;
    using ext_arr_type = std::array<ext_type, 2>;
    using data_type =
        std::tuple<c_tid, c_tid, intv_type, ext_arr_type, ext_arr_type, std::array<intv_type, 2>>;
    auto [tech_info, grid] = make_tech_grid();
    auto cv = make_cv(grid);
    auto [tid1, tid2, coord1, w_ext, tr_ext, expect] = GENERATE(values<data_type>({
        {c_tid(4, 1, 2, 1, 0),
         c_tid(5, 4, 1, 1, 0),
         {0, 400},
         {ext_type{}, ext_type{}},
         {ext_type{}, ext_type{}},
         {intv_type{0, 958}, intv_type{64, 320}}},
        {c_tid(4, 1, 2, 1, 0),
         c_tid(5, 4, 1, 1, 0),
         {0, 400},
         {ext_type{}, ext_type{}},
         {0, 400},
         {intv_type{0, 958}, intv_type{0, 400}}},
    }));

    auto warr = c_warr(std::make_shared<c_tid>(tid1), coord1[0], coord1[1]);
    auto ans = cbag::layout::connect_warr_track(*cv, warr, tid2, w_ext, tr_ext);

    REQUIRE(ans == expect);
}

TEST_CASE("connect warr to track failure", "[layout::cellview]") {
    using ext_type = std::optional<cbag::coord_t>;
    using ext_arr_type = std::array<ext_type, 2>;
    using data_type =
        std::tuple<c_tid, c_tid, intv_type, ext_arr_type, ext_arr_type, std::array<intv_type, 2>>;
    auto [tech_info, grid] = make_tech_grid();
    auto cv = make_cv(grid);
    auto [tid1, tid2, coord1, w_ext, tr_ext, expect] = GENERATE(values<data_type>({
        {c_tid(4, 0, 1, 1, 0),
         c_tid(5, 4, 1, 1, 0),
         {0, 400},
         {ext_type{}, ext_type{}},
         {ext_type{}, ext_type{}},
         {intv_type{0, 958}, intv_type{86, 202}}},
        {c_tid(4, 0, 1, 1, 0),
         c_tid(5, 4, 1, 1, 0),
         {0, 400},
         {ext_type{}, ext_type{}},
         {0, 400},
         {intv_type{0, 958}, intv_type{0, 400}}},
    }));

    auto warr = c_warr(std::make_shared<c_tid>(tid1), coord1[0], coord1[1]);
    REQUIRE_THROWS_AS(cbag::layout::connect_warr_track(*cv, warr, tid2, w_ext, tr_ext),
                      std::runtime_error);
}

TEST_CASE("add_pin_arr()", "[layout::cellview]") {
    using pin_list_t = std::vector<c_pin>;
    using data_type = std::tuple<std::string, std::string, c_tid, intv_type, pin_list_t>;
    auto [tech_info, grid] = make_tech_grid();
    auto cv = make_cv(grid);
    auto [net, label, tid, coord, expect] = GENERATE(values<data_type>({
        {"foo", "foo", c_tid(4, 0, 1, 1, 0), {0, 400}, {c_pin(0, 64, 400, 128, "foo", "foo")}},
        {"bar",
         "baz",
         c_tid(4, 0, 1, 3, 2),
         {0, 400},
         {c_pin(0, 64, 400, 128, "bar", "baz"), c_pin(0, 256, 400, 320, "bar", "baz"),
          c_pin(0, 448, 400, 512, "bar", "baz")}},
    }));

    auto warr = c_warr(std::make_shared<c_tid>(tid), coord[0], coord[1]);
    cbag::layout::add_pin_arr(*cv, net, label, warr);

    auto pin_map_iter = cv->begin_pin();
    REQUIRE(pin_map_iter != cv->end_pin());
    auto &ans = pin_map_iter->second;
    REQUIRE(ans == expect);
}
