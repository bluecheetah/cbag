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

#include <tuple>
#include <vector>

#include <catch2/catch.hpp>

#include <yaml-cpp/tuple.h>

#include <cbag/common/transformation_util.h>
#include <cbag/layout/routing_grid_util.h>
#include <cbag/layout/track_coloring.h>
#include <cbag/polygon/enum.h>
#include <cbag/tests/layout/util.h>
#include <cbag/tests/util/io.h>
#include <cbag/yaml/int_array.h>
#include <cbag/yaml/transformation.h>

using c_offset_t = cbag::offset_t;
using c_tr_color = cbag::layout::track_coloring;

namespace cbag {
namespace layout {

std::ostream &operator<<(std::ostream &stream, const track_coloring &value) {
    return stream << value.to_string();
}

} // namespace layout
} // namespace cbag

namespace std {

std::ostream &operator<<(std::ostream &stream,
                         const std::tuple<cbag::htr_t, cbag::offset_t> &value) {
    return stream << "(" << std::get<0>(value) << ", " << std::get<1>(value) << ")";
}

} // namespace std

TEST_CASE("track pitch and block pitch work properly", "[grid]") {
    auto[tech_info, grid] = make_tech_grid();

    auto[lev, tp, bp, bp2] =
        GENERATE(values<std::tuple<cbag::level_t, c_offset_t, c_offset_t, c_offset_t>>({
            {1, 180, 180, 90},
            {2, 192, 192, 96},
            {3, 180, 180, 90},
            {4, 192, 192, 96},
            {5, 360, 360, 180},
            {6, 288, 576, 288},
            {7, 540, 1080, 540},
            {8, 960, 2880, 1440},
        }));

    CAPTURE(lev, tp, bp, bp2);
    REQUIRE(grid->track_info_at(lev).get_pitch() == tp);
    REQUIRE(grid->get_blk_pitch(lev, false) == bp);
    REQUIRE(grid->get_blk_pitch(lev, true) == bp2);
}

TEST_CASE("get_top_track_pitches()", "[grid]") {
    auto[tech_info, grid] = make_tech_grid();
    auto top_layer = 5;
    auto x_pitch_expect = 360;
    auto y_pitch_expect = 192;

    auto[x_pitch, y_pitch] = get_top_track_pitches(*grid, top_layer);

    REQUIRE(x_pitch == x_pitch_expect);
    REQUIRE(y_pitch == y_pitch_expect);
}

TEST_CASE("get_track_coloring_at()", "[grid]") {
    using data_type =
        std::tuple<std::array<cbag::level_t, 2>, cbag::transformation,
                   std::vector<std::tuple<cbag::cnt_t, cbag::offset_t, cbag::offset_t>>>;
    auto fname = "tests/data/cbag/layout/grid/track_coloring_at.yaml";

    auto[lev_arr, xform, vec] = GENERATE_COPY(read_test_vector<data_type>(fname));
    auto[bot_level, top_level] = lev_arr;

    auto tc_vec = std::vector<cbag::layout::color_info>();
    tc_vec.reserve(vec.size());
    for (auto[mod, scale, offset] : vec) {
        tc_vec.emplace_back(mod, scale, offset);
    }

    auto[tech_info, grid] = make_tech_grid();
    auto tr_colors = make_tr_colors(*tech_info);
    auto tr_specs = std::vector<std::tuple<int, int, int, int, int>>();
    auto child = grid->get_copy_with(bot_level - 1, -1, tr_specs);
    auto ans = grid->get_track_coloring_at(*tr_colors, xform, child, top_level);
    auto expect = c_tr_color(bot_level, std::move(tc_vec));

    CAPTURE(bot_level, top_level, xform);
    REQUIRE(ans == expect);
}

TEST_CASE("get_copy_with", "[grid]") {
    auto[tech_info, grid] = make_tech_grid();

    auto info_list = GENERATE(
        values<std::vector<
            std::tuple<cbag::level_t, int, cbag::offset_t, cbag::offset_t, cbag::offset_t>>>({
            {{1, 1, 20, 20, 10}},
            {{2, 0, 24, 24, 10}},
        }));

    auto grid2 =
        grid->get_copy_with(grid->get_top_ignore_level(), grid->get_top_private_level(), info_list);

    for (const auto[level, dir_code, w, sp, offset] : info_list) {
        auto &tr_info = grid2.track_info_at(level);
        REQUIRE(tr_info.get_direction() == static_cast<cbag::orientation_2d>(dir_code));
        REQUIRE(tr_info.get_offset() == offset);
        REQUIRE(tr_info.get_pitch() == w + sp);
    }
}

TEST_CASE("get_hash()", "[grid]") {
    using info_vec_t =
        std::vector<std::tuple<cbag::level_t, int, cbag::offset_t, cbag::offset_t, cbag::offset_t>>;

    auto[tech_info, grid] = make_tech_grid();

    auto new_grid = grid->get_copy_with(grid->get_top_ignore_level(), grid->get_top_private_level(),
                                        info_vec_t{});

    REQUIRE(new_grid == *grid);
    REQUIRE(new_grid.get_hash() == grid->get_hash());
}

TEST_CASE("get_via_extensions()", "[grid]") {
    using data_type =
        std::tuple<int, cbag::level_t, cbag::cnt_t, cbag::cnt_t, std::array<cbag::offset_t, 2>>;
    auto fname = "tests/data/cbag/layout/grid/get_via_extensions.yaml";

    auto[vdir, bot_lev, bot_ntr, top_ntr, expect] =
        GENERATE_COPY(read_test_vector<data_type>(fname));

    CAPTURE(vdir, bot_lev, bot_ntr, top_ntr);

    auto[tech_info, grid] = make_tech_grid();

    auto ans = cbag::layout::get_via_extensions(*grid, static_cast<cbag::direction_1d>(vdir),
                                                bot_lev, bot_ntr, top_ntr);
    REQUIRE(ans == expect);
}

TEST_CASE("get_via_extensions_dim()", "[grid]") {
    using data_type = std::tuple<cbag::direction_1d, cbag::level_t, cbag::offset_t, cbag::offset_t,
                                 std::array<cbag::offset_t, 2>>;

    auto[vdir, bot_lev, bot_dim, top_dim, expect] = GENERATE(values<data_type>({
        {cbag::direction_1d::LOWER, 1, 64, 64, {80, 80}},
        {cbag::direction_1d::LOWER, 1, 120, 64, {56, 52}},
    }));

    auto[tech_info, grid] = make_tech_grid();

    auto ans = cbag::layout::get_via_extensions_dim(*grid, vdir, bot_lev, bot_dim, top_dim);
    REQUIRE(ans == expect);
}

TEST_CASE("find_next_htr()", "[grid]") {
    using data_type =
        std::tuple<cbag::level_t, cbag::offset_t, cbag::cnt_t, int, bool, cbag::htr_t>;
    auto fname = "tests/data/cbag/layout/grid/find_next_htr.yaml";

    auto[level, coord, ntr, mode, even, expect] = GENERATE_COPY(read_test_vector<data_type>(fname));

    CAPTURE(level, coord, ntr, mode, even);

    auto[tech_info, grid] = make_tech_grid();

    auto ans = cbag::layout::find_next_htr(*grid, level, coord, ntr,
                                           static_cast<cbag::round_mode>(mode), even);

    REQUIRE(ans == expect);
}

TEST_CASE("get_wire_width()", "[grid]") {
    using data_type = std::tuple<cbag::level_t, cbag::cnt_t,
                                 std::vector<std::tuple<cbag::htr_t, cbag::offset_t>>>;
    auto tech_fname = "tests/data/tech_config/tech_params_test.yaml";
    auto fname = "tests/data/cbag/layout/grid/get_wire_width.yaml";

    auto[lev, ntr, expect_vec] = GENERATE_COPY(read_test_vector<data_type>(fname));

    CAPTURE(lev, ntr);

    auto[tech_info, grid] = make_tech_grid(tech_fname);

    auto expect = cbag::layout::wire_width(std::move(expect_vec));
    auto ans = grid->get_wire_width(lev, ntr);
    REQUIRE(ans == expect);
}

TEST_CASE("get_wire_bounds()", "[grid]") {
    using data_type =
        std::tuple<cbag::level_t, cbag::htr_t, cbag::cnt_t, std::array<cbag::offset_t, 2>>;
    auto tech_fname = "tests/data/tech_config/tech_params_test.yaml";
    auto fname = "tests/data/cbag/layout/grid/get_wire_bounds.yaml";

    auto[lev, htr, ntr, expect] = GENERATE_COPY(read_test_vector<data_type>(fname));

    CAPTURE(lev, htr, ntr, expect);

    auto[tech_info, grid] = make_tech_grid(tech_fname);

    auto wbnds = cbag::layout::get_wire_bounds(*grid, lev, htr, ntr);
    REQUIRE(wbnds == expect);
}
