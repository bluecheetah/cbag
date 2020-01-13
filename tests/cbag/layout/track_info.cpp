// SPDX-License-Identifier: Apache-2.0
/*
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

#include <limits>

#include <catch2/catch.hpp>

#include <cbag/common/transformation_util.h>
#include <cbag/layout/track_info_util.h>

#include <yaml-cpp/tuple.h>

#include <cbag/tests/util/io.h>
#include <cbag/yaml/int_array.h>
#include <cbag/yaml/transformation.h>

using c_tinfo = cbag::layout::track_info;
using c_xform = cbag::transformation;

TEST_CASE("transform_htr()", "[track_info]") {
    using data_type =
        std::tuple<int, std::array<cbag::offset_t, 2>, c_xform, std::array<cbag::htr_t, 2>>;
    auto fname = "tests/data/cbag/layout/track_info/transform_htr.yaml";
    auto[tr_code, tr_dim, xform, htr_arr] = GENERATE_COPY(read_test_vector<data_type>(fname));

    auto tr_dir = static_cast<cbag::orientation_2d>(tr_code);
    auto[tr_w, tr_sp] = tr_dim;
    auto[htr, expect] = htr_arr;

    auto tr_pitch = tr_w + tr_sp;
    auto tr_off = tr_pitch / 2;
    auto tr_info = c_tinfo(tr_dir, tr_w, tr_sp, tr_off);

    auto ans = cbag::layout::transform_htr(tr_info, htr, xform);
    CAPTURE(tr_pitch, xform, htr, expect);
    REQUIRE(ans == expect);
}

TEST_CASE("coord_to_htr", "[track_info]") {
    using data_type =
        std::tuple<cbag::offset_t, cbag::offset_t, cbag::offset_t, int, bool, cbag::htr_t>;
    auto fname = "tests/data/cbag/layout/track_info/coord_to_htr.yaml";

    auto[tr_w, tr_sp, coord, mode, even, expect] =
        GENERATE_COPY(read_test_vector<data_type>(fname));

    auto tr_pitch = tr_w + tr_sp;
    auto tr_off = tr_pitch / 2;
    auto tr_info = c_tinfo(cbag::orientation_2d::VERTICAL, tr_w, tr_sp, tr_off);

    auto ans =
        cbag::layout::coord_to_htr(tr_info, coord, static_cast<cbag::round_mode>(mode), even);

    CAPTURE(tr_w, tr_sp, coord, mode, even);
    REQUIRE(ans == expect);
}
