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

#include <catch2/catch.hpp>

#include <cbag/layout/track_coloring.h>

TEST_CASE("get_htr_parity()", "[track_coloring]") {
    using cinfo_t = cbag::layout::color_info;
    using test_vec_t = std::vector<std::tuple<std::size_t, cbag::htr_t, cbag::cnt_t>>;
    using vec_t = std::vector<cinfo_t>;

    using data_type = std::tuple<vec_t, test_vec_t>;
    auto[tc_vec, test_vec] = GENERATE(values<data_type>({
        {vec_t({cinfo_t(2, 1, 0), cinfo_t(2, 1, 0), cinfo_t(2, 1, 0)}),
         {{0, -2, 1}, {0, -1, 1}, {0, 0, 0}, {0, 1, 0}, {0, 2, 1}, {0, 3, 1}, {0, 4, 0}}},
        {vec_t({cinfo_t(2, 1, 1), cinfo_t(2, 1, 0), cinfo_t(2, 1, 0)}),
         {{0, -2, 1}, {0, -1, 0}, {0, 0, 0}, {0, 1, 1}, {0, 2, 1}, {0, 3, 0}, {0, 4, 0}}},
        {vec_t({cinfo_t(2, -1, 0), cinfo_t(2, 1, 0), cinfo_t(2, 1, 0)}),
         {{0, -2, 0}, {0, -1, 1}, {0, 0, 1}, {0, 1, 0}, {0, 2, 0}, {0, 3, 1}, {0, 4, 1}}},
        {vec_t({cinfo_t(2, -1, 1), cinfo_t(2, 1, 0), cinfo_t(2, 1, 0)}),
         {{0, -2, 0}, {0, -1, 0}, {0, 0, 1}, {0, 1, 1}, {0, 2, 0}, {0, 3, 0}, {0, 4, 1}}},

    }));

    auto tc = cbag::layout::track_coloring(0, std::move(tc_vec));
    for (const auto[idx, htr, expect] : test_vec) {
        CAPTURE(tc, idx, htr);
        REQUIRE(tc.get_htr_parity(idx, htr) == expect);
    }
}
