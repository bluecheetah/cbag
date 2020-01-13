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

#include <yaml-cpp/tuple.h>

#include <cbag/layout/convex.h>

#include <cbag/yaml/point_data.h>
#include <cbag/yaml/polygon_45_data.h>
#include <cbag/yaml/rectangle_data.h>

#include <cbag/tests/util/io.h>

using c_poly_45_t = cbag::layout::convex::poly_45_t;
using c_box_t = cbag::box_t;
using c_point_t = cbag::point_t;

TEST_CASE("get_stairs_bbox", "[convex]") {
    using data_type = std::tuple<c_poly_45_t, std::vector<cbag::box_t>>;
    auto fname = "tests/data/cbag/layout/convex/get_stairs_bbox.yaml";
    auto[p, expect] = GENERATE_COPY(read_test_vector<data_type>(fname));

    auto ans = cbag::layout::convex::get_stairs_bbox(p.begin(), p.end());

    CAPTURE(p, ans, expect);

    REQUIRE(expect.size() == 4);
    REQUIRE(ans[0] == expect[0]);
    REQUIRE(ans[1] == expect[1]);
    REQUIRE(ans[2] == expect[2]);
    REQUIRE(ans[3] == expect[3]);
}

TEST_CASE("get_stairs", "[convex]") {
    using data_type = std::tuple<std::vector<c_point_t>, int, std::vector<c_point_t>>;
    auto fname = "tests/data/cbag/layout/convex/get_stairs.yaml";
    auto[pt_vec, code, expect] = GENERATE_COPY(read_test_vector<data_type>(fname));

    auto ans = cbag::layout::convex::get_stairs(pt_vec, code);

    CAPTURE(pt_vec, code, ans, expect);
    REQUIRE(ans == expect);
}

TEST_CASE("get_cr_convex_hull", "[convex]") {
    using data_type = std::tuple<std::vector<c_point_t>, c_poly_45_t>;
    auto fname = "tests/data/cbag/layout/convex/get_cr_convex_hull.yaml";
    auto[p, expect] = GENERATE_COPY(read_test_vector<data_type>(fname));

    auto ans = cbag::layout::convex::get_cr_convex_hull(p.begin(), p.end());

    CAPTURE(p, expect);
    REQUIRE(ans == expect);
}
