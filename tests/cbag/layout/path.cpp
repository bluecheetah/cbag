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

#include <yaml-cpp/tuple.h>

#include <cbag/layout/path_util.h>
#include <cbag/tests/util/io.h>
#include <cbag/yaml/enum.h>
#include <cbag/yaml/point_data.h>
#include <cbag/yaml/polygon_45_data.h>

using pt_type = cbag::point_t;

TEST_CASE("make_path", "[layout::path_util]") {
    using data_type = std::tuple<std::vector<pt_type>, cbag::offset_t,
                                 std::array<cbag::end_style, 3>, cbag::layout::poly_45_t>;
    auto fname = "tests/data/cbag/layout/path/make_path.yaml";

    auto [pt_list, half_w, styles, expect] = GENERATE_COPY(read_test_vector<data_type>(fname));

    auto poly = cbag::layout::make_path(pt_list, half_w, styles[0], styles[1], styles[2]);

    poly.clean();
    CAPTURE(poly);

    std::vector<cbag::layout::poly_45_t> poly_vec;
    poly.get_polygons(std::back_inserter(poly_vec));

    REQUIRE(poly_vec.size() == 1);
    REQUIRE(poly_vec[0] == expect);
}

TEST_CASE("path_to_poly_45", "[layout::path_util]") {
    using data_type = std::tuple<pt_type, pt_type, cbag::offset_t, int, int, std::vector<pt_type>>;
    auto fname = "tests/data/cbag/layout/path/path_to_poly45.yaml";

    auto [p1, p2, w_half, sty0, sty1, expect] = GENERATE_COPY(read_test_vector<data_type>(fname));

    CAPTURE(p1, p2, w_half, sty0, sty1, expect);

    auto vec = cbag::layout::path_to_poly45(p1[0], p1[1], p2[0], p2[1], w_half,
                                            static_cast<cbag::end_style>(sty0),
                                            static_cast<cbag::end_style>(sty1));

    CAPTURE(vec);

    REQUIRE(vec == expect);
}
