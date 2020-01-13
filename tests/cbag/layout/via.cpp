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

#include <cbag/common/transformation_util.h>
#include <cbag/common/vector.h>
#include <cbag/layout/via_param.h>
#include <cbag/layout/via_util.h>
#include <cbag/polygon/enum.h>

#include <cbag/tests/util/io.h>
#include <cbag/yaml/int_array.h>
#include <cbag/yaml/rectangle_data.h>
#include <cbag/yaml/transformation.h>
#include <cbag/yaml/via_info.h>
#include <cbag/yaml/via_param.h>

#include <cbag/tests/layout/util.h>

using c_vector = cbag::vector;
using c_via_param = cbag::layout::via_param;
using c_box = cbag::box_t;
using c_xform = cbag::transformation;

namespace cbag {
namespace layout {

std::ostream &operator<<(std::ostream &stream, via_param obj) { return stream << obj.to_string(); }

} // namespace layout
} // namespace cbag

TEST_CASE("via_info::get_via_param()", "[via]") {
    using data_type =
        std::tuple<cbag::layout::via_info, c_vector, int, int, int, bool, c_via_param>;
    auto fname = "tests/data/cbag/layout/via/via_info.yaml";
    auto[vinfo, dim, vd_code, ex_code, aex_code, extend, expect] =
        GENERATE_COPY(read_test_vector<data_type>(fname));

    CAPTURE(vinfo.get_name(), dim, vd_code, ex_code, aex_code, extend);

    auto param = vinfo.get_via_param(dim, static_cast<cbag::direction_1d>(vd_code),
                                     static_cast<cbag::orientation_2d>(ex_code),
                                     static_cast<cbag::orientation_2d>(aex_code), extend);
    REQUIRE(param == expect);
}

TEST_CASE("tech_info::get_via_param()", "[via]") {
    auto tech_info = make_tech();
    auto vd = cbag::direction_1d::LOWER;
    auto bot_dir = cbag::orientation_2d::HORIZONTAL;
    auto top_dir = cbag::orientation_2d::VERTICAL;
    auto extend = true;

    using data_type = std::tuple<std::string, std::string, c_vector, c_via_param>;
    auto fname = "tests/data/cbag/layout/via/tech_info.yaml";
    auto[bl, tl, dim, ans] = GENERATE_COPY(read_test_vector<data_type>(fname));
    auto vid =
        tech_info->get_via_id(cbag::direction_1d::LOWER, layer_t_at(*tech_info, bl, "drawing"),
                              layer_t_at(*tech_info, tl, "drawing"));
    auto param =
        tech_info->get_via_param(dim, vid, cbag::direction_1d::LOWER, bot_dir, top_dir, extend);
    CAPTURE(bl, tl, vid, dim, vd, bot_dir, top_dir, extend);
    REQUIRE(param == ans);
}

TEST_CASE("get via cuts", "[via]") {
    using data_type = std::tuple<c_xform, std::string, c_via_param, std::vector<c_box>>;
    auto fname = "tests/data/cbag/layout/via/via_cuts.yaml";

    auto[xform, via_id, via_param, expect] = GENERATE_COPY(read_test_vector<data_type>(fname));

    auto ans = std::vector<c_box>();
    auto via = cbag::layout::via(xform, via_id, via_param);
    cbag::layout::get_via_cuts(via, std::back_inserter(ans));
    REQUIRE(ans == expect);
}
