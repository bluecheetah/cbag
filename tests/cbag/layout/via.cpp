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

#include <cbag/common/transformation_util.h>
#include <cbag/common/vector.h>
#include <cbag/layout/via_param.h>
#include <cbag/layout/via_util.h>
#include <cbag/polygon/enum.h>

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

TEST_CASE("get_via_param()", "[via]") {
    auto tech_info = make_tech();
    auto bot_dir = cbag::orientation_2d::HORIZONTAL;
    auto top_dir = cbag::orientation_2d::VERTICAL;
    auto extend = true;

    auto [bl, tl, dim,
          ans] = GENERATE(values<std::tuple<std::string, std::string, c_vector, c_via_param>>({
        // 1 via solutions
        {"M1CA", "M2CA", c_vector{64, 64},
         c_via_param(1, 1, 64, 64, 0, 0, 80, 80, 0, 0, 0, 0, 80, 80)},
        {"M1CA", "M2CA", c_vector{68, 68},
         c_via_param(1, 1, 64, 64, 0, 0, 80, 80, 2, 2, 2, 2, 80, 80)},
        {"M1CA", "M2CA", c_vector{72, 72},
         c_via_param(1, 1, 64, 64, 0, 0, 78, 78, 4, 4, 4, 4, 78, 78)},
        {"M1CA", "M2CA", c_vector{168, 64},
         c_via_param(1, 1, 128, 64, 0, 0, 40, 40, 0, 0, 20, 20, 20, 20)},
        {"M1CB", "M2CB", c_vector{64, 168},
         c_via_param(1, 1, 64, 128, 0, 0, 20, 20, 20, 20, 0, 0, 40, 40)},
        // 0 via solutions
        {"M1CA", "M2CB", c_vector{64, 62}, c_via_param(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0)},
        {"M1CB", "M2CA", c_vector{62, 64}, c_via_param(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0)},
        {"M1CB", "M2CB", c_vector{62, 62}, c_via_param(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0)},
    }));

    auto param = tech_info->get_via_param(
        dim,
        tech_info->get_via_id(cbag::direction_1d::LOWER, layer_t_at(*tech_info, bl, "drawing"),
                              layer_t_at(*tech_info, tl, "drawing")),
        cbag::direction_1d::LOWER, bot_dir, top_dir, extend);
    CAPTURE(bl, tl, dim);
    REQUIRE(param == ans);
}

TEST_CASE("get via cuts", "[via]") {
    using data_type = std::tuple<c_xform, std::string, c_via_param, std::vector<c_box>>;

    auto [xform, via_id, via_param, expect] = GENERATE(values<data_type>({
        {cbag::transformation(0, 0), "M2_M1",
         c_via_param(1, 1, 32, 32, 0, 0, 14, 14, 14, 14, 14, 14, 14, 14),
         std::vector<c_box>{c_box(-16, -16, 16, 16)}},
    }));

    std::vector<c_box> ans;
    auto via = cbag::layout::via(xform, via_id, via_param);
    cbag::layout::get_via_cuts(via, std::back_inserter(ans));
    REQUIRE(ans == expect);
}
