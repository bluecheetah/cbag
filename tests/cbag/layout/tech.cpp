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

#include <cbag/enum/space_type.h>

#include <cbag/tests/layout/util.h>

TEST_CASE("technology layer/purpose lookup", "[tech]") {
    auto tech_info = make_tech();

    auto[lay_name, lay_id] = GENERATE(values<std::pair<std::string, cbag::lay_t>>({
        {"M1", 60},
        {"M2", 80},
        {"M3", 100},
        {"M4", 110},
    }));

    auto[purp_name, purp_id] = GENERATE(values<std::pair<std::string, cbag::purp_t>>({
        {"drawing", 4294967295},
        {"pin", 251},
        {"label", 237},
    }));

    REQUIRE(lay_id == cbag::layout::layer_id_at(*tech_info, lay_name));
    REQUIRE(purp_id == cbag::layout::purpose_id_at(*tech_info, purp_name));
    REQUIRE(tech_info->get_purpose_id("drawing") == cbag::layout::purpose_id_at(*tech_info, ""));
    REQUIRE(cbag::layer_t(lay_id, purp_id) ==
            cbag::layout::layer_t_at(*tech_info, lay_name, purp_name));
}

TEST_CASE("technology get_min_space", "[tech]") {
    auto tech_info = make_tech();

    auto[sp_type, lay_name, w, min_sp] =
        GENERATE(values<std::tuple<cbag::space_type, std::string, cbag::offset_t, cbag::offset_t>>({
            {cbag::space_type::DIFF_COLOR, "M1CA", 20, 64},
            {cbag::space_type::DIFF_COLOR, "M2CA", 200, 64},
            {cbag::space_type::DIFF_COLOR, "M4", 20, 96},
            {cbag::space_type::DIFF_COLOR, "M4", 198, 96},
            {cbag::space_type::DIFF_COLOR, "M4", 200, 144},
            {cbag::space_type::DIFF_COLOR, "M4", 9999, 440},
            {cbag::space_type::SAME_COLOR, "M5", 178, 160},
            {cbag::space_type::SAME_COLOR, "M6", 180, 200},
        }));

    auto key = layer_t_at(*tech_info, lay_name, "drawing");
    REQUIRE(tech_info->get_min_space(key, w, sp_type, false) == min_sp);
}

TEST_CASE("technology get_next_length", "[tech]") {
    auto tech_info = make_tech();

    auto[lay_name, w, min_len, even] =
        GENERATE(values<std::tuple<std::string, cbag::offset_t, cbag::offset_t, bool>>({
            {"M1CA", 2, 12352, true},
            {"M1CA", 20, 1236, true},
            {"M1CA", 40, 618, true},
            {"M1CA", 40, 618, false},
            {"M1CA", 80, 310, true},
            {"M1CA", 80, 309, false},
            {"M1CA", 100, 248, true},
            {"M1CA", 100, 248, false},
            {"M1CA", 1600, 64, true},
            {"M1CA", 1600, 64, false},
        }));

    auto key = layer_t_at(*tech_info, lay_name, "drawing");
    CAPTURE(lay_name, w, min_len, even);
    REQUIRE(tech_info->get_next_length(key, cbag::orientation_2d::Y, w, 0, even) == min_len);
}
