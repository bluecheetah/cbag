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

#include <array>
#include <vector>

#include <catch2/catch.hpp>

#include <cbag/util/interval.h>
#include <cbag/yaml/int_array.h>
#include <yaml-cpp/tuple.h>

#include <cbag/tests/util/io.h>

namespace cu = cbag::util;

using coord_t = cbag::offset_t;

using intv_type = std::array<coord_t, 2>;

TEST_CASE("disjoint_intvs is empty", "[disjoint_intvs]") {
    cu::disjoint_intvs<intv_type> iset;
    REQUIRE(iset.size() == 0);
    REQUIRE(iset.empty() == true);

    THEN("start and stop errors") {
        REQUIRE_THROWS_AS(iset.start(), std::out_of_range);
        REQUIRE_THROWS_AS(iset.stop(), std::out_of_range);
    }

    THEN("complement works") {
        coord_t lower = 0;
        coord_t upper = 6;
        auto comp = iset.get_complement(intv_type{lower, upper});
        REQUIRE(comp.start() == lower);
        REQUIRE(comp.stop() == upper);
        REQUIRE(comp.size() == 1);
    }

    WHEN("adding values") {
        bool success = iset.emplace(false, false, false, intv_type{8, 10});
        REQUIRE(success == true);
        success = iset.emplace(false, false, false, intv_type{4, 6});
        REQUIRE(success == true);
        success = iset.emplace(false, false, false, intv_type{1, 3});
        REQUIRE(success == true);

        WHEN("adding overlap interval") {
            // adding overlap interval fails
            success = iset.emplace(false, false, false, intv_type{2, 5});
            REQUIRE(success == false);

            success = iset.emplace(true, false, false, intv_type{2, 5});
            REQUIRE(success == true);
            REQUIRE(iset.size() == 2);
            REQUIRE(*iset.begin() == intv_type{1, 6});

            // adding abut interval fails
            success = iset.emplace(false, false, false, intv_type{10, 12});
            REQUIRE(success == false);

            // abut flag enables adding abut intervals
            success = iset.emplace(false, true, false, intv_type{10, 12});
            REQUIRE(success == true);
            // abut = True, merge = False does not join intervals
            REQUIRE(iset.size() == 3);
            REQUIRE(*std::next(iset.begin(), 1) == intv_type{8, 10});
            REQUIRE(*std::next(iset.begin(), 2) == intv_type{10, 12});

            // test abut=True, merge=True
            success = iset.emplace(true, true, false, intv_type{12, 14});
            REQUIRE(success == true);
            REQUIRE(iset.size() == 3);
            REQUIRE(*std::next(iset.begin(), 2) == intv_type{10, 14});
        }

        WHEN("removing overlaps") {
            iset.remove_overlaps(intv_type{2, 9});
            REQUIRE(iset.empty() == true);
        }

        WHEN("subtract overlaps") {
            iset.subtract(intv_type{2, 9});
            REQUIRE(iset.size() == 2);
            REQUIRE(*iset.begin() == intv_type{1, 2});
            REQUIRE(*(iset.begin() + 1) == intv_type{9, 10});
        }
    }
}

TEST_CASE("subtract", "[disjoint_intvs]") {
    using data_type = std::tuple<intv_type, std::vector<intv_type>, std::vector<intv_type>>;
    auto fname = "tests/data/test_util/intvs_subtract.yaml";

    auto[tot_intv, sub_list, expect] = GENERATE_COPY(read_test_vector<data_type>(fname));

    cu::disjoint_intvs<intv_type> intvs;
    intvs.emplace(false, false, false, tot_intv);

    for (const auto &intv : sub_list) {
        intvs.subtract(intv);
    }

    CAPTURE(tot_intv, sub_list, expect);
    auto iter = intvs.begin();
    for (std::size_t idx = 0; idx < std::min(intvs.size(), expect.size()); ++idx, ++iter) {
        REQUIRE(*iter == expect[idx]);
    }

    REQUIRE(intvs.size() == expect.size());
}

TEST_CASE("union", "[disjoint_intvs]") {
    using data_type =
        std::tuple<std::vector<intv_type>, std::vector<intv_type>, std::vector<intv_type>>;
    auto fname = "tests/data/test_util/intvs_union.yaml";

    auto[intvs1, intvs2, intvs3] = GENERATE_COPY(read_test_vector<data_type>(fname));

    CAPTURE(intvs1, intvs2, intvs3);

    auto lhs = cu::disjoint_intvs<intv_type>(std::move(intvs1));
    auto rhs = cu::disjoint_intvs<intv_type>(std::move(intvs2));
    auto expect = cu::disjoint_intvs<intv_type>(std::move(intvs3));

    lhs += rhs;

    REQUIRE(lhs == expect);
}

TEST_CASE("expand_space", "[disjoint_intvs]") {
    using data_type =
        std::tuple<std::array<cbag::offset_t, 3>, std::vector<intv_type>, std::vector<intv_type>>;
    auto fname = "tests/data/test_util/intvs_expand_space.yaml";

    auto[info, intvs1, intvs2] = GENERATE_COPY(read_test_vector<data_type>(fname));
    auto[lower, upper, delta] = info;

    CAPTURE(info, intvs1, intvs2);

    auto intvs = cu::disjoint_intvs<intv_type>(std::move(intvs1));
    auto expect = cu::disjoint_intvs<intv_type>(std::move(intvs2));

    intvs.expand_space(lower, upper, delta);

    REQUIRE(intvs == expect);
}

TEST_CASE("fix_drc", "[disjoint_intvs]") {
    using data_type =
        std::tuple<std::array<cbag::offset_t, 4>, std::vector<intv_type>, std::vector<intv_type>>;
    auto fname = "tests/data/test_util/intvs_fix_drc.yaml";

    auto[info, intvs1, intvs2] = GENERATE_COPY(read_test_vector<data_type>(fname));
    auto[lower, upper, min_len, min_sp] = info;

    CAPTURE(info, intvs1, intvs2);

    auto intvs = cu::disjoint_intvs<intv_type>(std::move(intvs1));
    auto expect = cu::disjoint_intvs<intv_type>(std::move(intvs2));

    intvs.fix_drc(lower, upper, min_len, min_sp);

    REQUIRE(intvs == expect);
}

TEST_CASE("upper_bound_int", "[disjoint_intvs]") {
    using data_type = std::tuple<std::vector<intv_type>, coord_t, intv_type>;
    auto fname = "tests/data/cbag/util/interval/upper_bound_int.yaml";

    auto[intvs, q, expect] = GENERATE_COPY(read_test_vector<data_type>(fname));

    CAPTURE(intvs, q, expect);

    auto data = cu::disjoint_intvs<intv_type>(std::move(intvs));
    auto iter = data.upper_bound(q);
    if (expect[1] < expect[0]) {
        REQUIRE(iter == data.end());
    } else {
        REQUIRE(*iter == expect);
    }
}
