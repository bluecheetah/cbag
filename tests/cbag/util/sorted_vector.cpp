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

#include <algorithm>
#include <set>
#include <vector>

#include <catch2/catch.hpp>

#include <cbag/util/sorted_vector.h>

namespace cu = cbag::util;

using sorted_vector = cu::sorted_vector<int>;
using model_vector = std::vector<int>;

// TODO: untested methods: find, find_exact, insert_back

void check_equal(const sorted_vector &v, const model_vector &expected, bool deep = false) {
    REQUIRE(v == expected);
    if (deep) {
        REQUIRE(expected == v);
        REQUIRE(v.size() == expected.size());
        REQUIRE(v.capacity() == expected.capacity());
        REQUIRE(v.empty() == expected.empty());
        REQUIRE(v.end() - v.begin() == expected.end() - expected.begin());
        REQUIRE(v.rend() - v.rbegin() == expected.rend() - expected.rbegin());
        if (v.empty()) {
            REQUIRE_THROWS_AS(v.at_front(), std::out_of_range);
            REQUIRE_THROWS_AS(v.at_back(), std::out_of_range);
        } else {
            REQUIRE(*v.begin() == *expected.begin());
            REQUIRE(*v.rbegin() == *expected.rbegin());
            REQUIRE(v.at_front() == expected.front());
            REQUIRE(v.at_back() == expected.back());
        }
    }
}

SCENARIO("sorted_vector constructors and accessors", "[sorted_vector]") {
    GIVEN("empty") {
        model_vector expected;

        THEN("vector constructor works") {
            sorted_vector v(expected);
            check_equal(v, expected, true);
            REQUIRE(v.size() == 0);
            REQUIRE(v.capacity() == 0);
            REQUIRE(v.empty() == true);
        }

        THEN("initializer list constructor works") {
            sorted_vector v{};
            check_equal(v, expected, true);
            REQUIRE(v.size() == 0);
            REQUIRE(v.capacity() == 0);
            REQUIRE(v.empty() == true);
        }

        sorted_vector v;

        THEN("default constructor works") {
            check_equal(v, expected, true);
            REQUIRE(v.size() == 0);
            REQUIRE(v.capacity() == 0);
            REQUIRE(v.empty() == true);
        }

        THEN("lower_bound/equal_range returns end") {
            auto iter_range = v.equal_range(0);
            REQUIRE(v.lower_bound(0) == v.end());
            REQUIRE(iter_range.first == v.end());
            REQUIRE(iter_range.second == v.end());
            REQUIRE(v.equal_size(0) == 0);
        }
    }

    THEN("non-empty initializer list constructor works") {
        model_vector expected{2, 4, 6};
        sorted_vector v{6, 2, 4};
        check_equal(v, expected, true);
    }

    GIVEN("not empty") {
        model_vector data = GENERATE(values<model_vector>({
            {6, 2, 4},
            {6, 6, 2, 4, 4, 4},
        }));
        model_vector expected(data);
        std::sort(expected.begin(), expected.end());

        sorted_vector v(data);

        THEN("vector constructor works") { check_equal(v, expected, true); }

        THEN("lower_bound works") {
            for (std::size_t idx = 0; idx < expected.size(); ++idx) {
                for (int offset = -1; offset < 2; ++offset) {
                    int val = expected[idx] + offset;
                    REQUIRE(v.lower_bound(val) - v.begin() ==
                            std::lower_bound(expected.begin(), expected.end(), val) -
                                expected.begin());
                }
            }
        }

        THEN("equal_range works") {
            for (std::size_t idx = 0; idx < expected.size(); ++idx) {
                for (int offset = -1; offset < 2; ++offset) {
                    int val = expected[idx] + offset;
                    auto pair1 = v.equal_range(val);
                    auto pair2 = std::equal_range(expected.begin(), expected.end(), val);
                    REQUIRE(pair1.first - v.begin() == pair2.first - expected.begin());
                    REQUIRE(pair1.second - v.begin() == pair2.second - expected.begin());
                    REQUIRE(v.equal_size(val) == pair2.second - pair2.first);
                }
            }
        }
    }
}

SCENARIO("sorted_vector insert_unique/emplace_unique/emplace_back test", "[sorted_vector]") {
    model_vector data = GENERATE(values<model_vector>({
        {6, 2, 4},
        {6, 6, 2, 4, 4, 4},
    }));

    int insert_mode = GENERATE(range(0, 3));

    sorted_vector v;
    std::set<int> model;

    for (auto val : data) {
        bool success = model.emplace(val).second;

        std::pair<sorted_vector::iterator, bool> ans;
        switch (insert_mode) {
        case 0:
            ans = v.insert_unique(val);
            break;
        case 1:
            ans = v.emplace_unique(val);
            break;
        default:
            ans = v.insert_unique(std::move(val));
            break;
        }
        REQUIRE(*(ans.first) == val);
        REQUIRE(ans.second == success);
        check_equal(v, model_vector(model.begin(), model.end()));
    }

    auto cur_size = v.size();
    auto last_val = v.at_back();
    THEN("emplace back works for larger and equal item") {
        int offset = GENERATE(range(0, 2));
        v.emplace_back(last_val + offset);
        REQUIRE(v.at_back() == last_val + offset);
        REQUIRE(v.size() == cur_size + 1);
    }

    THEN("emplace back fails for smaller item") {
        REQUIRE_THROWS_AS(v.emplace_back(last_val - 1), std::invalid_argument);
    }
}

SCENARIO("sorted_vector deletion", "[sorted_vector]") {
    GIVEN("some data") {

        model_vector expected{2, 4, 6, 8, 10};
        sorted_vector v(expected);

        std::size_t start = GENERATE(range(0, 4));

        THEN("deleting single element works") {
            auto iter1 = v.erase(v.begin() + start);
            auto iter2 = expected.erase(expected.begin() + start);
            REQUIRE(iter1 - v.begin() == iter2 - expected.begin());
            check_equal(v, expected);
        }
        THEN("deleting range works") {
            for (std::size_t stop = start; stop <= expected.size(); ++stop) {
                auto iter1 = v.erase(v.begin() + start, v.begin() + stop);
                auto iter2 = expected.erase(expected.begin() + start, expected.begin() + stop);
                REQUIRE(iter1 - v.begin() == iter2 - expected.begin());
                check_equal(v, expected);
            }
        }
    }
}
