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

#include <map>
#include <string>
#include <utility>
#include <vector>

#include <catch2/catch.hpp>

#include <cbag/util/sorted_map.h>

namespace cu = cbag::util;

using sorted_map = cu::sorted_map<int, std::string>;
using model_map = std::map<int, std::string>;

void check_equal(const sorted_map &v, const model_map &m) {
    REQUIRE(v.size() == m.size());
    auto b1 = v.begin();
    auto b2 = m.begin();
    auto e1 = v.end();
    auto e2 = m.end();
    for (; b1 != e1 && b2 != e2; ++b1, ++b2) {
        REQUIRE(b1->first == b2->first);
        REQUIRE(b1->second == b2->second);
    }
}

bool test_equal(const sorted_map &v, const model_map &m) {
    if (v.size() != m.size())
        return false;
    auto b1 = v.begin();
    auto b2 = m.begin();
    auto e1 = v.end();
    auto e2 = m.end();
    for (; b1 != e1 && b2 != e2; ++b1, ++b2) {
        if (b1->first != b2->first)
            return false;
        if (b1->second != b2->second)
            return false;
    }
    return true;
}

template <class Iter1, class Iter2>
void check_equal_iter(const Iter1 &it1, const Iter2 &it2, const Iter1 &e1, const Iter2 &e2) {
    if (it1 == e1)
        REQUIRE(it2 == e2);
    else {
        REQUIRE(it1->first == it2->first);
        REQUIRE(it1->second == it2->second);
    }
}

SCENARIO("sorted_map constructors and accessors", "[sorted_map]") {
    GIVEN("empty") {
        model_map expected;

        THEN("initializer list constructor works") {
            sorted_map v{};
            check_equal(v, expected);
            REQUIRE(v.size() == 0);
            REQUIRE(v.capacity() == 0);
            REQUIRE(v.empty() == true);
        }

        sorted_map v;

        THEN("default constructor works") {
            check_equal(v, expected);
            REQUIRE(v.size() == 0);
            REQUIRE(v.capacity() == 0);
            REQUIRE(v.empty() == true);
        }

        THEN("lower_bound/upper_bound/equal_range returns end") {
            auto iter_range = v.equal_range(0);
            REQUIRE(v.lower_bound(0) == v.end());
            REQUIRE(v.upper_bound(0) == v.end());
            REQUIRE(iter_range.first == v.end());
            REQUIRE(iter_range.second == v.end());
        }

        THEN("insert_or_assign works") {
            expected.insert_or_assign(2, "hi");
            v.insert_or_assign(2, "hi");
            check_equal(v, expected);
        }
    }

    GIVEN("not empty") {
        std::vector<sorted_map::value_type> data =
            GENERATE(values<std::vector<sorted_map::value_type>>({
                {{3, "foo"}},
                {{-3, "abs"}, {17, "hi"}, {18, "bye"}, {1, "current"}},
            }));

        THEN("non-empty initializer list constructor works") {
            model_map expected{{3, "foo"}, {6, "bar"}};
            sorted_map v{{3, "foo"}, {6, "bar"}};
            check_equal(v, expected);
        }

        model_map expected(data.begin(), data.end());

        sorted_map v;
        v.reserve(expected.size());
        v.insert(expected.begin(), expected.end());
        check_equal(v, expected);

        THEN("lower_bound works") {
            for (const auto &p : expected) {
                for (int offset = -1; offset < 2; ++offset) {
                    int val = p.first + offset;
                    check_equal_iter(v.lower_bound(val), expected.lower_bound(val), v.end(),
                                     expected.end());
                }
            }
        }

        THEN("equal_range works") {
            for (const auto &p : expected) {
                for (int offset = -1; offset < 2; ++offset) {
                    int val = p.first + offset;
                    auto pair1 = v.equal_range(val);
                    auto pair2 = expected.equal_range(val);
                    check_equal_iter(pair1.first, pair2.first, v.cend(), expected.end());
                    check_equal_iter(pair1.second, pair2.second, v.cend(), expected.end());
                }
            }
        }
    }
}

SCENARIO("sorted_map insert/emplace test", "[sorted_map]") {
    std::vector<sorted_map::value_type> data =
        GENERATE(values<std::vector<sorted_map::value_type>>({
            {{3, "foo"}},
            {{-3, "abs"}, {17, "hi"}, {18, "bye"}, {1, "current"}},
        }));

    int insert_mode = GENERATE(range(0, 3));

    sorted_map v;
    model_map expected;

    for (auto const &val : data) {
        bool success = expected.emplace(val).second;

        std::pair<sorted_map::iterator, bool> ans;
        switch (insert_mode) {
        case 0:
            ans = v.insert(val);
            break;
        case 1:
            ans = v.emplace(val.first, val.second);
            break;
        default:
            ans = v.insert(std::move(val));
        }
        REQUIRE(*(ans.first) == val);
        REQUIRE(ans.second == success);
        check_equal(v, expected);
    }
}

SCENARIO("sorted_map deletion", "[sorted_map]") {
    GIVEN("some data") {
        model_map expected{{4, "abs"}, {5, "hi"}, {8, "bye"}, {1, "current"}};

        sorted_map v;
        v.reserve(expected.size());
        v.insert(expected.begin(), expected.end());

        int del_key = GENERATE(range(0, 10));

        THEN("erase works") {
            REQUIRE(v.erase(del_key) == expected.erase(del_key));
            check_equal(v, expected);
        }
    }
}

SCENARIO("sorted_map replace_key", "[sorted_map]") {

    model_map expected{{-3, "abs"}, {17, "hi"}, {18, "bye"}, {1, "current"}};
    sorted_map v;
    v.reserve(expected.size());
    v.insert(expected.begin(), expected.end());
    check_equal(v, expected);

    THEN("replace_key works") {
        int from = GENERATE(range(-4, 20));
        int to = GENERATE(range(-4, 20));

        auto test_iter = v.replace_key(from, to);
        auto find_iter = expected.find(from);
        if (find_iter == expected.end() || (from != to && expected.find(to) != expected.end())) {
            REQUIRE(test_iter == v.end());
        } else {
            if (from != to) {
                expected[to] = find_iter->second;
                expected.erase(find_iter);
            }
            check_equal(v, expected);
        }
    }
}
