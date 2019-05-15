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
#include <string>
#include <tuple>
#include <utility>

#include <catch2/catch.hpp>

#include <yaml-cpp/tuple.h>

#include <cbag/spirit/util.h>
#include <cbag/tests/util/io.h>
#include <cbag/util/name_convert.h>

TEST_CASE("CDBA valid names", "[spirit::name]") {
    using data_type = std::tuple<std::string, std::string>;
    auto fname = "tests/data/cbag/spirit/name/cdba_valid.yaml";

    auto [str_in, str_out] = GENERATE_COPY(read_test_vector<data_type>(fname));
    auto expect = (str_out.empty()) ? str_in : str_out;

    CAPTURE(str_in, expect);

    auto name_obj = cbag::util::parse_cdba_name(str_in);
    REQUIRE(name_obj.to_string(cbag::spirit::namespace_cdba{}) == expect);
}

TEST_CASE("CDBA invalid names", "[spirit::name]") {
    using data_type = std::string;
    auto fname = "tests/data/cbag/spirit/name/cdba_invalid.yaml";

    auto str_in = GENERATE_COPY(read_test_vector<data_type>(fname));

    CAPTURE(str_in);
    CHECK_THROWS_AS(cbag::util::parse_cdba_name(str_in), std::invalid_argument);
}

TEST_CASE("CDBA get_name_bits", "[spirit::name]") {
    using data_type = std::tuple<std::string, std::vector<std::string>>;
    auto fname = "tests/data/cbag/spirit/name/cdba_get_bits.yaml";

    auto [str_in, bit_list] = GENERATE_COPY(read_test_vector<data_type>(fname));

    CAPTURE(str_in, bit_list);

    auto name_obj = cbag::util::parse_cdba_name(str_in);
    auto output = std::vector<std::string>();
    cbag::spirit::util::get_name_bits(name_obj, std::back_inserter(output),
                                      cbag::spirit::namespace_cdba{});
    auto num = std::min(output.size(), bit_list.size());
    for (std::size_t idx = 0; idx < num; ++idx) {
        REQUIRE(output[idx] == bit_list[idx]);
    }
    REQUIRE(output.size() == bit_list.size());
}

TEST_CASE("get_partition_test", "[spirit::name]") {
    using data_type = std::tuple<std::string, std::vector<std::string>, cbag::cnt_t>;
    auto fname = "tests/data/cbag/spirit/name/cdba_partition.yaml";

    auto [test_name, str_list, chunk] = GENERATE_COPY(read_test_vector<data_type>(fname));

    CAPTURE(test_name, str_list, chunk);

    auto name_obj = cbag::util::parse_cdba_name(test_name);
    auto output = std::vector<cbag::spirit::ast::name>();
    cbag::spirit::util::get_partition(name_obj, chunk, std::back_inserter(output));
    auto num = std::min(output.size(), str_list.size());
    for (std::size_t idx = 0; idx < num; ++idx) {
        REQUIRE(output[idx].to_string(cbag::spirit::namespace_cdba{}) == str_list[idx]);
    }
    REQUIRE(output.size() == str_list.size());
}

TEST_CASE("CDBA repeat name", "[spirit::name]") {
    using data_type = std::tuple<std::string, std::string, cbag::cnt_t>;
    auto fname = "tests/data/cbag/spirit/name/cdba_repeat.yaml";

    auto [start, expect, mult] = GENERATE_COPY(read_test_vector<data_type>(fname));

    CAPTURE(start, expect, mult);

    auto name_obj = cbag::util::parse_cdba_name(start);
    auto ans = name_obj.repeat(mult).to_string(cbag::spirit::namespace_cdba{});
    REQUIRE(ans == expect);
}

TEST_CASE("CDBA to verilog", "[spirit::name]") {
    using data_type = std::tuple<std::string, std::string>;
    auto fname = "tests/data/cbag/spirit/name/cdba_to_verilog.yaml";

    auto [str_in, expect] = GENERATE_COPY(read_test_vector<data_type>(fname));

    CAPTURE(str_in, expect);

    auto name_obj = cbag::util::parse_cdba_name(str_in);
    REQUIRE(name_obj.to_string(cbag::spirit::namespace_verilog{}) == expect);
}

TEST_CASE("CDBA bounds to string", "[spirit::name]") {
    using data_type = std::tuple<std::string, std::string, std::array<cbag::cnt_t, 2>>;
    auto fname = "tests/data/cbag/spirit/name/cdba_bounds_to_str.yaml";

    auto [base, expect, bounds] = GENERATE_COPY(read_test_vector<data_type>(fname));

    CAPTURE(base, expect, bounds);

    auto ans = cbag::spirit::ast::to_string(base, bounds, cbag::spirit::namespace_cdba{});

    REQUIRE(ans == expect);
}
