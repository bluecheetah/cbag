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

#include <cstdlib>

#include <catch2/catch.hpp>

#include <fmt/core.h>

#include <yaml-cpp/tuple.h>

#include <cbag/tests/util/io.h>
#include <cbag/util/string.h>

TEST_CASE("expand_env", "[util::string]") {
    using data_type = std::tuple<std::string, std::string, std::string>;
    auto fname = "tests/data/cbag/util/string/expand_env.yaml";

    auto [prefix, suffix, env_name] = GENERATE_COPY(read_test_vector<data_type>(fname));

    auto env_val_ptr = std::getenv(env_name.c_str());
    auto env_val = (env_val_ptr) ? std::string(env_val_ptr) : std::string("");
    auto str_in = fmt::format("{}${{{}}}{}", prefix, env_name, suffix);
    auto expect = fmt::format("{}{}{}", prefix, env_val, suffix);

    CAPTURE(str_in, expect);

    REQUIRE(cbag::util::expand_env(str_in) == expect);
}
