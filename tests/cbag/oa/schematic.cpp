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

#include <filesystem>
#include <sstream>
#include <string>

#include <catch2/catch.hpp>

#include <fmt/core.h>

#include <cbag/oa/database.h>
#include <cbag/schematic/cellview.h>

#include <cbag/tests/util/io.h>

cbag::sch::cellview read_cv(const std::string &lib_name, const std::string &cell_name,
                            const std::string &view_name, const std::string &lib_file) {
    cbagoa::database db(lib_file);
    return db.read_sch_cellview(lib_name, cell_name, view_name);
}

std::vector<std::string> split(const std::string &s) {
    auto buf = std::stringstream(s);
    std::vector<std::string> ans;
    std::string tmp;
    while (std::getline(buf, tmp, '\n')) {
        ans.emplace_back(tmp);
    }
    return ans;
}

void compare_strings(const std::string &s1, const std::string &s2) {
    auto vec1 = split(s1);
    auto vec2 = split(s2);

    auto n = std::min(vec1.size(), vec2.size());
    for (std::size_t idx = 0; idx < n; ++idx) {
        CAPTURE(idx);
        REQUIRE(vec1[idx] == vec2[idx]);
    }
    CAPTURE(n);
    REQUIRE(vec1.size() == vec2.size());
}

std::string write_cv(const cbag::sch::cellview &cv, const std::string &cell_name,
                     const std::string &view_name, const std::string &output_dir,
                     const std::string &expect_dir) {
    std::string out_fname, expect_fname;

    if (view_name == "schematic") {
        out_fname = fmt::format("{}/{}.yaml", output_dir, cell_name);
        expect_fname = fmt::format("{}/{}.yaml", expect_dir, cell_name);
    } else {
        out_fname = fmt::format("{}/{}.{}.yaml", output_dir, cell_name, view_name);
        expect_fname = fmt::format("{}/{}.{}.yaml", expect_dir, cell_name, view_name);
    }
    cv.to_file(out_fname);

    // check two file equivalent
    std::string output_str = read_file(out_fname);
    std::string expect_str = read_file(expect_fname);
    CAPTURE(cell_name);
    compare_strings(output_str, expect_str);

    return out_fname;
}

TEST_CASE("read OA schematic", "[oa]") {
    auto root_dir = std::filesystem::path("tests/data/OA");
    auto lib_name = "test_netlist";
    auto lib_dir = root_dir / lib_name;
    auto lib_file = root_dir / "cds.lib";
    auto output_dir = root_dir / "output" / "yaml";
    auto expect_dir = root_dir / "expect" / "yaml";

    auto cell_name = GENERATE_COPY(read_directories(lib_dir));
    auto view_name = GENERATE(values<std::string>({"schematic", "symbol"}));
    // read cellview
    auto cv = read_cv(lib_name, cell_name, view_name, lib_file);
    // write cellview to yaml
    auto out_fname = write_cv(cv, cell_name, view_name, output_dir, expect_dir);
    // check we can read cellview
    cbag::sch::cellview cv_test(out_fname);
}

TEST_CASE("reading cds.lib with library that does not exist", "[oa]") {
    auto lib_file = "tests/data/OA/cds_bad.lib";
    REQUIRE_THROWS_AS(cbagoa::database(lib_file), std::runtime_error);
}
