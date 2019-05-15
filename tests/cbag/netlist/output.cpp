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
#include <cctype>
#include <locale>
#include <memory>
#include <unordered_set>

#include <catch2/catch.hpp>

#include <fmt/core.h>

#include <cbag/common/typedefs.h>
#include <cbag/netlist/netlist.h>

#include <cbag/tests/util/io.h>

using name_cv_vec = std::vector<
    std::pair<std::string, std::pair<std::unique_ptr<cbag::sch::cellview>, std::string>>>;

// trim from start (in place)
static inline void ltrim(std::string &s) {
    s.erase(s.begin(), std::find_if(s.begin(), s.end(), [](int ch) { return !std::isspace(ch); }));
}

// trim from end (in place)
static inline void rtrim(std::string &s) {
    s.erase(std::find_if(s.rbegin(), s.rend(), [](int ch) { return !std::isspace(ch); }).base(),
            s.end());
}

// trim from both ends (in place)
static inline std::string trim(std::string s) {
    ltrim(s);
    rtrim(s);
    return s;
}

std::string get_extension(cbag::design_output netlist_type) {
    switch (netlist_type) {
    case cbag::design_output::CDL:
        return "cdl";
    case cbag::design_output::VERILOG:
        return "v";
    default:
        return "netlist";
    }
}

void populate_name_cv_list(const char *fmt_str, const std::string &yaml_dir,
                           const std::string &top_cell, const std::string cv_name,
                           name_cv_vec &name_cv_list, std::unordered_set<std::string> &recorded) {
    auto cv = std::make_unique<cbag::sch::cellview>(fmt::format(fmt_str, yaml_dir, top_cell));

    // write subcells
    for (const auto &[inst_name, inst_ptr] : cv->instances) {
        auto &cur_cell = inst_ptr->cell_name;
        if (recorded.find(cur_cell) == recorded.end()) {
            populate_name_cv_list(fmt_str, yaml_dir, cur_cell, cur_cell, name_cv_list, recorded);
        }
    }

    // write this cellview
    name_cv_list.emplace_back(
        cv_name, std::pair<std::unique_ptr<cbag::sch::cellview>, std::string>(std::move(cv), ""));
    recorded.emplace(top_cell);
}

void populate_name_cv_list_netlist(const char *fmt_str, const std::string &yaml_dir,
                                   const std::string &top_cell, const std::string cv_name,
                                   name_cv_vec &name_cv_list, const std::string &netlist) {
    auto cv = std::make_unique<cbag::sch::cellview>(fmt::format(fmt_str, yaml_dir, top_cell));

    // write subcells
    for (const auto &[inst_name, inst_ptr] : cv->instances) {
        name_cv_list.emplace_back(
            inst_ptr->cell_name,
            std::pair<std::unique_ptr<cbag::sch::cellview>, std::string>(nullptr, ""));
    }

    // write this cellview
    name_cv_list.emplace_back(cv_name, std::pair<std::unique_ptr<cbag::sch::cellview>, std::string>(
                                           std::move(cv), netlist));
}

SCENARIO("netlist generation", "[netlist]") {
    GIVEN("cellviews from yaml files") {
        std::string yaml_dir = "tests/data/test_netlist_yaml";
        std::string output_dir = "tests/data/test_outputs/netlist";
        std::string expect_dir = "tests/data/test_netlist_netlist";

        std::string cell_name = GENERATE(values<std::string>({
            "nmos4_standard",
            "pmos4_standard",
            "cv_simple",
            "cv_bus_term",
            "cv_inst_w_bus",
            "cv_array_inst_simple",
            "cv_array_inst_w_bus",
        }));
        std::pair<cbag::design_output, bool> fmt_shell =
            GENERATE(values<std::pair<cbag::design_output, bool>>({
                {cbag::design_output::CDL, false},
                {cbag::design_output::VERILOG, true},
                {cbag::design_output::VERILOG, false},
            }));

        auto format = fmt_shell.first;
        bool shell = fmt_shell.second;
        const char *fmt_str = "{}/{}.yaml";
        name_cv_vec name_cv_list;
        std::unordered_set<std::string> recorded;
        populate_name_cv_list(fmt_str, yaml_dir, cell_name, "TEST", name_cv_list, recorded);
        bool flat = false;
        cbag::cnt_t rmin = 2000;

        std::string ext_str = get_extension(format);
        std::string fname = fmt::format("{}/{}_{:d}.{}", output_dir, cell_name, shell, ext_str);
        std::string expect_fname =
            fmt::format("{}/{}_{:d}.{}", expect_dir, cell_name, shell, ext_str);

        CAPTURE(cell_name);
        CAPTURE(ext_str);
        THEN("writes netlist correctly") {
            std::vector<std::string> inc_list;
            std::string append_file;
            cbag::sch::netlist_map_t netlist_map;
            cbag::netlist::write_netlist(name_cv_list, fname, format, netlist_map, append_file,
                                         inc_list, flat, shell, rmin);
            std::string output_str = read_file(fname);
            std::string expect_str = read_file(expect_fname);
            REQUIRE(output_str == expect_str);
        }
    }
}

TEST_CASE("netlist generation with netlist override", "[netlist]") {
    auto yaml_dir = "tests/data/test_netlist_yaml";
    auto output_dir = "tests/data/test_outputs/netlist";
    auto flat = false;
    auto rmin = 2000;
    auto fmt_str = "{}/{}.yaml";
    auto netlist_expect = "foobar";

    auto cell_name = GENERATE(values<std::string>({
        "cv_array_inst_simple",
        "cv_array_inst_w_bus",
    }));

    auto [format, shell] = GENERATE(values<std::pair<cbag::design_output, bool>>({
        {cbag::design_output::SYSVERILOG, false},
    }));

    name_cv_vec name_cv_list;
    populate_name_cv_list_netlist(fmt_str, yaml_dir, cell_name, "TEST", name_cv_list,
                                  netlist_expect);

    auto ext_str = get_extension(format);
    auto fname = fmt::format("{}/{}_{:d}.{}", output_dir, cell_name, shell, ext_str);

    CAPTURE(cell_name);
    CAPTURE(ext_str);

    std::vector<std::string> inc_list;
    std::string append_file;
    cbag::sch::netlist_map_t netlist_map;
    cbag::netlist::write_netlist(name_cv_list, fname, format, netlist_map, append_file, inc_list,
                                 flat, shell, rmin);
    std::string output_str = read_file(fname);
    REQUIRE(trim(output_str) == netlist_expect);
}
