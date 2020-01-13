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
#include <filesystem>
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
    case cbag::design_output::SYSVERILOG:
        return "sv";
    default:
        return "netlist";
    }
}

void populate_name_cv_list(const char *fmt_str, const std::string &yaml_dir,
                           const std::string &top_cell, const std::string cv_name,
                           name_cv_vec &name_cv_list, std::unordered_set<std::string> &recorded) {
    auto cv = std::make_unique<cbag::sch::cellview>(fmt::format(fmt_str, yaml_dir, top_cell));

    // write subcells
    for (const auto & [ inst_name, inst_ptr ] : cv->instances) {
        auto &cur_cell = inst_ptr->cell_name;
        if (recorded.find(cur_cell) == recorded.end() && inst_ptr->lib_name != "basic") {
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
    for (const auto & [ inst_name, inst_ptr ] : cv->instances) {
        if (inst_ptr->lib_name != "basic") {
            name_cv_list.emplace_back(
                inst_ptr->cell_name,
                std::pair<std::unique_ptr<cbag::sch::cellview>, std::string>(nullptr, ""));
        }
    }

    // write this cellview
    name_cv_list.emplace_back(cv_name, std::pair<std::unique_ptr<cbag::sch::cellview>, std::string>(
                                           std::move(cv), netlist));
}

cbag::sch::netlist_map_t make_netlist_map() {
    auto ans = cbag::sch::netlist_map_t();
    auto lib_map = cbag::sch::lib_map_t();

    auto no_conn_info = cbag::sch::cellview_info("basic", "noConn", true);
    no_conn_info.io_terms.push_back("noConn");
    no_conn_info.ignore = true;
    lib_map.emplace("noConn", std::move(no_conn_info));

    auto thru_info = cbag::sch::cellview_info("basic", "cds_thru", true);
    thru_info.io_terms.push_back("src");
    thru_info.io_terms.push_back("dst");
    lib_map.emplace("cds_thru", std::move(thru_info));

    ans.emplace("basic", std::move(lib_map));
    return ans;
}

TEST_CASE("netlist generation", "[netlist]") {
    auto root_dir = std::filesystem::path("tests/data/OA");
    auto lib_name = "test_netlist";
    auto lib_dir = root_dir / lib_name;
    auto yaml_dir = root_dir / "expect" / "yaml";
    auto expect_dir = root_dir / "expect" / "netlist";
    auto output_dir = root_dir / "output" / "netlist";

    auto flat = false;
    auto top_subckt = true;
    auto square_bracket = false;
    auto rmin = 2000;
    auto precision = 6;
    auto impl_cell = std::string("TEST");

    auto cell_name = GENERATE_COPY(read_directories(lib_dir));

    auto[format, sup_mode, shell] =
        GENERATE(values<std::tuple<cbag::design_output, cbag::supply_wrap, bool>>({
            {cbag::design_output::CDL, cbag::supply_wrap::NONE, false},
            {cbag::design_output::VERILOG, cbag::supply_wrap::NONE, true},
            {cbag::design_output::SYSVERILOG, cbag::supply_wrap::NONE, false},
            {cbag::design_output::SYSVERILOG, cbag::supply_wrap::TOP, false},
        }));

    auto sup_code = static_cast<int>(sup_mode);
    if (sup_mode != cbag::supply_wrap::NONE) {
        impl_cell += cbag::netlist::verilog_stream::sup_suffix;
    }

    auto fmt_str = "{}/{}.yaml";
    name_cv_vec name_cv_list;
    std::unordered_set<std::string> recorded;
    populate_name_cv_list(fmt_str, yaml_dir, cell_name, impl_cell, name_cv_list, recorded);

    auto ext_str = get_extension(format);
    auto basename = fmt::format("{}_{:d}_{:d}.{}", cell_name, shell, sup_code, ext_str);
    auto output_path = output_dir / basename;
    auto expect_path = expect_dir / basename;
    auto fname = output_path.string();

    CAPTURE(basename);

    std::vector<std::string> inc_list;
    std::string append_file;
    auto netlist_map = make_netlist_map();
    auto top_set = std::unordered_set<std::string>({impl_cell});
    cbag::netlist::write_netlist(name_cv_list, top_set, fname, format, netlist_map, append_file,
                                 inc_list, flat, shell, top_subckt, square_bracket, rmin, precision,
                                 sup_mode);
    auto output_str = read_file(fname);
    auto expect_str = read_file(expect_path.string());
    REQUIRE(output_str == expect_str);
}

TEST_CASE("netlist generation with netlist override", "[netlist]") {
    auto root_dir = std::filesystem::path("tests/data/OA");
    auto lib_name = "test_netlist";
    auto lib_dir = root_dir / lib_name;
    auto yaml_dir = root_dir / "expect" / "yaml";
    auto output_dir = root_dir / "output" / "netlist_override";

    auto format = cbag::design_output::SPECTRE;
    auto shell = false;
    auto flat = false;
    auto rmin = 2000;
    auto precision = 6;
    auto impl_cell = std::string("TEST");
    auto top_subckt = true;
    auto square_bracket = false;
    auto sup_mode = cbag::supply_wrap::NONE;
    auto netlist_sub = "foobar";
    auto netlist_expect = "simulator lang=spectre\n\n\nfoobar";

    auto fmt_str = "{}/{}.yaml";
    auto cell_name = GENERATE_COPY(read_directories(lib_dir));
    name_cv_vec name_cv_list;
    populate_name_cv_list_netlist(fmt_str, yaml_dir.string(), cell_name, impl_cell, name_cv_list,
                                  netlist_sub);

    auto ext_str = get_extension(format);
    auto basename = fmt::format("{}_{:d}.{}", cell_name, shell, ext_str);
    auto output_path = output_dir / basename;
    auto fname = output_path.string();

    CAPTURE(basename);

    std::vector<std::string> inc_list;
    std::string append_file;
    auto netlist_map = make_netlist_map();
    auto top_set = std::unordered_set<std::string>({cell_name});
    cbag::netlist::write_netlist(name_cv_list, top_set, fname, format, netlist_map, append_file,
                                 inc_list, flat, shell, top_subckt, square_bracket, rmin, precision,
                                 sup_mode);
    std::string output_str = read_file(fname);
    REQUIRE(trim(output_str) == netlist_expect);
}
