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

#include <fmt/core.h>

#include "yaml-cpp/unordered_map.h"
#include <yaml-cpp/yaml.h>

#include <cbag/enum/design_output.h>
#include <cbag/netlist/netlist.h>
#include <cbag/util/io.h>
#include <cbag/util/string.h>
#include <cbag/yaml/cellviews.h>
#include <cbag/yaml/enum.h>

namespace cbag {
namespace netlist {

void read_prim_info(const std::string &prim_fname, std::vector<std::string> &inc_list,
                    sch::netlist_map_t &netlist_map, std::string &append_file,
                    design_output out_type) {
    if (prim_fname.empty())
        return;

    if (!util::is_file(prim_fname)) {
        throw std::invalid_argument(
            fmt::format("{} does not point to a netlist primitive file.", prim_fname));
    }

    YAML::Node n = YAML::LoadFile(prim_fname);
    netlist_map = n["netlist_map"].as<sch::netlist_map_t>();
    auto inc_list_map =
        n["inc_list"].as<std::unordered_map<design_output, std::vector<std::string>>>();
    auto iter = inc_list_map.find(out_type);
    if (iter == inc_list_map.end()) {
        throw std::out_of_range("Cannot find include files for netlist output code " +
                                std::to_string(static_cast<enum_t>(out_type)));
    }
    inc_list.reserve(iter->second.size());
    for (const auto &inc_file : iter->second) {
        inc_list.emplace_back(util::expand_env(inc_file));
    }

    auto app_file_map = n["prim_files"].as<std::unordered_map<design_output, std::string>>();
    auto iter2 = app_file_map.find(out_type);
    if (iter2 == app_file_map.end()) {
        throw std::out_of_range("Cannot find append primitive file for netlist output code " +
                                std::to_string(static_cast<enum_t>(out_type)));
    }
    append_file = iter2->second;
}

} // namespace netlist
} // namespace cbag
