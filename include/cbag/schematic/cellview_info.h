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

#ifndef CBAG_SCHEMATIC_CELLVIEW_INFO_H
#define CBAG_SCHEMATIC_CELLVIEW_INFO_H

#include <array>
#include <string>
#include <vector>

#include <cbag/common/param_map.h>
#include <cbag/util/sorted_map.h>

namespace cbag {
namespace sch {

struct cellview;

using attr_map_t = util::sorted_map<std::string, std::string>;

/** A simple struct representing netlist information of a cellview.
 */
struct cellview_info {
  public:
    std::string lib_name;
    std::string cell_name;
    std::vector<std::string> in_terms;
    std::vector<std::string> out_terms;
    std::vector<std::string> io_terms;
    std::vector<std::string> nets;
    param_map props;
    bool is_prim = false;
    std::string va = "";
    bool ignore = false;
    util::sorted_map<std::string, attr_map_t> term_net_attrs;
    const cellview *cv_ptr = nullptr;

    cellview_info();

    explicit cellview_info(const std::string &file_name);

    cellview_info(std::string lib_name, std::string cell_name, bool is_prim);

    void to_file(const std::string &file_name) const;
};

using lib_map_t = std::unordered_map<std::string, sch::cellview_info>;
using netlist_map_t = std::unordered_map<std::string, lib_map_t>;

const cellview_info &get_cv_info(const netlist_map_t &info_map, const std::string &lib_name,
                                 const std::string &cell_name);

void record_cv_info(netlist_map_t &info_map, std::string &&cell_name, cellview_info &&info);

/*
  cell_name: the final cell name of the given cellview (after renaming).
  info_map: map from library/cell name (before renaming) to cellview_info objects.
 */
cellview_info get_cv_netlist_info(const cellview &cv, const std::string &cell_name,
                                  const netlist_map_t &info_map, bool compute_net_attrs);

} // namespace sch
} // namespace cbag

#endif
