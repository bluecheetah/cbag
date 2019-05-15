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

#ifndef CBAG_SCHEMATIC_CELLVIEW_INST_MOD_H
#define CBAG_SCHEMATIC_CELLVIEW_INST_MOD_H

#include <fmt/core.h>

#include <cbag/spirit/name.h>
#include <cbag/spirit/name_rep.h>
#include <cbag/spirit/parsers.h>

#include <cbag/common/box_t.h>
#include <cbag/common/transformation_util.h>
#include <cbag/schematic/cellview_fwd.h>
#include <cbag/schematic/instance.h>

namespace cbag {
namespace sch {

template <class ConnRange>
void copy_instance(inst_map_t &instances, const instance &inst, cnt_t old_size,
                   const std::string &new_name, coord_t dx, coord_t dy, const ConnRange &conns) {
    // check the new name is legal.  Parse will throw exception if not passed
    spirit::ast::name_rep new_ast;
    parse(new_name, spirit::name_rep(), new_ast);

    // insert with empty pointer
    auto emp_iter = instances.emplace(new_name, std::unique_ptr<instance>());
    if (!emp_iter.second) {
        throw std::invalid_argument(
            fmt::format("instance {} already exists.", emp_iter.first->first));
    }
    // insert successful, make pointer point at something
    emp_iter.first->second = std::make_unique<instance>(inst);
    auto cur_ptr = emp_iter.first->second.get();

    // resize nets
    auto new_size = new_ast.size();
    if (old_size != new_size) {
        cur_ptr->resize_nets(old_size, new_size);
    }

    // shift and update connections
    cur_ptr->xform.move_by(dx, dy);
    for (auto const &p : conns) {
        cur_ptr->update_connection(new_name, new_ast.size(), p.first, p.second);
    }
}

template <class NameConnRange>
void array_instance(inst_map_t &instances, const std::string &old_name, coord_t dx, coord_t dy,
                    const NameConnRange &name_conn_range) {
    // find the instance to copy
    auto iter = instances.find(old_name);
    if (iter == instances.end()) {
        throw std::invalid_argument("Cannot find instance: " + old_name);
    }
    auto inst_ptr = iter->second.get();
    // get old instance name and size
    spirit::ast::name_rep old_ast;
    parse(old_name, spirit::name_rep(), old_ast);
    auto old_size = old_ast.size();

    if (dx == 0 && dy == 0) {
        // figure out default shift
        dx = width(inst_ptr->bbox) + 10;
    }

    coord_t x = 0;
    coord_t y = 0;
    for (const auto &p : name_conn_range) {
        copy_instance(instances, *inst_ptr, old_size, p.first, x, y, p.second);
        x += dx;
        y += dy;
    }

    // remove original instance
    instances.erase(old_name);
}

} // namespace sch
} // namespace cbag

#endif // CBAG_SCHEMATIC_CELLVIEWS_H
