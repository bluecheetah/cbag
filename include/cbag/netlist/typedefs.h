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

#ifndef CBAG_NETLIST_TYPEDEFS_H
#define CBAG_NETLIST_TYPEDEFS_H

#include <unordered_map>
#include <utility>
#include <vector>

#include <cbag/spirit/ast.h>

namespace cbag {
namespace netlist {

using term_net_vec_t =
    std::vector<std::pair<spirit::ast::name_unit, std::vector<spirit::ast::name>>>;
using net_rename_map_t = std::unordered_map<spirit::ast::name_bit, spirit::ast::name_bit>;

} // namespace netlist
} // namespace cbag

#endif
