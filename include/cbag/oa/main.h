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

#ifndef CBAG_OA_MAIN_H
#define CBAG_OA_MAIN_H

#include <memory>
#include <string>

namespace cbag {
namespace layout {

class routing_grid;
class track_coloring;

} // namespace layout
} // namespace cbag

namespace cbagoa {

class database;

void gds_to_oa(const database &db, const std::string &gds_fname, const std::string &lib_name,
               const std::string &layer_map, const std::string &obj_map,
               const std::shared_ptr<const cbag::layout::routing_grid> &g,
               const std::shared_ptr<const cbag::layout::track_coloring> &colors);

} // namespace cbagoa

#endif
