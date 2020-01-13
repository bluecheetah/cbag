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

#include <iterator>
#include <utility>
#include <vector>

#include <cbag/gdsii/read.h>
#include <cbag/oa/database.h>
#include <cbag/oa/main.h>
#include <cbag/oa/write_lib.h>

namespace cbagoa {

void gds_to_oa(const database &db, const std::string &gds_fname, const std::string &lib_name,
               const std::string &layer_map, const std::string &obj_map,
               const std::shared_ptr<const cbag::layout::routing_grid> &g,
               const std::shared_ptr<const cbag::layout::track_coloring> &colors) {
    auto cv_list = std::vector<std::shared_ptr<cbag::layout::cellview>>();
    cbag::gdsii::read_gds(gds_fname, layer_map, obj_map, g, colors, std::back_inserter(cv_list));

    auto cv_write_list =
        std::vector<std::pair<std::string, std::shared_ptr<cbag::layout::cellview>>>();
    cv_write_list.reserve(cv_list.size());
    for (const auto &cv_ptr : cv_list) {
        cv_write_list.emplace_back(cv_ptr->get_name(), cv_ptr);
    }

    implement_lay_list(db.ns_native, db.ns, *(db.logger), lib_name, "layout", cv_write_list);
}

} // namespace cbagoa
