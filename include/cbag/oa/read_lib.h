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

#ifndef CBAG_OA_READ_LIB_H
#define CBAG_OA_READ_LIB_H

#include <unordered_set>

#include <fmt/core.h>

#include <oa/oaDesignDB.h>

#include <cbag/schematic/cellview_fwd.h>
#include <cbag/schematic/instance.h>

#include <cbag/oa/typedef.h>
#include <cbag/oa/util.h>

namespace cbagoa {

cbag::sch::cellview cell_to_yaml(const oa::oaNativeNS &ns_native, const oa::oaCdbaNS &ns,
                                 spdlog::logger &logger, const std::string &lib_name,
                                 const std::string &cell_name, const std::string &sch_view,
                                 const std::string &yaml_path,
                                 const std::unordered_set<std::string> &primitive_libs);

template <class OutIter>
void get_cells(const oa::oaNativeNS &ns_native, spdlog::logger &logger, const std::string &lib_name,
               OutIter out_iter) {
    try {
        oa::oaLib *lib_ptr = open_library_read(ns_native, lib_name);
        oa::oaIter<oa::oaCell> cell_iter(lib_ptr->getCells());
        oa::oaCell *cell_ptr;
        oa::oaString tmp_str;
        while ((cell_ptr = cell_iter.getNext()) != nullptr) {
            cell_ptr->getName(ns_native, tmp_str);
            out_iter = std::string(tmp_str);
        }
        lib_ptr->releaseAccess();
    } catch (...) {
        handle_oa_exceptions(logger);
    }
}

template <class OutIter>
void read_sch_helper(const oa::oaNativeNS &ns_native, const oa::oaCdbaNS &ns,
                     spdlog::logger &logger, std::pair<std::string, std::string> &key,
                     const std::string &view_name, const str_map_t &yaml_path_map,
                     const std::unordered_set<std::string> &primitive_libs,
                     cell_set_t &exclude_cells, OutIter &out_iter) {
    // find yaml_path
    auto map_iter = yaml_path_map.find(key.first);
    if (map_iter == yaml_path_map.end())
        throw std::runtime_error(fmt::format(
            "Cannot find yaml path for library {}, is the library registered?", key.first));
    const std::string &yaml_path = map_iter->second;

    // write cellviews to yaml files
    cbag::sch::cellview sch_cv = cell_to_yaml(ns_native, ns, logger, key.first, key.second,
                                              view_name, yaml_path, primitive_libs);

    // update cell_list and exclude_cells
    out_iter = key;
    exclude_cells.insert(std::move(key));

    // recurse
    auto exc_lib_end = primitive_libs.end();
    for (const auto &pair : sch_cv.instances) {
        cbag::sch::instance *inst = pair.second.get();
        std::pair<std::string, std::string> ikey(inst->lib_name, inst->cell_name);
        if (exclude_cells.find(ikey) == exclude_cells.end()) {
            // did not see this schematic master before
            if (primitive_libs.find(inst->lib_name) == exc_lib_end) {
                // non-primitive master, parse normally
                read_sch_helper(ns_native, ns, logger, ikey, view_name, yaml_path_map,
                                primitive_libs, exclude_cells, out_iter);
            }
        }
    }
}

template <class OutIter>
void read_sch_recursive(const oa::oaNativeNS &ns_native, const oa::oaCdbaNS &ns,
                        spdlog::logger &logger, const std::string &lib_name,
                        const std::string &cell_name, const std::string &view_name,
                        const str_map_t &yaml_path_map,
                        const std::unordered_set<std::string> &primitive_libs, OutIter out_iter) {
    auto key = std::make_pair(lib_name, cell_name);
    cell_set_t exclude_cells;
    read_sch_helper(ns_native, ns, logger, key, view_name, yaml_path_map, primitive_libs,
                    exclude_cells, out_iter);
}

template <class OutIter> struct read_library_processor {
    const oa::oaNativeNS &ns_native;
    const oa::oaCdbaNS &ns;
    spdlog::logger &logger;
    const std::string &lib_name;
    const std::string &view_name;
    const str_map_t &yaml_path_map;
    const std::unordered_set<std::string> &primitive_libs;
    cell_set_t exclude_cells;
    OutIter &out_iter;

    read_library_processor(const oa::oaNativeNS &ns_native, const oa::oaCdbaNS &ns,
                           spdlog::logger &logger, const std::string &lib_name,
                           const std::string &view_name, const str_map_t &yaml_path_map,
                           const std::unordered_set<std::string> &primitive_libs, OutIter &out_iter)
        : ns_native(ns_native), ns(ns), logger(logger), lib_name(lib_name), view_name(view_name),
          yaml_path_map(yaml_path_map), primitive_libs(primitive_libs), out_iter(out_iter) {}

    read_library_processor &operator=(std::string &&cell_name) {
        std::pair<std::string, std::string> key(lib_name, cell_name);
        read_sch_helper<OutIter>(ns_native, ns, logger, key, view_name, yaml_path_map,
                                 primitive_libs, exclude_cells, out_iter);
        return *this;
    }
};

template <class OutIter>
void read_library(const oa::oaNativeNS &ns_native, const oa::oaCdbaNS &ns, spdlog::logger &logger,
                  const std::string &lib_name, const std::string &view_name,
                  const str_map_t &yaml_path_map,
                  const std::unordered_set<std::string> &primitive_libs, OutIter out_iter) {
    read_library_processor proc(ns_native, ns, logger, lib_name, view_name, yaml_path_map,
                                primitive_libs, out_iter);
    get_cells(ns_native, logger, lib_name, proc);
}

} // namespace cbagoa

#endif
