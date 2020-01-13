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

#ifndef CBAG_OA_WRITE_LIB_H
#define CBAG_OA_WRITE_LIB_H

#include <cbag/logging/spdlog.h>

#include <cbag/schematic/cellview.h>

#include <cbag/oa/typedef.h>
#include <cbag/oa/util.h>
#include <cbag/oa/write.h>

namespace cbagoa {

template <class Vector>
void implement_sch_list(const oa::oaNativeNS &ns_native, const oa::oaCdbaNS &ns,
                        spdlog::logger &logger, const std::string &lib_name,
                        const std::string &sch_view, const std::string &sym_view,
                        const Vector &cv_list) {
    try {
        str_map_t rename_map;

        for (const auto &cv_info : cv_list) {
            const auto &name = cv_info.first;
            const auto cv_ptr = cv_info.second.first;
            write_sch_cellview(ns_native, ns, logger, lib_name, name, sch_view, true, *cv_ptr,
                               &rename_map);
            if (cv_ptr->sym_ptr != nullptr && !sym_view.empty()) {
                write_sch_cellview(ns_native, ns, logger, lib_name, name, sym_view, false,
                                   *(cv_ptr->sym_ptr), &rename_map);
            }
            logger.info("cell name {} maps to {}", cv_ptr->cell_name, name);
            rename_map[cv_ptr->cell_name] = name;
        }
    } catch (...) {
        handle_oa_exceptions(logger);
    }
}

template <class Vector>
void implement_lay_list(const oa::oaNativeNS &ns_native, const oa::oaCdbaNS &ns,
                        spdlog::logger &logger, const std::string &lib_name,
                        const std::string &view, const Vector &cv_list) {
    try {
        str_map_t rename_map;
        auto *tech_ptr = read_tech(ns_native, lib_name);

        for (const auto &cv_info : cv_list) {
            const auto cv_ptr = cv_info.second;
            write_lay_cellview(ns_native, ns, logger, lib_name, cv_info.first, view, *cv_ptr,
                               tech_ptr, &rename_map);
            const auto &cell_name = cv_ptr->get_name();
            logger.info("cell name {} maps to {}", cell_name, cv_info.first);
            rename_map[std::move(cell_name)] = cv_info.first;
        }
    } catch (...) {
        handle_oa_exceptions(logger);
    }
}

} // namespace cbagoa

#endif
