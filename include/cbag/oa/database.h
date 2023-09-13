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

#ifndef CBAG_OA_DATABASE_H
#define CBAG_OA_DATABASE_H

#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include <oa/oaDesignDB.h>

#include <cbag/oa/typedef.h>

// forward declare structures to reduce dependencies
namespace spdlog {
class logger;
} // namespace spdlog

namespace cbag {
namespace sch {
struct instance;
} // namespace sch

} // namespace cbag

namespace cbagoa {

class LibDefObserver : public oa::oaObserver<oa::oaLibDefList> {
  public:
    std::string err_msg;

    explicit LibDefObserver(oa::oaUInt4 priority);

    oa::oaBoolean onLoadWarnings(oa::oaLibDefList *obj, const oa::oaString &msg,
                                 oa::oaLibDefListWarningTypeEnum type) override;
};

class database {
  private:
    // OA namespace objects
    std::string lib_def_file_;
    LibDefObserver lib_def_obs_{1};

  public:
    oa::oaNativeNS ns_native;
    oa::oaCdbaNS ns;
    str_map_t yaml_path_map;
    std::unordered_set<std::string> primitive_libs;
    std::shared_ptr<spdlog::logger> logger;

  public:
    explicit database(std::string lib_def_fname);
    ~database();

    void add_yaml_path(const std::string &lib_name, std::string yaml_path);

    void add_primitive_lib(std::string lib_name);

    bool is_primitive_lib(const std::string &lib_name) const;

    std::vector<std::string> get_cells_in_lib(const std::string &lib_name) const;

    std::string get_lib_path(const std::string &lib_name) const;

    void create_lib(const std::string &lib_name, const std::string &lib_path,
                    const std::string &tech_lib) const;

    cbag::sch::cellview read_sch_cellview(const std::string &lib_name, const std::string &cell_name,
                                          const std::string &view_name) const;

    std::vector<cell_key_t> read_sch_recursive(const std::string &lib_name,
                                               const std::string &cell_name,
                                               const std::string &view_name) const;

    std::vector<cell_key_t> read_library(const std::string &lib_name,
                                         const std::string &view_name) const;

    void write_sch_cellview(const std::string &lib_name, const std::string &cell_name,
                            const std::string &view_name, bool is_sch,
                            const cbag::sch::cellview &cv,
                            const str_map_t *rename_map = nullptr) const;

    void write_lay_cellview(const std::string &lib_name, const std::string &cell_name,
                            const std::string &view_name, const cbag::layout::cellview &cv,
                            oa::oaTech *tech, const str_map_t *rename_map = nullptr) const;

    void implement_sch_list(const std::string &lib_name, const std::string &sch_view,
                            const std::string &sym_view,
                            const std::vector<sch_cv_info> &cv_list) const;

    void implement_lay_list(const std::string &lib_name, const std::string &view,
                            const std::vector<lay_cv_info> &cv_list) const;

    void write_tech_info_file(const std::string &fname, const std::string &tech_lib,
                              const std::string &pin_purpose, const std::string &label_purpose) const;
};

} // namespace cbagoa

#endif // CBAGOA_DATABASE_H
