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

#include <cbag/schematic/cellview.h>

#include <cbag/oa/read.h>
#include <cbag/oa/read_lib.h>
#include <cbag/util/io.h>

namespace cbagoa {

cbag::sch::cellview cell_to_yaml(const oa::oaNativeNS &ns_native, const oa::oaCdbaNS &ns,
                                 spdlog::logger &logger, const std::string &lib_name,
                                 const std::string &cell_name, const std::string &sch_view,
                                 const std::string &yaml_path,
                                 const std::unordered_set<std::string> &primitive_libs) {
    // parse schematic
    cbag::sch::cellview sch_cv =
        read_sch_cellview(ns_native, ns, logger, lib_name, cell_name, sch_view, primitive_libs);

    // write schematic to file
    auto yaml_dir = cbag::util::get_canonical_path(yaml_path);
    auto yaml_fpath = yaml_dir / (cell_name + ".yaml");
    cbag::util::make_parent_dirs(yaml_fpath);
    sch_cv.to_file(yaml_fpath.string());

    // write all symbol views to file
    // get library read access
    oa::oaLib *lib_ptr = open_library_read(ns_native, lib_name);
    // find all symbol views
    oa::oaScalarName cell_name_oa(ns_native, cell_name.c_str());
    oa::oaCell *cell_ptr = oa::oaCell::find(lib_ptr, cell_name_oa);
    oa::oaIter<oa::oaCellView> cv_iter(cell_ptr->getCellViews());
    oa::oaCellView *cv_ptr;
    while ((cv_ptr = cv_iter.getNext()) != nullptr) {
        oa::oaString tmp_name;
        oa::oaView *view_ptr = cv_ptr->getView();
        if (view_ptr->getViewType() == oa::oaViewType::get(oa::oacSchematicSymbol)) {
            view_ptr->getName(ns_native, tmp_name);
            yaml_fpath = yaml_dir / fmt::format("{}.{}.yaml", cell_name, (const char *)tmp_name);
            read_sch_cellview(ns_native, ns, logger, lib_name, cell_name,
                              std::string((const char *)tmp_name), primitive_libs)
                .to_file(yaml_fpath.string());
        }
    }
    // release read access
    lib_ptr->releaseAccess();

    return sch_cv;
}

} // namespace cbagoa
