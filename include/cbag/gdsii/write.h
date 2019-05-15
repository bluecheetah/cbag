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

#ifndef CBAG_GDSII_WRITE_H
#define CBAG_GDSII_WRITE_H

#include <optional>
#include <string>
#include <unordered_map>

#include <boost/container_hash/hash.hpp>

#include <cbag/logging/logging.h>

#include <cbag/util/io.h>

#include <cbag/common/layer_t.h>
#include <cbag/enum/boundary_type.h>
#include <cbag/gdsii/typedefs.h>
#include <cbag/layout/cellview_fwd.h>
#include <cbag/layout/tech.h>

namespace cbag {
namespace gdsii {

using layer_map = std::unordered_map<layer_t, gds_layer_t, boost::hash<layer_t>>;
using boundary_map = std::unordered_map<boundary_type, gds_layer_t>;

class gds_lookup {
  private:
    layer_map lay_map;
    boundary_map bnd_map;

  public:
    gds_lookup(const layout::tech &tech, const std::string &lay_map_file,
               const std::string &obj_map_file);

    std::optional<gds_layer_t> get_gds_layer(layer_t key) const;

    std::optional<gds_layer_t> get_gds_layer(boundary_type bnd_type) const;
};

std::vector<tval_t> get_gds_time();

void write_gds_start(spdlog::logger &logger, std::ostream &stream, const std::string &lib_name,
                     double resolution, double user_unit, const std::vector<tval_t> &time_vec);

void write_gds_stop(spdlog::logger &logger, std::ostream &stream);

void write_lay_cellview(spdlog::logger &logger, std::ostream &stream, const std::string &cell_name,
                        const cbag::layout::cellview &cv,
                        const std::unordered_map<std::string, std::string> &rename_map,
                        const std::vector<tval_t> &time_vec, const gds_lookup &lookup);

template <class Vector>
void implement_gds(const std::string &fname, const std::string &lib_name,
                   const std::string &layer_map, const std::string &obj_map, double resolution,
                   double user_unit, const Vector &cv_list) {
    auto logger = get_cbag_logger();
    auto time_vec = get_gds_time();

    // get gds file stream
    auto stream = util::open_file_write(fname, true);
    write_gds_start(*logger, stream, lib_name, resolution, user_unit, time_vec);

    // get first element and setup gds_lookup
    auto cursor = cv_list.begin();
    auto stop = cv_list.end();
    if (cursor != stop) {
        std::unordered_map<std::string, std::string> rename_map{};
        gds_lookup lookup{*cursor->second->get_tech(), layer_map, obj_map};
        for (; cursor != stop; ++cursor) {
            auto &[cv_cell_name, cv_ptr] = *cursor;
            const auto &cell_name = cv_ptr->get_name();
            logger->info("Creating layout cell {}", cv_cell_name);
            write_lay_cellview(*logger, stream, cv_cell_name, *cv_ptr, rename_map, time_vec,
                               lookup);
            logger->info("cell name {} maps to {}", cell_name, cv_cell_name);
            rename_map[cell_name] = cv_cell_name;
        }
    }

    write_gds_stop(*logger, stream);
    stream.close();
}

} // namespace gdsii
} // namespace cbag

#endif
