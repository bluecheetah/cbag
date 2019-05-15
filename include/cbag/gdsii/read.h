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

#ifndef CBAG_GDSII_READ_H
#define CBAG_GDSII_READ_H

#include <memory>
#include <string>
#include <tuple>
#include <unordered_map>
#include <variant>

#include <boost/container_hash/hash.hpp>

#include <cbag/logging/logging.h>

#include <cbag/util/io.h>

#include <cbag/common/layer_t.h>
#include <cbag/enum/boundary_type.h>
#include <cbag/gdsii/record_type.h>
#include <cbag/gdsii/typedefs.h>
#include <cbag/layout/cellview.h>
#include <cbag/layout/routing_grid_fwd.h>
#include <cbag/layout/tech.h>

namespace cbag {
namespace gdsii {

using gds_to_lay_map = std::unordered_map<gds_layer_t, layer_t, boost::hash<gds_layer_t>>;
using gds_to_bnd_map = std::unordered_map<gds_layer_t, boundary_type, boost::hash<gds_layer_t>>;

class gds_rlookup {
  private:
    gds_to_lay_map lay_map;
    gds_to_bnd_map bnd_map;

  public:
    gds_rlookup();

    gds_rlookup(const std::string &lay_fname, const std::string &obj_fname, const layout::tech &t);

    std::variant<layer_t, boundary_type, bool> get_mapping(gds_layer_t key) const;

    layer_t get_layer_t(gds_layer_t key) const;
};

std::tuple<record_type, std::size_t> read_record_header(std::istream &stream);

std::string read_gds_start(spdlog::logger &logger, std::istream &stream);

std::tuple<std::string, std::shared_ptr<layout::cellview>> read_lay_cellview(
    spdlog::logger &logger, std::istream &stream, const std::string &lib_name,
    const std::shared_ptr<const layout::routing_grid> &g, const gds_rlookup &rmap,
    const std::unordered_map<std::string, std::shared_ptr<const layout::cellview>> &master_map);

template <class OutIter>
void read_gds(const std::string &fname, const std::string &layer_map, const std::string &obj_map,
              const std::shared_ptr<const layout::routing_grid> &g, OutIter &&out_iter) {
    auto log_ptr = get_cbag_logger();

    // get gds file stream
    log_ptr->info("Reading GDS file {}", fname);
    auto stream = util::open_file_read(fname, true);
    auto lib_name = read_gds_start(*log_ptr, stream);
    log_ptr->info("GDS library: {}", lib_name);

    bool is_done = false;
    gds_rlookup rmap(layer_map, obj_map, *(g->get_tech()));
    std::unordered_map<std::string, std::shared_ptr<const layout::cellview>> cv_map;
    while (!is_done) {
        auto [rtype, rsize] = read_record_header(stream);
        switch (rtype) {
        case record_type::BGNSTR: {
            log_ptr->info("Reading GDS cellview");
            stream.ignore(rsize);
            auto [cell_name, cv_ptr] =
                read_lay_cellview(*log_ptr, stream, lib_name, g, rmap, cv_map);

            cv_map.emplace(cell_name, cv_ptr);
            *out_iter = std::move(cv_ptr);
            ++out_iter;
            break;
        }
        case record_type::ENDLIB:
            log_ptr->info("Finish reading GDS file {}", fname);
            stream.close();
            return;
        default:
            throw std::runtime_error("Unrecognized GDS record type: " +
                                     std::to_string(static_cast<int>(rtype)));
        }
    }
}

} // namespace gdsii
} // namespace cbag

#endif
