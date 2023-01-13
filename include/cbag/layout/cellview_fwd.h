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

#ifndef CBAG_LAYOUT_CELLVIEW_FWD_H
#define CBAG_LAYOUT_CELLVIEW_FWD_H

#include <memory>
#include <vector>

#include <cbag/common/box_t.h>
#include <cbag/common/layer_t.h>
#include <cbag/common/transformation.h>
#include <cbag/layout/instance.h>
#include <cbag/layout/polygons.h>
#include <cbag/polygon/enum.h>
#include <cbag/polygon/geo_index.h>
#include <cbag/util/sorted_map.h>

namespace cbag {

struct box_array;
struct box_collection;

namespace layout {

class boundary;
class blockage;
class path;
class pin;
class via;
class via_wrapper;
class routing_grid;
class track_coloring;
class tech;
class label;
class track_id;

using geo_index_t = polygon::index::geo_index<coord_t>;

using geo_map_t = util::sorted_map<layer_t, poly_set_t>;
using path_map_t = util::sorted_map<layer_t, std::vector<path>>;
using block_map_t = util::sorted_map<lay_t, std::vector<blockage>>;
using pin_map_t = util::sorted_map<lay_t, std::vector<pin>>;
using inst_map_t = util::sorted_map<std::string, instance>;

class cellview {
  private:
    cnt_t inst_name_cnt = 0;
    std::shared_ptr<const routing_grid> grid_ = nullptr;
    std::shared_ptr<const track_coloring> tr_colors_ = nullptr;
    std::string cell_name;
    std::vector<std::shared_ptr<geo_index_t>> index_list;
    geo_map_t geo_map;
    path_map_t path_map;
    inst_map_t inst_map;
    pin_map_t pin_map;
    std::vector<via> via_list;
    block_map_t lay_block_map;
    std::vector<blockage> area_block_list;
    std::vector<boundary> boundary_list;
    std::vector<label> label_list;

    struct helper;

  public:
    cellview(std::shared_ptr<const routing_grid> grid,
             std::shared_ptr<const track_coloring> tr_colors, std::string cell_name);

    bool operator==(const cellview &rhs) const noexcept;

    auto find_geometry(layer_t key) const -> decltype(geo_map.find(key));

    const std::string &get_name() const noexcept;
    const tech *get_tech() const noexcept;
    const routing_grid *get_grid() const noexcept;
    const track_coloring *get_tr_colors() const noexcept;

    bool empty() const noexcept;

    const std::shared_ptr<geo_index_t> &get_geo_index(level_t lev) const;
    auto begin_inst() const -> decltype(inst_map.cbegin());
    auto end_inst() const -> decltype(inst_map.cend());
    auto begin_geometry() const -> decltype(geo_map.cbegin());
    auto end_geometry() const -> decltype(geo_map.cend());
    auto begin_path() const -> decltype(path_map.cbegin());
    auto end_path() const -> decltype(path_map.cend());
    auto begin_via() const -> decltype(via_list.cbegin());
    auto end_via() const -> decltype(via_list.cend());
    auto begin_lay_block() const -> decltype(lay_block_map.cbegin());
    auto end_lay_block() const -> decltype(lay_block_map.cend());
    auto begin_area_block() const -> decltype(area_block_list.cbegin());
    auto end_area_block() const -> decltype(area_block_list.cend());
    auto begin_boundary() const -> decltype(boundary_list.cbegin());
    auto end_boundary() const -> decltype(boundary_list.cend());
    auto begin_pin() const -> decltype(pin_map.cbegin());
    auto end_pin() const -> decltype(pin_map.cend());
    auto begin_label() const -> decltype(label_list.cbegin());
    auto end_label() const -> decltype(label_list.cend());

    void set_grid(const std::shared_ptr<const routing_grid> &grid);

    void add_pin(lay_t lay_id, std::string &&net, std::string &&label, box_t &&bbox);

    void add_label(layer_t &&key, transformation &&xform, std::string &&label, offset_t height);

    void add_object(const path &obj);
    void add_object(const blockage &obj);
    void add_object(const boundary &obj);
    void add_object(boundary &&obj);
    void add_object(const via_wrapper &obj);
    void add_object(const instance &obj);
    void add_shape(layer_t key, const box_t &obj);
    void add_shape(layer_t key, const box_array &obj);
    void add_shape(layer_t key, const box_collection &obj);
    void add_shape(layer_t key, const poly_90_t &obj);
    void add_shape(layer_t key, const poly_45_t &obj);
    void add_shape(layer_t key, const poly_t &obj);
    void add_shape(layer_t key, const poly_set_t &obj);
    void add_warr(const track_id &tid, coord_t lower, coord_t upper, bool is_dummy = false);
    void do_max_space_fill(level_t level, const box_t &bbox, bool fill_boundaries,
                           std::array<offset_t, 2> sp, std::array<offset_t, 2> margin,
                           double density);
};

} // namespace layout
} // namespace cbag

#endif
