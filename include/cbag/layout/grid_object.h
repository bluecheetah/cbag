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

#ifndef CBAG_LAYOUT_GRID_OBJECT_H
#define CBAG_LAYOUT_GRID_OBJECT_H

#include <array>
#include <memory>
#include <string>
#include <tuple>

#include <cbag/common/box_t.h>
#include <cbag/common/layer_t.h>
#include <cbag/common/typedefs.h>
#include <cbag/layout/wire_width.h>
#include <cbag/polygon/enum.h>

namespace cbag {
namespace layout {

class routing_grid;
class warr_rect_iterator;

class track_id : public std::enable_shared_from_this<track_id> {
  private:
    level_t level = 0;
    htr_t htr = 0;
    cnt_t ntr = 1;
    cnt_t num = 1;
    htr_t pitch = 0;

  public:
    track_id();

    track_id(level_t level, htr_t htr, cnt_t ntr, cnt_t num, htr_t pitch);

    bool operator==(const track_id &rhs) const noexcept;

    htr_t operator[](std::size_t idx) const noexcept;

    std::size_t get_hash() const noexcept;

    std::string to_string() const noexcept;

    level_t get_level() const noexcept;

    htr_t get_htr() const noexcept;

    cnt_t get_ntr() const noexcept;

    cnt_t get_num() const noexcept;

    htr_t get_pitch() const noexcept;

    std::array<offset_t, 2> get_bounds(const routing_grid &grid) const;

    void set_htr(htr_t val) noexcept;

    void set_pitch(htr_t val) noexcept;
};

class wire_array {
  private:
    std::shared_ptr<track_id> tid_ptr;
    std::array<offset_t, 2> coord;

  public:
    wire_array();

    wire_array(std::shared_ptr<track_id> tid, offset_t lower, offset_t upper);

    bool operator==(const wire_array &rhs) const noexcept;

    std::size_t get_hash() const;

    std::string to_string() const;

    const std::array<offset_t, 2> &get_coord() const noexcept;

    offset_t get_coord(direction_1d dir) const noexcept;

    std::shared_ptr<track_id> get_track_id() const noexcept;

    const track_id &get_track_id_ref() const;

    offset_t get_middle() const noexcept;

    void set_track_id(std::shared_ptr<track_id> tid) noexcept;

    void set_coord(offset_t lower, offset_t upper) noexcept;

    warr_rect_iterator begin_rect(const routing_grid &grid) const;

    warr_rect_iterator end_rect(const routing_grid &grid) const;
};

class warr_rect_iterator {
  private:
    const routing_grid *grid_ptr;
    const track_id *tr_ptr;
    std::array<offset_t, 2> coord;
    cnt_t tr_idx = 0;
    cnt_t ww_idx = 0;
    wire_width wire_w;

  public:
    warr_rect_iterator();

    warr_rect_iterator(const routing_grid &grid, const track_id &tid, std::array<offset_t, 2> coord,
                       bool begin);

    bool operator==(const warr_rect_iterator &rhs) const;
    bool operator!=(const warr_rect_iterator &rhs) const;

    std::tuple<layer_t, box_t> operator*() const;
    warr_rect_iterator &operator++();

    const routing_grid &get_grid() const;
};

warr_rect_iterator begin_rect(const routing_grid &grid, const track_id &tid,
                              std::array<offset_t, 2> coord);

warr_rect_iterator end_rect(const routing_grid &grid, const track_id &tid,
                            std::array<offset_t, 2> coord);

} // namespace layout
} // namespace cbag

#endif
