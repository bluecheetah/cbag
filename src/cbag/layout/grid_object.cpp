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

#include <boost/container_hash/hash.hpp>

#include <fmt/core.h>

#include <cbag/common/transformation_util.h>
#include <cbag/layout/grid_object.h>
#include <cbag/layout/routing_grid_util.h>
#include <cbag/layout/track_info_util.h>
#include <cbag/util/math.h>

namespace cbag {
namespace layout {

track_id::track_id() = default;

track_id::track_id(level_t level, htr_t htr, cnt_t ntr, cnt_t num, htr_t pitch)
    : level(level), htr(htr), ntr(ntr), num(num), pitch((num == 1) ? 0 : pitch) {}

bool track_id::operator==(const track_id &rhs) const noexcept {
    return level == rhs.level && htr == rhs.htr && ntr == rhs.ntr && num == rhs.num &&
           pitch == rhs.pitch;
}

htr_t track_id::operator[](std::size_t idx) const noexcept { return htr + idx * pitch; }

std::size_t track_id::get_hash() const noexcept {
    auto seed = static_cast<std::size_t>(0);
    boost::hash_combine(seed, level);
    boost::hash_combine(seed, htr);
    boost::hash_combine(seed, ntr);
    boost::hash_combine(seed, num);
    boost::hash_combine(seed, pitch);
    return seed;
}

std::string track_id::to_string() const noexcept {
    return fmt::format("TrackID(layer={}, htr={}, wdth={}, num={}, htr_pitch={})", level, htr, ntr,
                       num, pitch);
}

level_t track_id::get_level() const noexcept { return level; }

htr_t track_id::get_htr() const noexcept { return htr; }

cnt_t track_id::get_ntr() const noexcept { return ntr; }

cnt_t track_id::get_num() const noexcept { return num; }

htr_t track_id::get_pitch() const noexcept { return pitch; }

std::array<offset_t, 2> track_id::get_bounds(const routing_grid &grid) const {
    auto ans = get_wire_bounds(grid, level, htr, ntr);
    auto delta = static_cast<offset_t>(get_num() - 1) * get_pitch() * grid[level].get_pitch() / 2;
    ans[delta > 0] += delta;
    return ans;
}

void track_id::set_htr(htr_t val) noexcept { htr = val; }

void track_id::set_pitch(htr_t val) noexcept { pitch = (num == 1) ? 0 : val; }

wire_array::wire_array() = default;

wire_array::wire_array(std::shared_ptr<track_id> tid, offset_t lower, offset_t upper)
    : tid_ptr(std::move(tid)), coord({lower, upper}) {}

bool wire_array::operator==(const wire_array &rhs) const noexcept {
    auto is_null = tid_ptr == nullptr;
    auto rhs_is_null = rhs.tid_ptr == nullptr;
    return (is_null && rhs_is_null) ||
           (!is_null && !rhs_is_null && *tid_ptr == *(rhs.tid_ptr) && coord == rhs.coord);
}

std::size_t wire_array::get_hash() const {
    auto seed = static_cast<std::size_t>(0);
    boost::hash_combine(seed, tid_ptr->get_hash());
    boost::hash_combine(seed, coord[0]);
    boost::hash_combine(seed, coord[1]);
    return seed;
}

std::string wire_array::to_string() const {
    return fmt::format("WireArray({}, {}, {})", tid_ptr->to_string(), coord[0], coord[1]);
}

const std::array<offset_t, 2> &wire_array::get_coord() const noexcept { return coord; }

offset_t wire_array::get_coord(direction_1d dir) const noexcept { return coord[to_int(dir)]; }

std::shared_ptr<track_id> wire_array::get_track_id() const noexcept { return tid_ptr; }

const track_id &wire_array::get_track_id_ref() const { return *tid_ptr; }

offset_t wire_array::get_middle() const noexcept { return util::floor2(coord[0] + coord[1]); }

void wire_array::set_track_id(std::shared_ptr<track_id> tid) noexcept { tid_ptr = std::move(tid); }

void wire_array::set_coord(offset_t lower, offset_t upper) noexcept {
    coord[0] = lower;
    coord[1] = upper;
}

warr_rect_iterator wire_array::begin_rect(const routing_grid &grid) const {
    return cbag::layout::begin_rect(grid, *tid_ptr, coord);
}

warr_rect_iterator wire_array::end_rect(const routing_grid &grid) const {
    return cbag::layout::end_rect(grid, *tid_ptr, coord);
}

warr_rect_iterator::warr_rect_iterator() = default;

warr_rect_iterator::warr_rect_iterator(const routing_grid &grid, const track_id &tid,
                                       std::array<offset_t, 2> coord, bool begin)
    : grid_ptr(&grid), tr_ptr(&tid), coord(coord), tr_idx(begin ? 0 : tid.get_num()) {
    auto &tinfo = grid.track_info_at(tid.get_level());
    wire_w = tinfo.get_wire_width(tid.get_ntr());
    ww_idx = 0;
}

bool warr_rect_iterator::operator==(const warr_rect_iterator &rhs) const {
    return tr_idx == rhs.tr_idx && ww_idx == rhs.ww_idx;
}

bool warr_rect_iterator::operator!=(const warr_rect_iterator &rhs) const { return !(*this == rhs); }

std::tuple<layer_t, box_t> warr_rect_iterator::operator*() const {
    auto level = tr_ptr->get_level();

    auto &tinfo = (*grid_ptr)[level];
    auto htr = (*tr_ptr)[tr_idx];
    auto &[rel_htr, w] = wire_w[ww_idx];
    auto c = htr_to_coord(tinfo, rel_htr + htr);
    auto half_w = w / 2;
    return {get_layer_t(*grid_ptr, level, htr + rel_htr),
            box_t(tinfo.get_direction(), coord[0], coord[1], c - half_w, c + half_w)};
}

warr_rect_iterator &warr_rect_iterator::operator++() {
    ++ww_idx;
    if (ww_idx == wire_w.size()) {
        ww_idx = 0;
        ++tr_idx;
    }
    return *this;
}

const routing_grid &warr_rect_iterator::get_grid() const { return *grid_ptr; }

warr_rect_iterator begin_rect(const routing_grid &grid, const track_id &tid,
                              std::array<offset_t, 2> coord) {
    return {grid, tid, coord, true};
}

warr_rect_iterator end_rect(const routing_grid &grid, const track_id &tid,
                            std::array<offset_t, 2> coord) {
    return {grid, tid, coord, false};
}

} // namespace layout
} // namespace cbag
