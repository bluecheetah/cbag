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
#include <cbag/layout/tech_util.h>
#include <cbag/layout/track_info_util.h>
#include <cbag/util/math.h>

namespace cbag {
namespace layout {

track_id::track_id() = default;

track_id::track_id(level_t level, htr_t htr, cnt_t ntr, cnt_t num, htr_t pitch)
    : level_(level), htr_(htr), ntr_(ntr), num_(num), pitch_((num == 1) ? 0 : pitch) {}

bool track_id::operator==(const track_id &rhs) const noexcept {
    return level_ == rhs.level_ && htr_ == rhs.htr_ && ntr_ == rhs.ntr_ && num_ == rhs.num_ &&
           pitch_ == rhs.pitch_;
}

htr_t track_id::operator[](std::size_t idx) const noexcept { return htr_ + idx * pitch_; }

std::size_t track_id::get_hash() const noexcept {
    auto seed = static_cast<std::size_t>(level_);
    boost::hash_combine(seed, htr_);
    boost::hash_combine(seed, ntr_);
    boost::hash_combine(seed, num_);
    boost::hash_combine(seed, pitch_);
    return seed;
}

std::string track_id::to_string() const noexcept {
    return fmt::format("TrackID(layer={}, htr={}, width={}, num={}, htr_pitch={})", level_, htr_,
                       ntr_, num_, pitch_);
}

level_t track_id::get_level() const noexcept { return level_; }

htr_t track_id::get_htr() const noexcept { return htr_; }

cnt_t track_id::get_ntr() const noexcept { return ntr_; }

cnt_t track_id::get_num() const noexcept { return num_; }

htr_t track_id::get_pitch() const noexcept { return pitch_; }

std::array<offset_t, 2> track_id::get_bounds(const routing_grid &grid) const {
    auto ans = get_wire_bounds(grid, level_, htr_, ntr_);
    auto delta = static_cast<offset_t>(num_ - 1) * pitch_ * grid[level_].get_pitch() / 2;
    ans[delta > 0] += delta;
    return ans;
}

void track_id::set_htr(htr_t val) noexcept { htr_ = val; }

void track_id::set_pitch(htr_t val) noexcept { pitch_ = (num_ == 1) ? 0 : val; }

warr_rect_iterator::warr_rect_iterator() = default;

warr_rect_iterator::warr_rect_iterator(const track_id &tid) : tr_idx_(tid.get_num()){};

warr_rect_iterator::warr_rect_iterator(const routing_grid &grid, const track_coloring &tr_colors,
                                       const track_id &tid, coord_t lower, coord_t upper,
                                       bool is_dummy)
    : grid_(&grid), color_(&tr_colors), tid_(&tid), lower_(lower), upper_(upper),
      is_dummy_(is_dummy) {
    wire_w_ = grid.get_wire_width(tid.get_level(), tid.get_ntr());
}

bool warr_rect_iterator::operator==(const warr_rect_iterator &rhs) const {
    return tr_idx_ == rhs.tr_idx_ && ww_idx_ == rhs.ww_idx_;
}

bool warr_rect_iterator::operator!=(const warr_rect_iterator &rhs) const { return !(*this == rhs); }

std::tuple<layer_t, box_t> warr_rect_iterator::operator*() const {
    auto level = tid_->get_level();

    auto &tinfo = (*grid_)[level];
    auto htr = (*tid_)[tr_idx_];
    auto & [ rel_htr, w ] = wire_w_[ww_idx_];
    auto c = htr_to_coord(tinfo, rel_htr + htr);
    auto half_w = w / 2;
    return {get_layer_t(*(grid_->get_tech()), *color_, level, htr + rel_htr, is_dummy_),
            box_t(tinfo.get_direction(), lower_, upper_, c - half_w, c + half_w)};
}

warr_rect_iterator &warr_rect_iterator::operator++() {
    ++ww_idx_;
    if (ww_idx_ == wire_w_.size()) {
        ww_idx_ = 0;
        ++tr_idx_;
    }
    return *this;
}

const tech *warr_rect_iterator::get_tech() const { return grid_->get_tech(); }

warr_rect_iterator begin_rect(const routing_grid &grid, const track_coloring &tr_colors,
                              const track_id &tid, coord_t lower, coord_t upper, bool is_dummy) {
    return {grid, tr_colors, tid, lower, upper, is_dummy};
}

warr_rect_iterator end_rect(const track_id &tid) { return warr_rect_iterator(tid); }

} // namespace layout
} // namespace cbag
