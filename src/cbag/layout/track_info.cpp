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

#include <cbag/logging/logging.h>

#include <cbag/layout/track_info.h>
#include <cbag/layout/wire_width.h>
#include <cbag/util/math.h>

namespace cbag {
namespace layout {

track_info::track_info() = default;

track_info::track_info(orientation_2d tr_dir, offset_t tr_w, offset_t tr_sp, offset_t tr_off,
                       const std::vector<std::array<offset_t, 2>> &intv_list)
    : dir(tr_dir), w(tr_w), sp(tr_sp), offset(tr_off) {
    for (const auto &intv : intv_list) {
        bool success = w_intvs.emplace(false, false, false, intv);
        if (!success) {
            throw std::invalid_argument(fmt::format(
                "Overlapping routing grid width intervals: [{}, {})", intv[0], intv[1]));
        }
    }
}

bool track_info::operator==(const track_info &rhs) const noexcept {
    return dir == rhs.dir && w == rhs.w && sp == rhs.sp && offset == rhs.offset &&
           w_intvs == rhs.w_intvs;
}

std::size_t track_info::get_hash() const noexcept {
    auto seed = static_cast<std::size_t>(0);
    boost::hash_combine(seed, dir);
    boost::hash_combine(seed, w);
    boost::hash_combine(seed, sp);
    boost::hash_combine(seed, offset);
    for (const auto &intv : w_intvs) {
        boost::hash_combine(seed, intv);
    }
    return seed;
}

bool track_info::compatible(const track_info &rhs) const noexcept {
    return dir == rhs.dir && w == rhs.w && sp == rhs.sp && offset == rhs.offset;
}

orientation_2d track_info::get_direction() const noexcept { return dir; }

offset_t track_info::get_width() const noexcept { return w; }

offset_t track_info::get_space() const noexcept { return sp; }

offset_t track_info::get_pitch() const noexcept { return w + sp; }

offset_t track_info::get_offset() const noexcept { return offset; }

wire_width track_info::get_wire_width(cnt_t num_tr) const {
    // TODO: implement this with w_intvs taken into account
    auto wire_w = w + static_cast<offset_t>(num_tr - 1) * get_pitch();
    return wire_width{
        std::vector<std::tuple<level_t, offset_t>>({std::tuple<level_t, offset_t>{0, wire_w}})};
}

} // namespace layout
} // namespace cbag
