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

#include <algorithm>

#include <fmt/core.h>

#include <cbag/layout/tech_util.h>
#include <cbag/layout/wire_width.h>

namespace cbag {
namespace layout {

wire_width::width_iter::width_iter() = default;

wire_width::width_iter::width_iter(vec_type::const_iterator iter) : vec_iter(iter) {}

auto wire_width::width_iter::operator++() -> width_iter & {
    ++vec_iter;
    return *this;
}

auto wire_width::width_iter::operator*() const -> reference { return std::get<1>(*vec_iter); }

bool wire_width::width_iter::operator==(const width_iter &rhs) const {
    return vec_iter == rhs.vec_iter;
}

bool wire_width::width_iter::operator!=(const width_iter &rhs) const {
    return vec_iter != rhs.vec_iter;
}

wire_width::wire_width() = default;

wire_width::wire_width(vec_type &&widths) : widths_(std::move(widths)) {}

bool wire_width::operator==(const wire_width &rhs) const { return widths_ == rhs.widths_; }

std::string wire_width::to_string() const {
    if (widths_.empty())
        return "[]";

    auto content = fmt::format("[({}, {})", std::get<0>(widths_[0]), std::get<1>(widths_[0]));
    for (std::size_t idx = 1; idx < widths_.size(); ++idx) {
        content += fmt::format(",({}, {})", std::get<0>(widths_[idx]), std::get<1>(widths_[idx]));
    }
    content += "]";
    return content;
}

const std::tuple<htr_t, offset_t> &wire_width::operator[](std::size_t idx) const {
    return widths_[idx];
}

std::size_t wire_width::size() const { return widths_.size(); }

auto wire_width::begin() const -> vec_type::const_iterator { return widths_.begin(); }

auto wire_width::end() const -> vec_type::const_iterator { return widths_.end(); }

auto wire_width::begin_width() const -> width_iter { return width_iter(widths_.begin()); }

auto wire_width::end_width() const -> width_iter { return width_iter(widths_.end()); }

offset_t wire_width::get_edge_wire_width() const { return std::get<1>(widths_[0]); }

offset_t wire_width::get_total_width(offset_t half_pitch) const {
    auto & [ htr, w ] = widths_[0];
    return w + half_pitch * (std::get<0>(widths_[widths_.size() - 1]) - htr);
}

offset_t wire_width::get_sum_width() const {
    offset_t ans = 0;
    for (const auto &info : widths_) {
        ans += std::get<1>(info);
    }
    return ans;
}

} // namespace layout
} // namespace cbag
