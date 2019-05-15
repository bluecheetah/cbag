// SPDX-License-Identifier: Apache-2.0
/*
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

#include <cbag/layout/track_coloring.h>
#include <cbag/util/math.h>

namespace cbag {
namespace layout {

track_coloring::track_coloring() = default;

track_coloring::track_coloring(std::vector<std::tuple<cnt_t, offset_t, offset_t>> &&data)
    : data_(std::move(data)) {}

bool track_coloring::operator==(const track_coloring &rhs) const noexcept {
    return data_ == rhs.data_;
}

std::size_t track_coloring::size() const noexcept { return data_.size(); }

std::size_t track_coloring::get_hash() const noexcept {
    return boost::hash_range(data_.begin(), data_.end());
}

std::string track_coloring::to_string() const noexcept {
    auto n = data_.size();
    switch (n) {
    case 0:
        return "TrackColoring[]";
    case 1:
        return fmt::format("TrackColoring[({}, {}, {})]", std::get<0>(data_[0]),
                           std::get<1>(data_[0]), std::get<2>(data_[0]));
    default: {
        auto ans = fmt::format("TrackColoring[({}, {}, {})", std::get<0>(data_[0]),
                               std::get<1>(data_[0]), std::get<2>(data_[0]));
        for (decltype(n) idx = 1; idx < n; ++idx) {
            ans += fmt::format(", ({}, {}, {})", std::get<0>(data_[idx]), std::get<1>(data_[idx]),
                               std::get<2>(data_[idx]));
        }
        ans += "]";
        return ans;
    }
    }
}

const std::tuple<cnt_t, offset_t, offset_t> &
track_coloring::get_mod_scale_offset(std::size_t idx) const {
    return data_[idx];
}

cnt_t track_coloring::get_htr_parity(std::size_t idx, htr_t htr) const noexcept {
    if (idx < 0 || idx >= data_.size())
        return 0;
    auto [modulus, scale, offset] = data_[idx];
    // htr transforms to scale * htr + (scale - 1), since
    // htr = -1 aligns at the axis.
    return util::pos_mod(scale * htr + scale - 1 + offset, 2 * modulus) / 2;
}

} // namespace layout
} // namespace cbag
