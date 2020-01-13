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

color_info::color_info() noexcept = default;

color_info::color_info(cnt_t mod, offset_t scale, offset_t offset) noexcept
    : mod_(mod), scale_(scale), offset_(offset) {}

bool color_info::operator==(const color_info &rhs) const noexcept {
    return mod_ == rhs.mod_ && scale_ == rhs.scale_ && offset_ == rhs.offset_;
}

bool color_info::is_valid() const noexcept { return mod_ >= 1; }

std::size_t color_info::get_hash() const noexcept {
    auto seed = static_cast<std::size_t>(mod_);
    boost::hash_combine(seed, scale_);
    boost::hash_combine(seed, offset_);
    return seed;
}

std::string color_info::to_string() const {
    return fmt::format("({}, {}, {})", mod_, scale_, offset_);
}

cnt_t color_info::get_htr_parity(htr_t htr) const noexcept {
    // htr transforms to scale * htr + (scale - 1), since
    // htr = -1 aligns at the axis.
    return util::pos_mod(scale_ * htr + scale_ - 1 + offset_, 2 * mod_) / 2;
}

cnt_t color_info::get_modulus() const noexcept { return mod_; }

color_info color_info::get_transform(offset_t axis_scale, htr_t orig_htr) const noexcept {
    auto new_scale = axis_scale * scale_;
    auto new_offset = util::pos_mod(scale_ * orig_htr + offset_, 2 * mod_);
    if (mod_ == 2 && new_scale < 0) {
        // optimization: for 2 colors, sign flip is equivalent to shifting
        new_scale = 1;
        new_offset = 3 - new_offset;
    }
    return {mod_, new_scale, new_offset};
}

track_coloring::track_coloring() = default;

track_coloring::track_coloring(level_t bot_level, std::vector<color_info> &&data)
    : bot_level_(bot_level), data_(std::move(data)) {}

bool track_coloring::operator==(const track_coloring &rhs) const noexcept {
    return data_ == rhs.data_;
}

std::size_t track_coloring::size() const noexcept { return data_.size(); }

std::size_t track_coloring::get_hash() const noexcept {
    auto seed = static_cast<std::size_t>(0);
    for (const auto &info : data_) {
        boost::hash_combine(seed, info.get_hash());
    }
    return seed;
}

std::string track_coloring::to_string() const {
    auto n = data_.size();
    switch (n) {
    case 0:
        return "TrackColoring[]";
    case 1:
        return fmt::format("TrackColoring[{}]", data_.front().to_string());
    default: {
        auto ans = fmt::format("TrackColoring[{}", data_.front().to_string());
        for (decltype(n) idx = 1; idx < n; ++idx) {
            ans += fmt::format(", {}", data_[idx].to_string());
        }
        ans += "]";
        return ans;
    }
    }
}

cnt_t track_coloring::get_htr_parity(level_t level, htr_t htr) const noexcept {
    auto idx = level - bot_level_;
    if (idx < 0 || static_cast<std::size_t>(idx) >= data_.size())
        return 0;

    return data_[idx].get_htr_parity(htr);
}

const color_info &track_coloring::get_color_info(level_t level) const {
    return data_[level - bot_level_];
}

} // namespace layout
} // namespace cbag
