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

#include <cbag/common/box_collection.h>

namespace cbag {

box_collection::box_collection() = default;

std::size_t box_collection::size() const noexcept { return data_.size(); }

std::size_t box_collection::num_box() const noexcept { return nbox_; }

void box_collection::append(const box_t &base, cnt_t nx, cnt_t ny, offset_t spx, offset_t spy) {
    if (nx > 0 && ny > 0) {
        nbox_ += nx * ny;
        data_.push_back(
            box_array{base, std::array<cnt_t, 2>{nx, ny}, std::array<offset_t, 2>{spx, spy}});
    }
}

auto box_collection::begin() const -> vector_type::const_iterator { return data_.begin(); }

auto box_collection::end() const -> vector_type::const_iterator { return data_.end(); }

} // namespace cbag
