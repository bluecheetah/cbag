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

#ifndef CBAG_LAYOUT_TRACK_COLORING_H
#define CBAG_LAYOUT_TRACK_COLORING_H

#include <string>
#include <tuple>
#include <vector>

#include <cbag/common/typedefs.h>

namespace cbag {
namespace layout {

class routing_grid;

class color_info {
  private:
    cnt_t mod_ = 0;
    offset_t scale_ = 0;
    offset_t offset_ = 0;

  public:
    color_info() noexcept;

    color_info(cnt_t mod, offset_t scale, offset_t offset) noexcept;

    bool operator==(const color_info &rhs) const noexcept;

    bool is_valid() const noexcept;

    std::size_t get_hash() const noexcept;

    std::string to_string() const;

    cnt_t get_htr_parity(htr_t htr) const noexcept;

    cnt_t get_modulus() const noexcept;

    color_info get_transform(offset_t axis_scale, htr_t orig_htr) const noexcept;
};

class track_coloring {
  private:
    level_t bot_level_;
    std::vector<color_info> data_;

  public:
    track_coloring();

    track_coloring(level_t bot_level, std::vector<color_info> &&data);

    bool operator==(const track_coloring &rhs) const noexcept;

    std::size_t size() const noexcept;

    std::size_t get_hash() const noexcept;

    std::string to_string() const;

    cnt_t get_htr_parity(level_t level, htr_t htr) const noexcept;

    const color_info &get_color_info(level_t level) const;
};

} // namespace layout
} // namespace cbag

#endif
