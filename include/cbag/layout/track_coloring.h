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

class track_coloring {
  private:
    std::vector<std::tuple<cnt_t, offset_t, offset_t>> data_;

  public:
    track_coloring();

    explicit track_coloring(std::vector<std::tuple<cnt_t, offset_t, offset_t>> &&data);

    bool operator==(const track_coloring &rhs) const noexcept;

    std::size_t size() const noexcept;

    std::size_t get_hash() const noexcept;

    std::string to_string() const noexcept;

    const std::tuple<cnt_t, offset_t, offset_t> &get_mod_scale_offset(std::size_t idx) const;

    cnt_t get_htr_parity(std::size_t idx, htr_t htr) const noexcept;
};

} // namespace layout
} // namespace cbag

#endif
