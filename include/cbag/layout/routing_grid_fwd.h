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

#ifndef CBAG_LAYOUT_ROUTING_GRID_FWD_H
#define CBAG_LAYOUT_ROUTING_GRID_FWD_H

#include <memory>

#include <cbag/common/transformation.h>
#include <cbag/common/typedefs.h>
#include <cbag/layout/tech.h>
#include <cbag/layout/track_coloring.h>
#include <cbag/layout/track_info.h>
#include <cbag/polygon/enum.h>

namespace cbag {

namespace layout {

class routing_grid {
  private:
    std::shared_ptr<const tech> tech_ptr = nullptr;
    level_t bot_level = 0;
    level_t top_ignore_level = -1;
    level_t top_private_level = -1;
    std::vector<track_info> info_list;
    std::vector<std::array<offset_t, 2>> blk_pitch_list;
    track_coloring tr_colors;
    std::size_t hash_val = 0;
    struct helper;

  public:
    routing_grid();

    routing_grid(std::shared_ptr<const tech> &&t, level_t bot_level, level_t top_ignore_level,
                 level_t top_private_level, std::vector<track_info> &&tinfo_list,
                 track_coloring &&tr_colors);

    bool operator==(const routing_grid &rhs) const noexcept;

    std::size_t get_hash() const noexcept;

    const track_info &track_info_at(level_t level) const;

    const track_info &operator[](level_t level) const;

    offset_t get_blk_pitch(level_t level, bool half_blk) const;

    const tech *get_tech() const noexcept;

    level_t get_bot_level() const noexcept;

    level_t get_top_level() const noexcept;

    std::size_t get_num_levels() const noexcept;

    level_t get_top_ignore_level() const noexcept;

    level_t get_top_private_level() const noexcept;

    const track_coloring &get_track_coloring() const noexcept;

    track_coloring get_track_coloring_at(level_t bot_lev, level_t top_lev,
                                         const transformation &xform) const;

    cnt_t get_htr_parity(level_t level, htr_t htr) const noexcept;

    template <typename TrackList>
    routing_grid get_copy_with(level_t top_ignore_level, level_t top_private_level,
                               track_coloring new_tr_color, const TrackList &tr_spec_list) const {
        auto new_tinfo_list = info_list;
        for (const auto [level, dir_code, w, sp, offset] : tr_spec_list) {
            auto tr_dir = static_cast<orientation_2d>(dir_code);
            auto &wintv_list = tech_ptr->get_width_intervals(level, tr_dir);
            new_tinfo_list[level - bot_level] = track_info(tr_dir, w, sp, offset, wintv_list);
        }
        return {std::shared_ptr<const tech>(tech_ptr),
                bot_level,
                top_ignore_level,
                top_private_level,
                std::move(new_tinfo_list),
                std::move(new_tr_color)};
    }
};

} // namespace layout
} // namespace cbag

#endif
