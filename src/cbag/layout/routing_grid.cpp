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

#include <numeric>
#include <unordered_map>

#include <fmt/core.h>

#include <cbag/common/transformation_util.h>
#include <cbag/layout/routing_grid_util.h>
#include <cbag/layout/tech.h>
#include <cbag/layout/track_coloring.h>
#include <cbag/layout/track_info_util.h>
#include <cbag/util/math.h>

namespace cbag {
namespace layout {

struct routing_grid::helper {
    static std::size_t get_index(const routing_grid &grid, level_t level) {
        auto idx = static_cast<std::size_t>(level - grid.bot_level);
        if (idx >= grid.info_list.size())
            throw std::out_of_range("Undefined routing grid level: " + std::to_string(level));
        return idx;
    }

    static void update_blk_pitch_helper(std::vector<std::array<offset_t, 2>> &bp_list,
                                        const std::vector<track_info> &info_list, level_t start,
                                        level_t stop) {
        using p_type = std::array<offset_t, 2>;
        using p_vec = std::vector<p_type>;
        auto num_lay = stop - start;
        std::array<p_vec, 2> pitch_list = {p_vec{p_type{1, 1}}, p_vec{p_type{1, 1}}};
        pitch_list[0].reserve(num_lay);
        pitch_list[1].reserve(num_lay);

        for (auto idx = start; idx < stop; ++idx) {
            auto &info = info_list[idx];
            std::array<offset_t, 2> cur_bp;
            cur_bp[0] = info.get_pitch();
            cur_bp[1] = cur_bp[0] / 2;
            auto didx = static_cast<orientation_2d_t>(info.get_direction());
            for (const auto &[bp, bp2] : pitch_list[didx]) {
                cur_bp[0] = std::lcm(cur_bp[0], bp);
                cur_bp[1] = std::lcm(cur_bp[1], bp2);
            }
            bp_list.push_back(cur_bp);
            pitch_list[didx].push_back(cur_bp);
        }
    }

    static void update_block_pitch(std::vector<std::array<offset_t, 2>> &bp_list,
                                   const std::vector<track_info> &info_list, level_t bot_level,
                                   level_t top_private, level_t top_ignore) {
        bp_list.reserve(info_list.size());
        // set ignore layers block pitch
        for (decltype(bot_level) idx = 0; idx <= top_ignore - bot_level; ++idx) {
            bp_list.push_back(std::array<offset_t, 2>{-1, -1});
        }

        // set private layers block pitch
        auto start = top_ignore + 1 - bot_level;
        auto stop = top_private + 1 - bot_level;
        if (stop > start)
            update_blk_pitch_helper(bp_list, info_list, start, stop);

        // set public layers block pitch
        start = std::max(start, stop);
        stop = info_list.size();
        if (stop > start)
            update_blk_pitch_helper(bp_list, info_list, start, stop);
    }

    static std::size_t compute_hash(const routing_grid &self) {
        std::size_t hash_val = 0;
        boost::hash_combine(hash_val, self.tech_ptr);
        boost::hash_combine(hash_val, self.bot_level);
        boost::hash_combine(hash_val, self.top_ignore_level);
        boost::hash_combine(hash_val, self.top_private_level);
        for (const auto &info : self.info_list) {
            boost::hash_combine(hash_val, info.get_hash());
        }
        boost::hash_combine(hash_val, self.tr_colors.get_hash());
        return hash_val;
    }
};

routing_grid::routing_grid() = default;

routing_grid::routing_grid(std::shared_ptr<const tech> &&t, level_t bot_level,
                           level_t top_ignore_level, level_t top_private_level,
                           std::vector<track_info> &&tinfo_list, track_coloring &&tr_colors)
    : tech_ptr(std::move(t)), bot_level(bot_level), top_ignore_level(top_ignore_level),
      top_private_level(top_private_level), info_list(std::move(tinfo_list)),
      tr_colors(std::move(tr_colors)) {

    helper::update_block_pitch(blk_pitch_list, info_list, bot_level, top_private_level,
                               top_ignore_level);
    hash_val = helper::compute_hash(*this);
}

bool routing_grid::operator==(const routing_grid &rhs) const noexcept {
    return hash_val == rhs.hash_val && tech_ptr == rhs.tech_ptr && bot_level == rhs.bot_level &&
           top_ignore_level == rhs.top_ignore_level && top_private_level == rhs.top_private_level &&
           info_list == rhs.info_list && tr_colors == rhs.tr_colors;
}

std::size_t routing_grid::get_hash() const noexcept { return hash_val; }

const track_info &routing_grid::operator[](level_t level) const {
    return info_list[static_cast<std::size_t>(level - bot_level)];
}

offset_t routing_grid::get_blk_pitch(level_t level, bool half_blk) const {
    auto idx = helper::get_index(*this, level);
    return blk_pitch_list[idx][static_cast<std::size_t>(half_blk)];
}

const tech *routing_grid::get_tech() const noexcept { return tech_ptr.get(); }

level_t routing_grid::get_bot_level() const noexcept { return bot_level; }

level_t routing_grid::get_top_level() const noexcept { return bot_level + info_list.size() - 1; }

std::size_t routing_grid::get_num_levels() const noexcept { return info_list.size(); }

level_t routing_grid::get_top_ignore_level() const noexcept { return top_ignore_level; }

level_t routing_grid::get_top_private_level() const noexcept { return top_private_level; }

const track_info &routing_grid::track_info_at(level_t level) const {
    auto idx = helper::get_index(*this, level);
    return info_list[idx];
}

const track_coloring &routing_grid::get_track_coloring() const noexcept { return tr_colors; }

track_coloring routing_grid::get_track_coloring_at(level_t bot_lev, level_t top_lev,
                                                   const transformation &xform) const {
    if (swaps_xy(xform.orient()))
        throw std::invalid_argument("Unsupported orientation: " + to_string(xform.orient()));

    auto ascale = axis_scale(xform.orient());
    auto loc = xform.offset();
    std::vector<std::tuple<cnt_t, offset_t, offset_t>> data;
    data.reserve(tr_colors.size());

    for (cnt_t lev_idx = 0; lev_idx < tr_colors.size(); ++lev_idx) {
        level_t cur_lev = bot_level + lev_idx;
        auto [modulus, cur_scale, cur_offset] = tr_colors.get_mod_scale_offset(lev_idx);
        if (cur_lev < bot_lev || cur_lev > top_lev) {
            data.emplace_back(modulus, 1, 0);
        } else {
            auto &tr_info = operator[](cur_lev);
            auto dir = tr_info.get_direction();
            auto didx = static_cast<orientation_2d_t>(dir);
            auto coord = loc[1 - didx] + tr_info.get_offset();
            auto scale = ascale[1 - didx];
            auto htr = coord_to_htr(tr_info, coord);

            auto new_scale = cur_scale * scale;
            auto new_offset = util::pos_mod(cur_scale * htr + cur_offset, 2 * modulus);
            if (modulus == 2 && new_scale < 0) {
                // optimization: for 2 colors, sign flip is equivalent to shifting
                new_scale = 1;
                new_offset = 3 - new_offset;
            }
            data.emplace_back(modulus, new_scale, new_offset);
        }
    }
    return track_coloring(std::move(data));
}

cnt_t routing_grid::get_htr_parity(level_t level, htr_t htr) const noexcept {
    return tr_colors.get_htr_parity(level - bot_level, htr);
}

} // namespace layout
} // namespace cbag
