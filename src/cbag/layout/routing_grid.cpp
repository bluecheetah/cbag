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

#include <functional>
#include <numeric>
#include <unordered_map>

#include <fmt/core.h>

#include <cbag/common/transformation_util.h>
#include <cbag/layout/routing_grid_util.h>
#include <cbag/layout/tech_util.h>
#include <cbag/layout/track_coloring.h>
#include <cbag/layout/track_info_util.h>
#include <cbag/util/math.h>

namespace cbag {
namespace layout {

struct routing_grid::helper {

    using intv_iter = cbag::util::disjoint_intvs<>::const_iterator;

    static std::size_t get_index(const routing_grid &grid, level_t level) {
        auto idx = static_cast<std::size_t>(level - grid.bot_level_);
        if (idx >= grid.info_list_.size())
            throw std::out_of_range("Undefined routing grid level: " + std::to_string(level));
        return idx;
    }

    static void update_blk_pitch_helper(std::vector<offset_t> &bp2_list,
                                        const std::vector<track_info> &info_list, level_t start,
                                        level_t stop) {
        auto prev_pitches = std::array<offset_t, 2>{1, 1};

        for (auto idx = start; idx < stop; ++idx) {
            auto &info = info_list[idx];
            auto cur_bp2 = info.get_pitch() / 2;
            auto didx = static_cast<orientation_2d_t>(info.get_direction());
            cur_bp2 = std::lcm(cur_bp2, prev_pitches[didx]);
            bp2_list.push_back(cur_bp2);
            prev_pitches[didx] = cur_bp2;
        }
    }

    static void update_block_pitch(std::vector<offset_t> &bp2_list,
                                   const std::vector<track_info> &info_list, level_t bot_level,
                                   level_t top_private, level_t top_ignore) {
        bp2_list.reserve(info_list.size());
        // set ignore layers block pitch
        for (decltype(bot_level) idx = 0; idx <= top_ignore - bot_level; ++idx) {
            bp2_list.push_back(-1);
        }

        // set private layers block pitch
        auto start = top_ignore + 1 - bot_level;
        auto stop = top_private + 1 - bot_level;
        if (stop > start)
            update_blk_pitch_helper(bp2_list, info_list, start, stop);

        // set public layers block pitch
        start = std::max(start, stop);
        stop = info_list.size();
        if (stop > start)
            update_blk_pitch_helper(bp2_list, info_list, start, stop);
    }

    static std::size_t compute_hash(const routing_grid &self) {
        std::size_t hash_val = 0;
        boost::hash_combine(hash_val, std::hash<std::string>{}(self.tech_ptr_->get_tech_lib()));
        boost::hash_combine(hash_val, self.bot_level_);
        boost::hash_combine(hash_val, self.top_ignore_level_);
        boost::hash_combine(hash_val, self.top_private_level_);
        for (const auto &info : self.info_list_) {
            boost::hash_combine(hash_val, info.get_hash());
        }
        return hash_val;
    }

    static void update_opt_info(std::tuple<offset_t, cnt_t, htr_t, htr_t> &opt_info, offset_t area,
                                offset_t w, const tech &t, layer_t lay, offset_t p2) {
        auto sp_via = t.get_min_space(lay, w, space_type::META_WIRE, false);
        auto htr_diff = util::ceil(w + sp_via, p2);
        auto htr_same = htr_diff + (htr_diff & 1);
        auto sep_diff = htr_diff * p2;
        auto sep_same = htr_same * p2;

        auto cur_num = static_cast<cnt_t>((area - w) / sep_diff) + 1;
        auto & [ opt_w, opt_num, opt_sep_d, opt_sep_s ] = opt_info;
        if ((cur_num & 1) == 0 && sep_same != sep_diff) {
            // even number of wires, middle is same color spacing
            // update number of wires if necessary
            auto test_num = static_cast<cnt_t>((area - w - sep_same) / sep_diff) + 2;
            cur_num = (test_num & 1) ? cur_num - 1 : test_num;
        }
        if (cur_num * w > opt_num * opt_w) {
            opt_w = w;
            opt_num = cur_num;
            opt_sep_d = htr_diff;
            opt_sep_s = htr_same;
        }
    }

    static void get_next_wire_width(std::vector<wire_width> &vec, const tech &t, layer_t lay,
                                    const cbag::util::disjoint_intvs<> &w_intvs, offset_t w,
                                    offset_t pitch) {
        // NOTE: all sub-wires are the same length
        auto p2 = pitch / 2;
        auto area = w + static_cast<offset_t>(vec.size()) * p2;

        auto iteru = w_intvs.upper_bound(area);
        auto opt_info = std::tuple<offset_t, cnt_t, htr_t, htr_t>{0, 0, 0, 0};
        auto & [ tr_w, num_w, sep_d, sep_s ] = opt_info;
        for (auto iter = w_intvs.begin(); iter != iteru; ++iter) {
            auto[wmin, wmax] = *iter;
            if (area < wmax) {
                // because of upper bound, this interval must contain area
                tr_w = area;
                num_w = 1;
                sep_d = 0;
                sep_s = 0;
            } else {
                // for now, only test the two extremes
                update_opt_info(opt_info, area, wmin, t, lay, p2);
                if (wmax > wmin + 1) {
                    update_opt_info(opt_info, area, wmax - 1, t, lay, p2);
                }
            }
        }

        if (num_w == 0) {
            throw std::runtime_error(fmt::format("Cannot draw wires in area {}", area));
        }

        auto w_info = std::vector<std::tuple<htr_t, offset_t>>();
        w_info.reserve(num_w);
        auto tot_area =
            static_cast<htr_t>((num_w & 1) ? sep_d * (num_w - 1) : sep_d * (num_w - 2) + sep_s);
        auto idx0 = -tot_area / 2;
        for (cnt_t cnt = 0; cnt < num_w; ++cnt) {
            w_info.push_back(std::tuple<htr_t, offset_t>{idx0, tr_w});
            if ((num_w & 1) == 0 && cnt == (num_w / 2 - 1)) {
                idx0 += sep_s;
            } else {
                idx0 += sep_d;
            }
        }

        vec.emplace_back(std::move(w_info));
    }
};

routing_grid::routing_grid() = default;

routing_grid::routing_grid(std::shared_ptr<const tech> &&t, level_t bot_level,
                           level_t top_ignore_level, level_t top_private_level,
                           std::vector<track_info> &&tinfo_list)
    : tech_ptr_(std::move(t)), bot_level_(bot_level), top_ignore_level_(top_ignore_level),
      top_private_level_(top_private_level), info_list_(std::move(tinfo_list)),
      ww_list_(info_list_.size()) {

    helper::update_block_pitch(bp2_list_, info_list_, bot_level_, top_private_level_,
                               top_ignore_level_);
    hash_val_ = helper::compute_hash(*this);
}

bool routing_grid::operator==(const routing_grid &rhs) const noexcept {
    return hash_val_ == rhs.hash_val_ && tech_ptr_ == rhs.tech_ptr_ &&
           bot_level_ == rhs.bot_level_ && top_ignore_level_ == rhs.top_ignore_level_ &&
           top_private_level_ == rhs.top_private_level_ && info_list_ == rhs.info_list_;
}

std::size_t routing_grid::get_hash() const noexcept { return hash_val_; }

const track_info &routing_grid::operator[](level_t level) const {
    return info_list_[static_cast<std::size_t>(level - bot_level_)];
}

offset_t routing_grid::get_blk_pitch(level_t level, bool half_blk) const {
    auto idx = helper::get_index(*this, level);
    return bp2_list_[idx] << (half_blk ^ 1);
}

const tech *routing_grid::get_tech() const noexcept { return tech_ptr_.get(); }

level_t routing_grid::get_bot_level() const noexcept { return bot_level_; }

level_t routing_grid::get_top_level() const noexcept { return bot_level_ + info_list_.size() - 1; }

std::size_t routing_grid::get_num_levels() const noexcept { return info_list_.size(); }

level_t routing_grid::get_top_ignore_level() const noexcept { return top_ignore_level_; }

level_t routing_grid::get_top_private_level() const noexcept { return top_private_level_; }

const track_info &routing_grid::track_info_at(level_t level) const {
    auto idx = helper::get_index(*this, level);
    return info_list_[idx];
}

const wire_width &routing_grid::get_wire_width(level_t level, cnt_t num_tr) const {
    auto idx = helper::get_index(*this, level);
    auto &ww_vec = ww_list_[idx];

    auto cur_size = ww_vec.size();
    if (num_tr > cur_size) {
        auto &tech = *get_tech();
        auto lay = get_test_lay_purp(tech, level);
        auto &tinfo = info_list_[idx];
        auto num_calc = num_tr - cur_size;
        auto w = tinfo.get_width();
        auto pitch = tinfo.get_pitch();
        auto w_dir = perpendicular(tinfo.get_direction());
        auto &w_intvs = tech_ptr_->get_width_intervals(level, w_dir);
        for (cnt_t idx = 0; idx < num_calc; ++idx) {
            helper::get_next_wire_width(ww_vec, tech, lay, w_intvs, w, pitch);
        }
    }
    return ww_vec[num_tr - 1];
}

track_coloring routing_grid::get_track_coloring_at(const track_coloring &tr_colors,
                                                   const transformation &xform,
                                                   const routing_grid &child,
                                                   level_t top_lev) const {
    if (swaps_xy(xform.orient()))
        throw std::invalid_argument("Unsupported orientation: " + to_string(xform.orient()));

    auto ascale = axis_scale(xform.orient());
    auto loc = xform.offset();
    std::vector<color_info> data;
    data.reserve(tr_colors.size());

    auto bot_lev = child.get_top_ignore_level();
    for (cnt_t lev_idx = 0; lev_idx < tr_colors.size(); ++lev_idx) {
        auto cur_lev = static_cast<level_t>(bot_level_ + lev_idx);
        auto &tr_info = operator[](cur_lev);
        auto &info = tr_colors.get_color_info(cur_lev);
        auto dir = tr_info.get_direction();
        auto didx = static_cast<orientation_2d_t>(dir);
        if (cur_lev <= bot_lev || cur_lev > top_lev || !tr_info.compatible(child[cur_lev]) ||
            (loc[1 - didx] % (tr_info.get_pitch() / 2)) != 0) {
            data.emplace_back(info.get_modulus(), 1, 0);
        } else {
            auto coord = loc[1 - didx] + tr_info.get_offset();
            auto htr = coord_to_htr(tr_info, coord);
            data.push_back(info.get_transform(ascale[1 - didx], htr));
        }
    }
    return track_coloring(bot_level_, std::move(data));
}

} // namespace layout
} // namespace cbag
