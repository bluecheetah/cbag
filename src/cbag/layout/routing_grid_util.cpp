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

#include <fmt/core.h>

#include <yaml-cpp/tuple.h>
#include <yaml-cpp/yaml.h>

#include <cbag/common/box_t.h>
#include <cbag/enum/space_type.h>
#include <cbag/layout/routing_grid_util.h>
#include <cbag/layout/tech_util.h>
#include <cbag/layout/track_info_util.h>
#include <cbag/layout/via_param_util.h>
#include <cbag/layout/wire_width.h>
#include <cbag/util/binary_iterator.h>
#include <cbag/util/math.h>
#include <cbag/yaml/common.h>
#include <cbag/yaml/track_info.h>

namespace cbag {
namespace layout {

routing_grid make_grid(std::shared_ptr<const tech> tech_ptr, const std::string &fname) {
    auto node = YAML::LoadFile(fname);

    auto tmp = cbagyaml::int_map_to_vec<track_info>(node["routing_grid"]);
    auto bot_level = std::get<0>(tmp);
    auto top_ignore_level = bot_level - 1;
    auto top_private_level = bot_level - 1;
    auto &info_list = std::get<1>(tmp);

    return {std::move(tech_ptr), bot_level, top_ignore_level, top_private_level,
            std::move(info_list)};
}

std::array<offset_t, 2> get_margins(const routing_grid &grid, layer_t key, level_t lev,
                                    const rect_t &obj) {
    auto tech_ptr = grid.get_tech();
    std::array<offset_t, 2> ans{0, 0};
    auto dir = grid[lev].get_direction();
    auto pdir = perpendicular(dir);
    auto width = dimension(obj, static_cast<orientation_2d>(to_int(pdir)));
    ans[to_int(dir)] = tech_ptr->get_min_space(key, width, space_type::LINE_END, false);
    ans[to_int(pdir)] = tech_ptr->get_min_space(key, width, space_type::DIFF_COLOR, false);
    return ans;
}

std::array<offset_t, 2> get_margins(const routing_grid &grid, layer_t key, level_t lev,
                                    const poly_90_t &obj) {
    // TODO: add margins around polygon?
    return {0, 0};
}

std::array<offset_t, 2> get_margins(const routing_grid &grid, layer_t key, level_t lev,
                                    const poly_45_t &obj) {
    // TODO: add margins around polygon?
    return {0, 0};
}

std::array<offset_t, 2> get_margins(const routing_grid &grid, layer_t key, level_t lev,
                                    const poly_t &obj) {
    // TODO: add margins around polygon?
    return {0, 0};
}

level_t get_lower_orthogonal_level(const routing_grid &grid, level_t level) {
    auto top_dir = grid.track_info_at(level).get_direction();
    auto bot_lev = level - 1;
    auto min_lev = grid.get_bot_level();
    while (bot_lev >= min_lev && grid[bot_lev].get_direction() == top_dir) {
        --bot_lev;
    }
    return bot_lev;
}

bool block_defined_at(const routing_grid &grid, level_t level) {
    auto bot_lev = get_lower_orthogonal_level(grid, level);
    return bot_lev >= grid.get_bot_level() && bot_lev > grid.get_top_private_level();
}

std::array<offset_t, 2> get_top_track_pitches(const routing_grid &grid, level_t level) {
    if (!block_defined_at(grid, level))
        throw std::invalid_argument("Size is undefined at layer " + std::to_string(level));

    auto bot_lev = get_lower_orthogonal_level(grid, level);
    auto &tinfo = grid[level];
    auto &binfo = grid[bot_lev];

    std::array<offset_t, 2> ans;
    auto tidx = to_int(tinfo.get_direction());
    ans[1 - tidx] = tinfo.get_pitch();
    ans[tidx] = binfo.get_pitch();
    return ans;
}

std::array<offset_t, 2> get_via_extensions_dim_helper(const tech &t, direction_1d vdir, layer_t key,
                                                      layer_t adj_key, orientation_2d dir,
                                                      orientation_2d adj_dir, offset_t dim,
                                                      offset_t adj_dim) {
    auto &via_id = t.get_via_id(vdir, key, adj_key);
    vector vbox_dim;
    vbox_dim[to_int(dir)] = adj_dim;
    vbox_dim[to_int(adj_dir)] = dim;
    auto via_param = t.get_via_param(vbox_dim, via_id, vdir, dir, adj_dir, true);

    if (empty(via_param)) {
        throw std::invalid_argument(fmt::format("Cannot draw via {} in area with w = {}, h = {}, "
                                                "vdir = {}, dir = {}, adj_dir = {}",
                                                via_id, vbox_dim[0], vbox_dim[1], to_int(vdir),
                                                to_int(dir), to_int(adj_dir)));
    }

    return get_via_extensions(via_param, vbox_dim, vdir, dir, adj_dir);
}

std::array<offset_t, 2> get_via_extensions_dim(const routing_grid &grid, direction_1d vdir,
                                               level_t level, offset_t dim, offset_t adj_dim) {
    auto adj_level = get_adj_level(vdir, level);
    auto &tr_info = grid.track_info_at(level);
    auto &adj_tr_info = grid.track_info_at(adj_level);
    auto dir = tr_info.get_direction();
    auto adj_dir = adj_tr_info.get_direction();
    if (dir == adj_dir) {
        throw std::invalid_argument(
            "get_via_extensions_dim() requires two layers with different directions.");
    }

    auto &tech = *grid.get_tech();
    auto key = get_test_lay_purp(tech, level);
    auto adj_key = get_test_lay_purp(tech, adj_level);

    return get_via_extensions_dim_helper(tech, vdir, key, adj_key, dir, adj_dir, dim, adj_dim);
}

std::array<offset_t, 2> get_via_extensions(const routing_grid &grid, direction_1d vdir,
                                           level_t level, cnt_t ntr, cnt_t adj_ntr) {
    auto adj_level = get_adj_level(vdir, level);
    auto &tr_info = grid.track_info_at(level);
    auto &adj_tr_info = grid.track_info_at(adj_level);
    auto dir = tr_info.get_direction();
    auto adj_dir = adj_tr_info.get_direction();
    if (dir == adj_dir) {
        // TODO: implement for layers with same direction
        throw std::invalid_argument(
            "get_via_extensions() right now requires two layers with different directions.");
    }

    auto &tech = *grid.get_tech();
    auto key = get_test_lay_purp(tech, level);
    auto adj_key = get_test_lay_purp(tech, adj_level);

    auto wire_width = grid.get_wire_width(level, ntr);
    auto adj_wire_width = grid.get_wire_width(adj_level, adj_ntr);

    auto ans = std::array<offset_t, 2>{0, 0};

    auto adj_edge_w = adj_wire_width.get_edge_wire_width();
    auto iter_stop = wire_width.end_width();
    auto ans_idx = to_int(vdir);
    for (auto iter = wire_width.begin_width(); iter != iter_stop; ++iter) {
        auto tmp = get_via_extensions_dim_helper(tech, vdir, key, adj_key, dir, adj_dir, *iter,
                                                 adj_edge_w);
        ans[ans_idx] = std::max(ans[ans_idx], tmp[ans_idx]);
    }

    auto edge_w = wire_width.get_edge_wire_width();
    iter_stop = adj_wire_width.end_width();
    ans_idx = 1 - ans_idx;
    for (auto iter = adj_wire_width.begin_width(); iter != iter_stop; ++iter) {
        auto tmp =
            get_via_extensions_dim_helper(tech, vdir, key, adj_key, dir, adj_dir, edge_w, *iter);
        ans[ans_idx] = std::max(ans[ans_idx], tmp[ans_idx]);
    }

    return ans;
}

std::array<offset_t, 2> get_via_extensions_dim_tr(const routing_grid &grid, direction_1d vdir,
                                                  level_t level, offset_t dim, cnt_t adj_ntr) {
    auto adj_level = get_adj_level(vdir, level);
    auto &tr_info = grid.track_info_at(level);
    auto &adj_tr_info = grid.track_info_at(adj_level);
    auto dir = tr_info.get_direction();
    auto adj_dir = adj_tr_info.get_direction();
    if (dir == adj_dir) {
        throw std::invalid_argument(
            "get_via_extensions_dim_tr() right now requires two layers with different directions.");
    }

    auto &tech = *grid.get_tech();
    auto key = get_test_lay_purp(tech, level);
    auto adj_key = get_test_lay_purp(tech, adj_level);

    auto adj_wire_width = grid.get_wire_width(adj_level, adj_ntr);

    auto ans = std::array<offset_t, 2>{0, 0};

    auto adj_edge_w = adj_wire_width.get_edge_wire_width();
    auto ans_idx = to_int(vdir);
    ans[ans_idx] = get_via_extensions_dim_helper(tech, vdir, key, adj_key, dir, adj_dir, dim,
                                                 adj_edge_w)[ans_idx];

    auto iter_stop = adj_wire_width.end_width();
    ans_idx = 1 - ans_idx;
    for (auto iter = adj_wire_width.begin_width(); iter != iter_stop; ++iter) {
        auto tmp =
            get_via_extensions_dim_helper(tech, vdir, key, adj_key, dir, adj_dir, dim, *iter);
        ans[ans_idx] = std::max(ans[ans_idx], tmp[ans_idx]);
    }

    return ans;
}

htr_t get_sep_htr(const routing_grid &grid, level_t level, cnt_t ntr1, cnt_t ntr2,
                  bool same_color) {
    auto space_type = get_space_type(same_color);
    auto &tech = *(grid.get_tech());
    auto &tr_info = grid.track_info_at(level);
    auto p2 = tr_info.get_pitch() / 2;
    auto w1 = grid.get_wire_width(level, ntr1);
    auto w2 = grid.get_wire_width(level, ntr2);
    auto sp = std::max(get_min_space(tech, level, w1, space_type, false),
                       get_min_space(tech, level, w2, space_type, false));
    auto tot_dim = (w1.get_total_width(p2) + w2.get_total_width(p2)) / 2 + sp;
    return util::ceil(tot_dim, p2);
}

htr_t get_line_end_sep_htr(const routing_grid &grid, direction_1d vdir, level_t le_level, cnt_t ntr,
                           cnt_t adj_ntr) {
    auto sp_level = get_adj_level(vdir, le_level);
    auto &tr_info = grid.track_info_at(le_level);
    auto &tr_info_sp = grid.track_info_at(sp_level);
    if (tr_info.get_direction() == tr_info_sp.get_direction())
        throw std::invalid_argument("space layer must be orthogonal to wire layer.");

    auto via_ext = get_via_extensions(grid, vdir, le_level, ntr, adj_ntr)[to_int(vdir)];
    auto wire_w = grid.get_wire_width(le_level, ntr);
    auto sp_le = get_min_space(*grid.get_tech(), le_level, wire_w, space_type::LINE_END, false);
    auto htr_pitch = tr_info_sp.get_pitch() / 2;
    auto sp_wire_w = grid.get_wire_width(sp_level, adj_ntr).get_total_width(htr_pitch);
    return util::ceil(sp_wire_w + 2 * via_ext + sp_le, htr_pitch);
}

std::array<offset_t, 2> get_blk_size(const routing_grid &grid, level_t level, bool include_private,
                                     std::array<bool, 2> half_blk) {
    auto xidx = static_cast<int>(half_blk[0]);
    auto yidx = static_cast<int>(half_blk[1]);

    // default quantization is 2 if no half-block, 1 if half-block.
    std::array<offset_t, 2> ans = {2 - xidx, 2 - yidx};

    auto &top_info = grid.track_info_at(level);
    auto top_dir = top_info.get_direction();
    auto top_didx = to_int(top_dir);
    ans[1 - top_didx] = grid.get_blk_pitch(level, half_blk[1 - top_didx]);

    // find bot level track info
    auto bot_lev = get_lower_orthogonal_level(grid, level);
    if (bot_lev >= grid.get_bot_level()) {
        auto private_lev = grid.get_top_private_level();
        if (include_private || level <= private_lev || private_lev < bot_lev) {
            // bottom level is quantized only if:
            // 1. include_private flag is enabled.
            // 2. top and bottom level are both public or both private.
            ans[top_didx] = grid.get_blk_pitch(bot_lev, half_blk[top_didx]);
        }
    }
    return ans;
}

std::array<offset_t, 2> get_wire_bounds(const routing_grid &grid, level_t level, htr_t htr,
                                        cnt_t ntr) {
    auto &tinfo = grid.track_info_at(level);
    auto wire_w = grid.get_wire_width(level, ntr);
    auto half_w = wire_w.get_total_width(tinfo.get_pitch() / 2) / 2;
    auto coord = htr_to_coord(tinfo, htr);
    return std::array<offset_t, 2>{coord - half_w, coord + half_w};
}

em_specs_t get_wire_em_specs(const routing_grid &grid, level_t level, cnt_t ntr, offset_t length,
                             bool vertical, temp_t dc_temp, temp_t rms_dt) {
    auto wire_w = grid.get_wire_width(level, ntr);
    auto &tech = *grid.get_tech();
    return get_metal_em_specs(tech, level, wire_w, length, vertical, dc_temp, rms_dt);
}

em_specs_t get_via_em_specs(const routing_grid &grid, direction_1d vdir, level_t level,
                            const wire_width &wire_w, offset_t length, const wire_width &adj_wire_w,
                            offset_t adj_length, temp_t dc_temp, temp_t rms_dt) {
    auto &tech = *grid.get_tech();
    auto adj_level = get_adj_level(vdir, level);
    auto &tinfo = grid.track_info_at(level);
    auto &adj_tinfo = grid.track_info_at(adj_level);

    auto tr_dir = tinfo.get_direction();
    auto adj_tr_dir = adj_tinfo.get_direction();

    if (tr_dir == adj_tr_dir) {
        std::array<level_t, 2> lev_vec;
        lev_vec[to_int(vdir)] = level;
        lev_vec[1 - to_int(vdir)] = adj_level;

        throw std::invalid_argument(fmt::format("levels ({}, {}) have the same direction, but "
                                                "get_via_em_specs only works on orthogonal levels.",
                                                lev_vec[0], lev_vec[1]));
    }

    auto tdir_idx = to_int(tr_dir);
    auto key = get_test_lay_purp(tech, level);
    auto adj_key = get_test_lay_purp(tech, adj_level);
    auto &via_id = tech.get_via_id(vdir, key, adj_key);
    double idc = 0, iac_rms = 0, iac_peak = 0;
    vector vbox_dim;
    std::array<offset_t, 2> m_dim, adj_m_dim;
    for (auto wi = wire_w.begin_width(), we = wire_w.end_width(); wi != we; ++wi) {
        auto w = *wi;
        vbox_dim[1 - tdir_idx] = w;
        m_dim[0] = w;
        m_dim[1] = length;
        for (auto awi = adj_wire_w.begin_width(), awe = adj_wire_w.end_width(); awi != awe; ++awi) {
            auto aw = *awi;
            vbox_dim[tdir_idx] = aw;
            adj_m_dim[0] = aw;
            adj_m_dim[1] = adj_length;

            auto param = tech.get_via_param(vbox_dim, via_id, vdir, tr_dir, adj_tr_dir, true);
            if (empty(param)) {
                // no solution: return all zeros
                return {0, 0, 0};
            }
            auto is_array = param.num[0] > 1 || param.num[1] > 1;
            auto[cur_idc, cur_irms, cur_ipeak] =
                get_via_em_specs(tech, vdir, key, adj_key, param.cut_dim, m_dim, adj_m_dim,
                                 is_array, dc_temp, rms_dt);

            idc += cur_idc;
            iac_rms += cur_irms;
            iac_peak += cur_ipeak;
        }
    }

    return {idc, iac_rms, iac_peak};
}

cnt_t get_min_num_tr(const routing_grid &grid, level_t level, double idc, double iac_rms,
                     double iac_peak, offset_t length, cnt_t bot_ntr, cnt_t top_ntr, temp_t dc_temp,
                     temp_t rms_dt) {
    util::binary_iterator<cnt_t> bin_iter(1);

    auto &tech = *grid.get_tech();
    auto &tr_info = grid.track_info_at(level);
    auto tr_dir = tr_info.get_direction();
    auto bot_lev = level - 1;
    auto top_lev = level + 1;
    auto has_bot = (bot_ntr > 0) && grid.track_info_at(bot_lev).get_direction() != tr_dir;
    auto has_top = (top_ntr > 0) && grid.track_info_at(top_lev).get_direction() != tr_dir;

    double idc_max = 0, irms_max = 0, ipeak_max = 0;
    auto result = std::tie(idc_max, irms_max, ipeak_max);
    while (bin_iter.has_next()) {
        auto cur_ntr = *bin_iter;
        auto wire_w = grid.get_wire_width(level, cur_ntr);

        result = get_metal_em_specs(tech, level, wire_w, length, false, dc_temp, rms_dt);
        if (idc > idc_max || iac_rms > irms_max || iac_peak > ipeak_max) {
            bin_iter.up();
            continue;
        }
        // wire passes EM specs, check top/bottom via EM specs
        if (has_bot) {
            auto adj_wire_w = grid.get_wire_width(bot_lev, bot_ntr);
            result = get_via_em_specs(grid, direction_1d::UPPER, level, wire_w, length, adj_wire_w,
                                      -1, dc_temp, rms_dt);
            if ((idc_max == 0 && irms_max == 0 && ipeak_max == 0) || idc > idc_max ||
                iac_rms > irms_max || iac_peak > ipeak_max) {
                bin_iter.up();
                continue;
            }
        }
        if (has_top) {
            auto adj_wire_w = grid.get_wire_width(top_lev, top_ntr);
            result = get_via_em_specs(grid, direction_1d::LOWER, level, wire_w, length, adj_wire_w,
                                      -1, dc_temp, rms_dt);
            if ((idc_max == 0 && irms_max == 0 && ipeak_max == 0) || idc > idc_max ||
                iac_rms > irms_max || iac_peak > ipeak_max) {
                bin_iter.up();
                continue;
            }
        }
        // we got here, all EM specs passed.
        bin_iter.save();
        bin_iter.down();
    }

    return *bin_iter.get_save();
}

htr_t find_next_htr(const routing_grid &grid, level_t level, offset_t coord, cnt_t ntr,
                    round_mode mode, bool even) {
    auto int_mode = static_cast<senum_t>(mode);
    if (mode == round_mode::NEAREST || mode == round_mode::NONE)
        throw std::invalid_argument("Invalid find_next_htr rounding mode: " +
                                    std::to_string(int_mode));
    auto sgn = util::sign(int_mode);
    auto &tr_info = grid.track_info_at(level);
    auto pitch = tr_info.get_pitch();
    auto off = tr_info.get_offset();
    auto wire_w = grid.get_wire_width(level, ntr).get_total_width(pitch / 2);
    return coord_to_htr(coord + sgn * (wire_w / 2), pitch, off, mode, even);
}

cnt_t get_min_space(const routing_grid &grid, level_t level, cnt_t num_tr, bool same_color,
                    bool even) {
    auto wire_w = grid.get_wire_width(level, num_tr);
    return get_min_space(*grid.get_tech(), level, wire_w, get_space_type(same_color), even);
}

} // namespace layout
} // namespace cbag
