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

#include <tuple>

#include <cbag/util/binary_iterator.h>

#include <cbag/common/transformation_util.h>
#include <cbag/layout/cellview.h>
#include <cbag/layout/grid_object.h>
#include <cbag/layout/routing_grid_util.h>
#include <cbag/layout/tech_util.h>
#include <cbag/layout/track_info_util.h>
#include <cbag/layout/via_util.h>
#include <cbag/layout/via_wrapper.h>
#include <cbag/util/interval.h>

namespace cbag {
namespace layout {

struct cellview::helper {
    static void add_inst(cellview &self, const instance &inst) {
        auto &inst_name = inst.get_inst_name();
        if (!inst_name.empty()) {
            // test if given name is valid
            if (self.inst_map.emplace(inst_name, inst).second)
                return;
        }
        auto map_end = self.inst_map.end();
        cbag::util::binary_iterator<cnt_t> iter(self.inst_name_cnt);
        while (iter.has_next()) {
            if (self.inst_map.find("X" + std::to_string(*iter)) == map_end) {
                iter.save();
                iter.down();
            } else {
                iter.up();
            }
        }

        self.inst_name_cnt = *(iter.get_save());
        self.inst_map.emplace("X" + std::to_string(self.inst_name_cnt), inst);
    }

    static poly_set_t &make_geometry(cellview &self, layer_t key) {
        auto iter = self.geo_map.find(key);
        if (iter == self.geo_map.end()) {
            iter = self.geo_map.emplace(std::move(key), poly_set_t()).first;
        }
        return iter->second;
    }

    static std::shared_ptr<geo_index_t> &get_geo_index(cellview &self, level_t lev) {
        return self.index_list[lev - self.get_grid()->get_bot_level()];
    }

    template <typename T> static void add_shape(cellview &self, layer_t key, const T &obj) {
        auto &geo = make_geometry(self, key);
        geo.insert(obj);

        auto lev_opt = self.get_tech()->get_level(key);
        if (lev_opt) {
            auto &grid = *self.get_grid();
            auto &index = get_geo_index(self, *lev_opt);
            auto [spx, spy] = get_margins(grid, key, *lev_opt, obj);
            index->insert(obj, spx, spy);
        }
    }

    static util::disjoint_intvs<> get_ip_fill_intvs(const geo_index_t &geo_idx,
                                                    const box_t &test_box, coord_t wl, coord_t wu,
                                                    int dir_idx, offset_t sp) {
        auto sp2 = sp / 2;

        util::disjoint_intvs<> intvs;
        intvs.emplace(false, false, false, std::array<offset_t, 2>{wl, wu});

        apply_intersect(
            geo_idx,
            [&test_box, &intvs, &dir_idx, &sp2](const auto &v) {
                auto pset = test_box & v;
                if (!pset.empty()) {
                    auto box = get_bbox(pset);
                    intvs.subtract(
                        std::array<offset_t, 2>{box[dir_idx][0] - sp2, box[dir_idx][1] + sp2});
                }
            },
            test_box, 0, 0, true);

        intvs.expand_space(wl, wu, sp2);
        return intvs;
    }

    static void fill_ip_edge(cellview &self, util::disjoint_intvs<> &intvs, coord_t wl, coord_t wu,
                             offset_t min_len, offset_t min_sp, level_t level, htr_t htr) {
        if (intvs.empty())
            return;
        auto lower = intvs[0][0];
        auto upper = intvs[intvs.size() - 1][1];
        if (lower < wl) {
            intvs.subtract(std::array<offset_t, 2>{lower, wl});
            intvs.emplace(true, true, false, std::array<offset_t, 2>{wl, wl});
        }
        if (upper > wu) {
            intvs.subtract(std::array<offset_t, 2>{wu, upper});
            intvs.emplace(true, true, false, std::array<offset_t, 2>{wu, wu});
        }
        intvs.fix_drc(wl, wu, min_len, min_sp);

        auto tid = track_id(level, htr, 1, 1, 0);
        for (auto [lower, upper] : intvs) {
            self.add_warr(tid, std::array<offset_t, 2>{lower, upper});
        }
    }

    static void fill_ip_end(cellview &self, util::disjoint_intvs<> &intvs, coord_t wl, coord_t wu,
                            level_t level, offset_t pitch, offset_t offset) {
        auto bnds = std::array<offset_t, 2>{wl, wu};
        for (auto [lower, upper] : intvs) {
            auto htr_l = coord_to_htr(lower, pitch, offset, round_mode::LESS_EQ, true);
            auto htr_u = coord_to_htr(upper, pitch, offset, round_mode::GREATER_EQ, true);
            self.add_warr(track_id(level, htr_l, 1, 1 + (htr_u - htr_l) / 2, 2), bnds);
        }
    }
};

cellview::cellview(std::shared_ptr<const routing_grid> grid, std::string cell_name)
    : grid_ptr(std::move(grid)), cell_name(std::move(cell_name)) {
    auto num = grid_ptr->get_num_levels();
    index_list.reserve(num);
    for (decltype(num) idx = 0; idx < num; ++idx) {
        index_list.emplace_back(std::make_shared<polygon::index::geo_index<coord_t>>());
    }
}

bool cellview::operator==(const cellview &rhs) const noexcept {
    return *grid_ptr == *(rhs.grid_ptr) && cell_name == rhs.cell_name && geo_map == rhs.geo_map &&
           inst_map == rhs.inst_map && pin_map == rhs.pin_map && via_list == rhs.via_list &&
           lay_block_map == rhs.lay_block_map && area_block_list == rhs.area_block_list &&
           boundary_list == rhs.boundary_list && label_list == rhs.label_list;
}

auto cellview::find_geometry(layer_t key) const -> decltype(geo_map.find(key)) {
    return geo_map.find(key);
}

const std::string &cellview::get_name() const noexcept { return cell_name; }
const tech *cellview::get_tech() const noexcept { return grid_ptr->get_tech(); }
const routing_grid *cellview::get_grid() const noexcept { return grid_ptr.get(); }

bool cellview::empty() const noexcept {
    return geo_map.empty() && inst_map.empty() && via_list.empty() && lay_block_map.empty() &&
           area_block_list.empty() && boundary_list.empty() && pin_map.empty();
}

const std::shared_ptr<polygon::index::geo_index<coord_t>> &
cellview::get_geo_index(level_t lev) const {
    return index_list[lev - get_grid()->get_bot_level()];
}

auto cellview::begin_inst() const -> decltype(inst_map.cbegin()) { return inst_map.cbegin(); }
auto cellview::end_inst() const -> decltype(inst_map.cend()) { return inst_map.cend(); }
auto cellview::begin_geometry() const -> decltype(geo_map.cbegin()) { return geo_map.cbegin(); }
auto cellview::end_geometry() const -> decltype(geo_map.cend()) { return geo_map.cend(); }
auto cellview::begin_via() const -> decltype(via_list.cbegin()) { return via_list.cbegin(); }
auto cellview::end_via() const -> decltype(via_list.cend()) { return via_list.cend(); }
auto cellview::begin_lay_block() const -> decltype(lay_block_map.cbegin()) {
    return lay_block_map.cbegin();
}
auto cellview::end_lay_block() const -> decltype(lay_block_map.cend()) {
    return lay_block_map.cend();
}
auto cellview::begin_area_block() const -> decltype(area_block_list.cbegin()) {
    return area_block_list.cbegin();
}
auto cellview::end_area_block() const -> decltype(area_block_list.cend()) {
    return area_block_list.cend();
}
auto cellview::begin_boundary() const -> decltype(boundary_list.cbegin()) {
    return boundary_list.cbegin();
}
auto cellview::end_boundary() const -> decltype(boundary_list.cend()) {
    return boundary_list.cend();
}
auto cellview::begin_pin() const -> decltype(pin_map.cbegin()) { return pin_map.cbegin(); }
auto cellview::end_pin() const -> decltype(pin_map.cend()) { return pin_map.cend(); }
auto cellview::begin_label() const -> decltype(label_list.cbegin()) { return label_list.cbegin(); }
auto cellview::end_label() const -> decltype(label_list.cend()) { return label_list.cend(); }

void cellview::set_grid(const std::shared_ptr<const routing_grid> &grid) {
    if (!empty())
        throw std::runtime_error("Cannot change routing grid with non-empty layout.");
    grid_ptr = grid;
}

void cellview::add_pin(lay_t lay_id, std::string &&net, std::string &&label, box_t &&bbox) {
    auto iter = pin_map.find(lay_id);
    if (iter == pin_map.end()) {
        iter = pin_map.emplace(lay_id, std::vector<pin>()).first;
    }
    iter->second.emplace_back(std::move(bbox), std::move(net), std::move(label));
}

void cellview::add_label(layer_t &&key, transformation &&xform, std::string &&label,
                         offset_t height) {
    label_list.emplace_back(std::move(key), std::move(xform), std::move(label), height);
}

void cellview::add_object(const blockage &obj) {
    if (obj.get_type() == blockage_type::placement) {
        // area blockage
        area_block_list.push_back(obj);
    } else {
        auto layer = obj.get_layer();
        auto iter = lay_block_map.find(layer);
        if (iter == lay_block_map.end()) {
            iter = lay_block_map.emplace(layer, std::vector<blockage>()).first;
        }
        iter->second.push_back(obj);
    }
}

void cellview::add_object(const boundary &obj) { boundary_list.push_back(obj); }

void cellview::add_object(boundary &&obj) { boundary_list.push_back(std::move(obj)); }

void cellview::add_object(const via_wrapper &obj) {
    via_list.push_back(obj.v);
    if (obj.add_layers) {
        auto [bot_key, unused, top_key] = get_tech()->get_via_layer_purpose(obj.v.get_via_id());
        (void)unused;
        auto bot_box = get_bot_box(obj.v);
        auto top_box = get_top_box(obj.v);
        add_shape(bot_key, bot_box);
        add_shape(top_key, top_box);
    }
}

void cellview::add_object(const instance &obj) {
    helper::add_inst(*this, obj);
    auto master = obj.get_cellview();
    if (master != nullptr) {
        // NOTE: all routing_grids are guaranteed to have the same levels.
        auto &grid = *get_grid();
        transformation xform_copy(obj.xform);
        auto tot_dx = obj.spx * obj.nx;
        auto tot_dy = obj.spy * obj.ny;
        for (auto cur_lev = grid.get_bot_level(); cur_lev <= grid.get_top_level(); ++cur_lev) {
            auto &parent_index = helper::get_geo_index(*this, cur_lev);
            auto &inst_index = master->get_geo_index(cur_lev);
            for (decltype(obj.nx) ix = 0; ix < obj.nx; ++ix, xform_copy.move_by(obj.spx, 0)) {
                for (decltype(obj.ny) iy = 0; iy < obj.ny; ++iy, xform_copy.move_by(0, obj.spy)) {
                    parent_index->insert(inst_index, xform_copy);
                }
                xform_copy.move_by(0, -tot_dy);
            }
            xform_copy.move_by(-tot_dx, 0);
        }
    }
}

void cellview::add_shape(layer_t key, const box_t &obj) { helper::add_shape(*this, key, obj); }
void cellview::add_shape(layer_t key, const poly_90_t &obj) { helper::add_shape(*this, key, obj); }
void cellview::add_shape(layer_t key, const poly_45_t &obj) { helper::add_shape(*this, key, obj); }
void cellview::add_shape(layer_t key, const poly_t &obj) { helper::add_shape(*this, key, obj); }
void cellview::add_shape(layer_t key, const poly_set_t &obj) {
    polygon::apply_polygons(obj, [this, &key](const auto &v) { helper::add_shape(*this, key, v); });
}

void cellview::add_warr(const track_id &tid, std::array<offset_t, 2> coord) {
    auto &grid = *get_grid();
    auto lev = tid.get_level();
    auto &index = helper::get_geo_index(*this, lev);
    for (auto iter = begin_rect(grid, tid, coord), stop = end_rect(grid, tid, coord); iter != stop;
         ++iter) {
        auto [key, box] = *iter;
        auto &geo = helper::make_geometry(*this, key);
        geo.insert(box);

        auto [spx, spy] = get_margins(grid, key, lev, box);
        index->insert(box, spx, spy);
    }
}
void cellview::do_max_space_fill(level_t level, const box_t &bbox, bool fill_boundaries,
                                 std::array<offset_t, 2> sp, std::array<offset_t, 2> margin,
                                 double density) {
    auto &grid = *get_grid();
    auto &tech = *(grid.get_tech());
    auto tinfo = grid.track_info_at(level);
    auto tdir = tinfo.get_direction();
    auto pdir = perpendicular(tdir);
    auto tdir_idx = to_int(tdir);
    auto pdir_idx = 1 - tdir_idx;
    auto pitch = tinfo.get_pitch();
    auto offset = tinfo.get_offset();
    auto lev_idx = level - grid.get_bot_level();
    auto &geo_idx = *index_list[lev_idx];

    auto m_w = margin[pdir_idx];
    auto m_l = margin[tdir_idx];
    auto sp_w = sp[pdir_idx];
    auto sp_l = sp[tdir_idx];

    auto tr_w = tinfo.get_width();
    auto tr_w2 = tr_w / 2;
    auto test_lp = get_test_lay_purp(tech, level);
    auto min_len = tech.get_min_length(test_lp, tr_w, true);
    auto min_sp = tech.get_min_space(test_lp, tr_w, space_type::LINE_END, false);

    if (fill_boundaries) {
        auto m_w2 = m_w / 2;
        auto m_l_delta = (m_l - min_len) / 2;

        auto pcoord_l = lower(bbox, pdir);
        auto pcoord_u = upper(bbox, pdir);
        auto tcoord_l = lower(bbox, tdir);
        auto tcoord_u = upper(bbox, tdir);
        auto wire_l = tcoord_l + m_l_delta;
        auto wire_u = tcoord_u - m_l_delta;
        if (wire_u - wire_l < min_len) {
            wire_l = (wire_l + wire_u - min_len) / 2;
            wire_u = wire_l + min_len;
        }

        // fill edges; get fill intervals
        // add extra 1 so that we detect touches correctly
        auto test_box = box_t(tdir, tcoord_l, tcoord_u, pcoord_l, pcoord_l + m_w + 1);
        auto intvs_l =
            helper::get_ip_fill_intvs(geo_idx, test_box, tcoord_l, tcoord_u, tdir_idx, sp_l);
        set_interval(test_box, pdir, pcoord_u - m_w - 1, pcoord_u);
        auto intvs_u =
            helper::get_ip_fill_intvs(geo_idx, test_box, tcoord_l, tcoord_u, tdir_idx, sp_l);

        // get fill tracks
        auto htr_l = coord_to_htr(pcoord_l + m_w2, pitch, offset, round_mode::LESS_EQ, false);
        auto htr_u = coord_to_htr(pcoord_u - m_w2, pitch, offset, round_mode::GREATER_EQ, false);
        auto coord_l = htr_to_coord(htr_l, pitch, offset);
        auto coord_u = htr_to_coord(htr_u, pitch, offset);
        if (pcoord_l + m_w >= coord_u - tr_w2 || pcoord_u - m_w <= coord_l + tr_w2) {
            // the block is narrow enough that filling left/right edge will effect each other,
            // which will make the fill not symmetric.
            // to avoid this, we fill the center track instead.
            intvs_l += intvs_u;
            helper::fill_ip_edge(
                *this, intvs_l, wire_l, wire_u, min_len, min_sp, level,
                coord_to_htr((pcoord_l + pcoord_u) / 2, pitch, offset, round_mode::NEAREST, false));
        } else {
            helper::fill_ip_edge(*this, intvs_l, wire_l, wire_u, min_len, min_sp, level, htr_l);
            helper::fill_ip_edge(*this, intvs_u, wire_l, wire_u, min_len, min_sp, level, htr_u);
        }

        // fill ends; get fill intervals
        set_interval(test_box, pdir, pcoord_l, pcoord_u);
        // add extra 1 so we detech touches
        set_interval(test_box, tdir, tcoord_l, tcoord_l + m_l + 1);
        intvs_l = helper::get_ip_fill_intvs(geo_idx, test_box, pcoord_l, pcoord_u, pdir_idx, sp_w);
        set_interval(test_box, tdir, tcoord_u - m_l - 1, tcoord_u);
        intvs_u = helper::get_ip_fill_intvs(geo_idx, test_box, pcoord_l, pcoord_u, pdir_idx, sp_w);

        if (tcoord_l + m_l >= wire_u - min_len || tcoord_u - m_l <= wire_l + min_len) {
            // the block is short enough that filling top/bottom ends will effect each other,
            // which will make the fill not symmetric.
            // to avoid this, we fill the center instead.
            wire_l = (wire_l + wire_u - min_len) / 2;
            wire_u = wire_l + min_len;
            intvs_l += intvs_u;
            helper::fill_ip_end(*this, intvs_l, wire_l, wire_u, level, pitch, offset);
        } else {
            helper::fill_ip_end(*this, intvs_l, wire_l, wire_l + min_len, level, pitch, offset);
            helper::fill_ip_end(*this, intvs_u, wire_u - min_len, wire_u, level, pitch, offset);
        }
    }

    // fill the center

    // get empty spaces
    auto sp_w2 = sp_w / 2;
    auto sp_l2 = sp_l / 2;
    auto grow_box = polygon::rectangle_data<coord_t>(static_cast<orientation_2d>(pdir_idx), -sp_w2,
                                                     sp_w2, -sp_l2, sp_l2);
    auto tot_geo = geo_idx.get_geometry();
    tot_geo.convolve_rect(grow_box).invert(bbox);

    // shrink space geometry
    /*
    auto shrink = polygon::convolve_box(tot_geo, box_t(pdir, sp_w2, -sp_w2, sp_l2, -sp_l2));
    auto shrink_grow = polygon::convolve_box(shrink, grow_box);
    polygon::operator-=(tot_geo, shrink_grow);
    polygon::operator+=(tot_geo, shrink);
    */
}

} // namespace layout
} // namespace cbag
