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

#include <algorithm>

#include <boost/container_hash/hash.hpp>

#include <cbag/layout/via_info.h>
#include <cbag/layout/via_param.h>
#include <cbag/util/unique_heap.h>

namespace cbag {
namespace layout {

constexpr offset_t MAX_INT = std::numeric_limits<offset_t>::max();

venc_data::venc_data() = default;

via_info::via_info() = default;

via_info::via_info(std::string &&name, vector &&cdim, vector &&s, std::vector<vector> &&s2_list,
                   std::vector<vector> &&s3_list, std::array<venc_info, 2> &&e_list)
    : name_(std::move(name)), cut_dim_(std::move(cdim)), sp_list_(), sp2_list_(std::move(s2_list)),
      sp3_list_(std::move(s3_list)), enc_list_(std::move(e_list)) {
    sp_list_.push_back(std::move(s));
    if (sp2_list_.empty()) {
        sp2_list_.push_back(sp_list_.front());
    }
    if (sp3_list_.empty()) {
        sp3_list_.insert(sp3_list_.end(), sp2_list_.begin(), sp2_list_.end());
    }
}

const std::string &via_info::get_name() const noexcept { return name_; }

cnt_t get_n_max(offset_t dim, offset_t w, offset_t sp) {
    if (sp == MAX_INT) {
        // NOTE: always return 1, as we could have negative enclosure
        return 1;
    }
    return (dim + sp) / (w + sp);
}

const std::vector<vector> &get_enc_list(const venc_info &enc_info, offset_t width) {
    for (const auto &data : enc_info) {
        if (width <= data.width)
            return data.enc_list;
    }
    return enc_info.back().enc_list;
}

vector get_metal_dim(vector box_dim, vector arr_dim, const venc_info &enc_info,
                     orientation_2d_t dir_idx, bool extend) {
    auto pdir_idx = 1 - dir_idx;
    auto wire_w = box_dim[pdir_idx];
    vector ans;
    ans[dir_idx] = MAX_INT;
    ans[pdir_idx] = wire_w;
    for (const auto &enc_vec : get_enc_list(enc_info, wire_w)) {
        vector enc_dim = {2 * enc_vec[0] + arr_dim[0], 2 * enc_vec[1] + arr_dim[1]};
        if (enc_dim[pdir_idx] <= box_dim[pdir_idx]) {
            if (extend || enc_dim[dir_idx] <= box_dim[dir_idx]) {
                // enclosure rule passed, get optimal metal dimension
                ans[dir_idx] = std::min(ans[dir_idx], enc_dim[dir_idx]);
            }
        }
    }
    if (ans[dir_idx] == MAX_INT) {
        // no solution
        ans[0] = 0;
        ans[1] = 0;
    } else {
        ans[dir_idx] = std::max(ans[dir_idx], box_dim[dir_idx]);
    }

    return ans;
}

offset_t get_arr_dim(cnt_t n, offset_t w, offset_t sp) {
    if (sp == MAX_INT)
        if (n != 1)
            return 0;
        else
            return w;
    else
        return n * (w + sp) - sp;
}

via_param via_info::get_via_param(vector box_dim, direction_1d vdir, orientation_2d ex_dir,
                                  orientation_2d ex_adj_dir, bool extend) const {
    via_param ans;

    auto vidx = to_int(vdir);
    auto dir_idx = to_int(ex_dir);
    auto adj_dir_idx = to_int(ex_adj_dir);

    // get maximum possible number of vias
    auto min_sp = vector{MAX_INT, MAX_INT};
    for (const auto &arr : sp_list_) {
        min_sp[0] = std::min(min_sp[0], arr[0]);
        min_sp[1] = std::min(min_sp[1], arr[1]);
    }
    for (const auto &arr : sp2_list_) {
        min_sp[0] = std::min(min_sp[0], arr[0]);
        min_sp[1] = std::min(min_sp[1], arr[1]);
    }
    for (const auto &arr : sp3_list_) {
        min_sp[0] = std::min(min_sp[0], arr[0]);
        min_sp[1] = std::min(min_sp[1], arr[1]);
    }
    auto nx_max = get_n_max(box_dim[0], cut_dim_[0], min_sp[0]);
    auto ny_max = get_n_max(box_dim[1], cut_dim_[1], min_sp[1]);

    // check if it's possible to place a via
    if (nx_max == 0 || ny_max == 0)
        return ans;

    // use priority queue to find maximum number of vias
    util::unique_heap<via_cnt_t, boost::hash<via_cnt_t>> heap;
    heap.emplace(nx_max * ny_max, std::array<cnt_t, 2>{nx_max, ny_max});

    while (!heap.empty()) {
        auto & [ via_cnt, num_via ] = heap.top();
        auto[nx_cur, ny_cur] = num_via;

        // get list of valid via spacing
        const std::vector<vector> *sp_vec_ptr;
        if (nx_cur == 2 && ny_cur == 2) {
            sp_vec_ptr = &sp2_list_;
        } else if (nx_cur > 1 && ny_cur > 1) {
            sp_vec_ptr = &sp3_list_;
        } else {
            sp_vec_ptr = &sp_list_;
        }

        // find optimal legal enclosure via
        vector opt_arr_dim;
        std::array<vector, 2> opt_mdim = {vector{MAX_INT, MAX_INT}, vector{MAX_INT, MAX_INT}};
        vector opt_sp = {-1, -1};
        for (const auto &sp_via : *sp_vec_ptr) {
            vector arr_dim = {get_arr_dim(nx_cur, cut_dim_[0], sp_via[0]),
                              get_arr_dim(ny_cur, cut_dim_[1], sp_via[1])};
            if (arr_dim[0] == 0 || arr_dim[1] == 0)
                continue;

            auto m_dim = get_metal_dim(box_dim, arr_dim, enc_list_[vidx], dir_idx, extend);

            if (m_dim[0] == 0)
                continue;
            auto adj_m_dim =
                get_metal_dim(box_dim, arr_dim, enc_list_[vidx ^ 1], adj_dir_idx, extend);

            if (adj_m_dim[0] == 0)
                continue;

            if (m_dim[dir_idx] <= opt_mdim[vidx][dir_idx] &&
                adj_m_dim[adj_dir_idx] <= opt_mdim[1 - vidx][adj_dir_idx]) {
                opt_arr_dim = arr_dim;
                opt_mdim[vidx] = m_dim;
                opt_mdim[1 - vidx] = adj_m_dim;
                opt_sp = sp_via;
            }
        }

        if (opt_sp[0] < 0) {
            // fail to find legal via enclosure
            // continue searching
            if (nx_cur > 1) {
                heap.emplace((nx_cur - 1) * ny_cur, std::array<cnt_t, 2>{nx_cur - 1, ny_cur});
            }
            if (ny_cur > 1) {
                heap.emplace(nx_cur * (ny_cur - 1), std::array<cnt_t, 2>{nx_cur, ny_cur - 1});
            }
        } else {
            // solution found
            // set space to 0 if ony 1 via, this prevents space with MAX_INT values.
            if (nx_cur == 1)
                opt_sp[0] = 0;
            if (ny_cur == 1)
                opt_sp[1] = 0;
            auto enc1x = (opt_mdim[0][0] - opt_arr_dim[0]) / 2;
            auto enc1y = (opt_mdim[0][1] - opt_arr_dim[1]) / 2;
            auto enc2x = (opt_mdim[1][0] - opt_arr_dim[0]) / 2;
            auto enc2y = (opt_mdim[1][1] - opt_arr_dim[1]) / 2;

            ans = {nx_cur, ny_cur, cut_dim_[0], cut_dim_[1], opt_sp[0], opt_sp[1], enc1x,
                   enc1x,  enc1y,  enc1y,       enc2x,       enc2x,     enc2y,     enc2y};
            return ans;
        }

        heap.pop();
    }

    return ans;
}

via_info via_info::get_flip_xy(std::string new_name) const {
    auto ans = *this;
    ans.name_ = std::move(new_name);

    std::swap(ans.cut_dim_[0], ans.cut_dim_[1]);
    for (auto &vec : ans.sp_list_) {
        std::swap(vec[0], vec[1]);
    }
    for (auto &vec : ans.sp2_list_) {
        std::swap(vec[0], vec[1]);
    }
    for (auto &vec : ans.sp3_list_) {
        std::swap(vec[0], vec[1]);
    }
    for (auto &data : ans.enc_list_[0]) {
        for (auto &vec : data.enc_list) {
            std::swap(vec[0], vec[1]);
        }
    }
    for (auto &data : ans.enc_list_[1]) {
        for (auto &vec : data.enc_list) {
            std::swap(vec[0], vec[1]);
        }
    }

    return ans;
}

void _add_flipped_enclosures_helper(venc_info &obj) {
    for (auto &data : obj) {
        auto n = data.enc_list.size();
        for (decltype(n) i = 0; i < n; ++i) {
            auto &enc_vec = data.enc_list[i];
            if (enc_vec[0] != enc_vec[1]) {
                data.enc_list.push_back(vector{enc_vec[1], enc_vec[0]});
            }
        }
    }
}

void via_info::add_flipped_enclosures() {
    _add_flipped_enclosures_helper(enc_list_[0]);
    _add_flipped_enclosures_helper(enc_list_[1]);
}

} // namespace layout
} // namespace cbag
