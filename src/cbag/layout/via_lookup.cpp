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

#include <yaml-cpp/yaml.h>

#include "yaml-cpp/tuple.h"
#include "yaml-cpp/unordered_map.h"

#include <cbag/layout/tech_util.h>
#include <cbag/layout/via_lookup.h>
#include <cbag/layout/via_param_util.h>
#include <cbag/yaml/via_info.h>

namespace cbag {
namespace layout {

const std::vector<via_info> &via_info_at(const vinfo_map_t &info_map, const std::string &via_id) {
    auto iter = info_map.find(via_id);
    if (iter == info_map.end())
        throw std::out_of_range("Cannot find via info for via ID: " + via_id);

    return iter->second;
}

vlayers_t parse_via_layers(const YAML::Node &node, const lp_lookup &lp) {
    return {layer_t_at(lp, node[0][0].as<std::string>(), node[0][1].as<std::string>()),
            layer_t_at(lp, node[1][0].as<std::string>(), node[1][1].as<std::string>())};
}

via_lookup::via_lookup() = default;

via_lookup::via_lookup(const YAML::Node &parent, const lp_lookup &lp)
    : lp_map(parent["via_layers"].as<vlp_map_t>()), info_map(parent["via"].as<vinfo_map_t>()) {
    for (const auto &node : parent["via_id"]) {
        id_map.emplace(parse_via_layers(node.first, lp), node.second.as<std::string>());
    }
}

via_lay_purp_t via_lookup::get_via_layer_purpose(const std::string &key) const {
    auto iter = lp_map.find(key);
    if (iter == lp_map.end()) {
        throw std::out_of_range(fmt::format("Cannot find via ID: {}", key));
    }
    return iter->second;
}

uint64_t get_via_score(const via_param &p) {
    return static_cast<uint64_t>(p.num[0]) * p.num[1] * p.cut_dim[0] * p.cut_dim[1];
}

const std::string &via_lookup::get_via_id(direction_1d vdir, layer_t layer,
                                          layer_t adj_layer) const {
    vlayers_t key;
    auto dir_idx = to_int(vdir);
    key[dir_idx] = layer;
    key[1 - dir_idx] = adj_layer;
    auto iter = id_map.find(key);
    if (iter == id_map.end())
        throw std::out_of_range(fmt::format("Cannot find via ID between ({}, {}) and ({}, {})",
                                            key[0].first, key[0].second, key[1].first,
                                            key[1].second));
    return iter->second;
}

via_param via_lookup::get_via_param(vector dim, const std::string &via_id, direction_1d vdir,
                                    orientation_2d ex_dir, orientation_2d adj_ex_dir,
                                    bool extend) const {
    auto adj_vdir = flip(vdir);
    auto &vinfo_list = via_info_at(info_map, via_id);
    auto vidx = to_int(vdir);
    via_param ans;
    auto opt_score = static_cast<uint64_t>(0);
    vector opt_ext_dim = {0, 0};
    for (const auto &vinfo : vinfo_list) {
        auto via_param = vinfo.get_via_param(dim, vdir, ex_dir, adj_ex_dir, extend);
        auto cur_score = get_via_score(via_param);
        vector cur_ext_dim;
        cur_ext_dim[vidx] = get_metal_dim(via_param, ex_dir, vdir);
        cur_ext_dim[1 - vidx] = get_metal_dim(via_param, adj_ex_dir, adj_vdir);
        if (cur_score > opt_score ||
            (cur_score > 0 && cur_score == opt_score && cur_ext_dim[0] <= opt_ext_dim[0] &&
             cur_ext_dim[1] <= opt_ext_dim[1])) {
            ans = std::move(via_param);
            opt_score = cur_score;
            opt_ext_dim = cur_ext_dim;
        }
    }

    return ans;
}

} // namespace layout
} // namespace cbag
