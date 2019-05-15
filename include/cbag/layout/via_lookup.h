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

#ifndef CBAG_COMMON_VIA_LOOKUP_H
#define CBAG_COMMON_VIA_LOOKUP_H

#include <array>
#include <unordered_map>

#include <boost/container_hash/hash.hpp>

#include <cbag/common/box_t.h>
#include <cbag/common/layer_t.h>
#include <cbag/common/typedefs.h>
#include <cbag/layout/via_info.h>
#include <cbag/layout/via_param.h>
#include <cbag/polygon/enum.h>

namespace YAML {
class Node;
}

namespace cbag {
namespace layout {

class lp_lookup;

using via_lay_purp_t = std::tuple<layer_t, layer_t, layer_t>;
using vlp_map_t = std::unordered_map<std::string, via_lay_purp_t>;
using vlayers_t = std::array<layer_t, 2>;
using vid_map_t = std::unordered_map<vlayers_t, std::string, boost::hash<vlayers_t>>;
using vinfo_map_t = std::unordered_map<std::string, std::vector<via_info>>;

class via_lookup {
  private:
    vlp_map_t lp_map;
    vid_map_t id_map;
    vinfo_map_t info_map;

  public:
    via_lookup();

    via_lookup(const YAML::Node &parent, const lp_lookup &lp);

    via_lay_purp_t get_via_layer_purpose(const std::string &key) const;

    const std::string &get_via_id(direction_1d vdir, layer_t layer, layer_t adj_layer) const;

    via_param get_via_param(vector dim, const std::string &via_id, direction_1d vdir,
                            orientation_2d ex_dir, orientation_2d adj_ex_dir, bool extend) const;
};

} // namespace layout
} // namespace cbag

#endif
