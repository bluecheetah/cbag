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

#include <type_traits>

#include <cbag/logging/logging.h>

#include <cbag/common/typedefs.h>
#include <cbag/yaml/common.h>
#include <cbag/yaml/int_array.h>
#include <cbag/yaml/via_info.h>

namespace YAML {

bool convert<cbag::layout::venc_data>::decode(const Node &node, cbag::layout::venc_data &rhs) {
    auto logger = cbag::get_cbag_logger();

    if (!node.IsSequence() || node.size() != 2) {
        logger->warn("cbag::layout::vend_data YAML decode: not a sequence or size != 2.  Node:\n{}",
                     cbagyaml::node_to_str(node));
        return false;
    }

    try {
        rhs.width = cbagyaml::get_int<cbag::offset_t>(node[0]);

        rhs.enc_list = node[1].as<std::decay_t<decltype(rhs.enc_list)>>();
    } catch (...) {
        logger->warn("cbag::layout::venc_data YAML decode exception.  Node:\n{}",
                     cbagyaml::node_to_str(node));
        return false;
    }
    return true;
}

bool convert<cbag::layout::via_info>::decode(const Node &node, cbag::layout::via_info &rhs) {
    auto logger = cbag::get_cbag_logger();
    try {
        auto name = node["name"].as<std::string>();
        auto cut_dim = node["dim"].as<cbag::vector>();
        auto sp = node["sp"].as<cbag::vector>();
        auto sp2_node = node["sp2"];
        auto sp3_node = node["sp3"];
        auto sp2_list =
            (sp2_node) ? sp2_node.as<std::vector<cbag::vector>>() : std::vector<cbag::vector>();
        auto sp3_list =
            (sp3_node) ? sp3_node.as<std::vector<cbag::vector>>() : std::vector<cbag::vector>();
        std::array<cbag::layout::venc_info, 2> enc_list = {
            node["bot_enc"].as<cbag::layout::venc_info>(),
            node["top_enc"].as<cbag::layout::venc_info>()};
        auto priority = 1;
        if (node["priority"])
            priority = node["priority"].as<int>();

        rhs = cbag::layout::via_info(std::move(name), std::move(cut_dim), std::move(sp),
                                     std::move(sp2_list), std::move(sp3_list), std::move(enc_list),
                                     std::move(priority));
    } catch (...) {
        logger->warn("cbag::layout::via_info YAML decode exception.  Node:\n{}",
                     cbagyaml::node_to_str(node));
        return false;
    }
    return true;
}

} // namespace YAML
