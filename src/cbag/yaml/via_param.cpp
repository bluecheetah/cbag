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

#include <type_traits>

#include <cbag/logging/logging.h>

#include <cbag/common/typedefs.h>
#include <cbag/yaml/common.h>
#include <cbag/yaml/int_array.h>
#include <cbag/yaml/via_param.h>

namespace YAML {

bool convert<cbag::layout::via_param>::decode(const Node &node, cbag::layout::via_param &rhs) {
    auto logger = cbag::get_cbag_logger();
    try {
        rhs.num = node["num"].as<std::array<cbag::cnt_t, 2>>();
        rhs.cut_dim = node["cut_dim"].as<cbag::vector>();
        rhs.cut_spacing = node["cut_spacing"].as<cbag::vector>();
        rhs.enc = node["enc"].as<std::array<cbag::vector, 2>>();
        rhs.off = node["off"].as<std::array<cbag::vector, 2>>();
    } catch (...) {
        logger->warn("cbag::layout::via_param YAML decode exception.  Node:\n{}",
                     cbagyaml::node_to_str(node));
        return false;
    }
    return true;
}

} // namespace YAML
