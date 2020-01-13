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
#include <cbag/yaml/track_info.h>

namespace YAML {

bool convert<cbag::layout::track_info>::decode(const Node &node, value_type &rhs) {
    auto logger = cbag::get_cbag_logger();
    try {
        auto dir_char = node[0].as<char>();
        auto tr_w = node[1].as<cbag::offset_t>();
        auto tr_sp = node[2].as<cbag::offset_t>();
        auto tr_off = (tr_w + tr_sp) / 2;
        auto tr_dir = static_cast<cbag::orientation_2d>(dir_char != 'x');

        rhs = value_type(tr_dir, tr_w, tr_sp, tr_off);
    } catch (...) {
        logger->warn("cbag::layout::track_info YAML decode exception.  Node:\n{}",
                     cbagyaml::node_to_str(node));
        return false;
    }
    return true;
}

} // namespace YAML
