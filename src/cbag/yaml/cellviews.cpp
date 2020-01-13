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

#include <cbag/logging/logging.h>

#include <cbag/yaml/cellviews.h>

namespace YAML {

Node convert<cbag::sch::cellview_info>::encode(const cbag::sch::cellview_info &rhs) {
    Node root(NodeType::Map);
    root.force_insert("lib_name", rhs.lib_name);
    root.force_insert("cell_name", rhs.cell_name);
    root.force_insert("in_terms", rhs.in_terms);
    root.force_insert("out_terms", rhs.out_terms);
    root.force_insert("io_terms", rhs.io_terms);
    root.force_insert("nets", rhs.nets);
    root.force_insert("props", rhs.props);
    root.force_insert("is_prim", rhs.is_prim);
    root.force_insert("ignore", rhs.ignore);
    if (!rhs.term_net_attrs.empty())
        root.force_insert("term_net_attrs", rhs.term_net_attrs);

    return root;
}

bool convert<cbag::sch::cellview_info>::decode(const Node &node, cbag::sch::cellview_info &rhs) {
    auto logger = cbag::get_cbag_logger();
    if (!node.IsMap()) {
        logger->warn("cbag::sch::cellview_info YAML decode: not a map.  Node:\n{}",
                     cbagyaml::node_to_str(node));
        return false;
    }
    try {
        rhs.lib_name = node["lib_name"].as<std::string>();
        rhs.cell_name = node["cell_name"].as<std::string>();
        rhs.in_terms = node["in_terms"].as<std::vector<std::string>>();
        rhs.out_terms = node["out_terms"].as<std::vector<std::string>>();
        rhs.io_terms = node["io_terms"].as<std::vector<std::string>>();
        rhs.nets = node["nets"].as<std::vector<std::string>>();
        rhs.props = node["props"].as<cbag::param_map>();
        rhs.is_prim = node["is_prim"].as<bool>();

        auto ignore_node = node["ignore"];
        if (ignore_node)
            rhs.ignore = ignore_node.as<bool>();
        else
            rhs.ignore = false;

        auto term_net_attrs_node = node["term_net_attrs"];
        if (term_net_attrs_node)
            rhs.term_net_attrs = term_net_attrs_node.as<cbag::util::sorted_map<
                std::string, cbag::util::sorted_map<std::string, std::string>>>();
        else
            rhs.term_net_attrs.clear();

        rhs.cv_ptr = nullptr;

        return true;
    } catch (...) {
        logger->warn("cbag::sch::cellview_info YAML decode exception.  Node:\n{}",
                     cbagyaml::node_to_str(node));
        return false;
    }
}

} // namespace YAML
