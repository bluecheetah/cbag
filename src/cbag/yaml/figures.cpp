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

#include <cbag/yaml/common.h>
#include <cbag/yaml/datatypes.h>
#include <cbag/yaml/enum.h>
#include <cbag/yaml/figures.h>
#include <cbag/yaml/rectangle_data.h>
#include <cbag/yaml/shapes.h>
#include <cbag/yaml/transformation.h>

namespace YAML {

Node convert<cbag::sch::pin_fig_t>::encode(const cbag::sch::pin_fig_t &rhs) {
    Node root;
    root.push_back(rhs.index());
    std::visit(cbagyaml::to_yaml_visitor(&root), rhs);
    return root;
}

bool convert<cbag::sch::pin_fig_t>::decode(const Node &node, cbag::sch::pin_fig_t &rhs) {
    auto logger = cbag::get_cbag_logger();
    if (!node.IsSequence() || node.size() != 2) {
        logger->warn("cbag::sch::pin_fig_t YAML decode: not a sequence or size != 2.  Node:\n{}",
                     cbagyaml::node_to_str(node));
        return false;
    }
    try {
        int value = node[0].as<int>();
        switch (value) {
        case 0:
            rhs = node[1].as<cbag::sch::rectangle>();
            return true;
        case 1:
            rhs = node[1].as<cbag::sch::pin_object>();
            return true;
        default:
            logger->warn("cbag::sch::pin_fig_t YAML decode: unexpected which value: {}.  Node:\n{}",
                         value, cbagyaml::node_to_str(node));
            return false;
        }
    } catch (...) {
        logger->warn("cbag::sch::pin_fig_t YAML decode exception.  Node:\n{}",
                     cbagyaml::node_to_str(node));
        return false;
    }
}

Node convert<cbag::sch::pin_figure>::encode(const cbag::sch::pin_figure &rhs) {
    Node root(NodeType::Map);
    root.force_insert("obj", rhs.obj);
    root.force_insert("stype", rhs.stype);
    root.force_insert("ttype", rhs.ttype);
    if (!rhs.attrs.empty())
        root.force_insert("attrs", rhs.attrs);
    return root;
}

bool convert<cbag::sch::pin_figure>::decode(const Node &node, cbag::sch::pin_figure &rhs) {
    auto logger = cbag::get_cbag_logger();
    if (!node.IsMap()) {
        logger->warn("cbag::sch::pin_figure YAML decode: not a map.  Node:\n{}",
                     cbagyaml::node_to_str(node));
        return false;
    }
    try {
        rhs.obj = node["obj"].as<cbag::sch::pin_fig_t>();
        rhs.stype = node["stype"].as<cbag::sig_type>();
        rhs.ttype = node["ttype"].as<cbag::term_type>();
        if (node["attrs"])
            rhs.attrs = node["attrs"].as<cbag::util::sorted_map<std::string, std::string>>();
        else
            rhs.attrs.clear();

        return true;
    } catch (...) {
        logger->warn("cbag::sch::pin_figure YAML decode exception.  Node:\n{}",
                     cbagyaml::node_to_str(node));
        return false;
    }
}

} // namespace YAML
