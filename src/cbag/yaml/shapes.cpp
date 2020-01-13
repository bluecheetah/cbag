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
#include <cbag/yaml/enum.h>
#include <cbag/yaml/rectangle_data.h>
#include <cbag/yaml/shapes.h>
#include <cbag/yaml/transformation.h>

namespace YAML {

Node convert<cbag::sch::eval_text>::encode(const value_type &rhs) {
    auto root = Node(NodeType::Map);
    root.force_insert("layer", rhs.layer);
    root.force_insert("purpose", rhs.purpose);
    root.force_insert("net", rhs.net);
    root.force_insert("origin", rhs.origin);
    root.force_insert("alignment", rhs.alignment);
    root.force_insert("orient", rhs.orient);
    root.force_insert("font", rhs.font);
    root.force_insert("height", rhs.height);
    root.force_insert("overbar", rhs.overbar);
    root.force_insert("visible", rhs.visible);
    root.force_insert("drafting", rhs.drafting);
    root.force_insert("text", rhs.text);
    root.force_insert("evaluator", rhs.evaluator);
    return root;
}

bool convert<cbag::sch::eval_text>::decode(const Node &node, value_type &rhs) {
    auto logger = cbag::get_cbag_logger();
    if (!node.IsMap()) {
        logger->warn("cbag::sch::eval_text YAML decode: not a map.  Node:\n{}",
                     cbagyaml::node_to_str(node));
        return false;
    }
    try {
        rhs.layer = node["layer"].as<cbag::lay_t>();
        rhs.purpose = node["purpose"].as<cbag::purp_t>();
        rhs.net = node["net"].as<std::string>();
        rhs.origin = node["origin"].as<cbag::point_t>();
        rhs.alignment = node["alignment"].as<cbag::text_align>();
        rhs.orient = node["orient"].as<cbag::orientation>();
        rhs.font = node["font"].as<cbag::font_t>();
        rhs.height = node["height"].as<cbag::dist_t>();
        rhs.overbar = node["overbar"].as<bool>();
        rhs.visible = node["visible"].as<bool>();
        rhs.drafting = node["drafting"].as<bool>();
        rhs.evaluator = node["evaluator"].as<std::string>();

        // NOTE for compatibility, if text attribute is not found, use empty string
        auto tmp = node["text"];
        if (tmp) {
            rhs.text = tmp.as<std::string>();
        } else {
            rhs.text = "";
        }

        return true;
    } catch (...) {
        logger->warn("cbag::sch::eval_text YAML decode exception.  Node:\n{}",
                     cbagyaml::node_to_str(node));
        return false;
    }
}

Node convert<cbag::sch::shape_t>::encode(const value_type &rhs) {
    Node root;
    root.push_back(rhs.index());
    std::visit(cbagyaml::to_yaml_visitor(&root), rhs);
    return root;
}

bool convert<cbag::sch::shape_t>::decode(const Node &node, value_type &rhs) {
    auto logger = cbag::get_cbag_logger();
    if (!node.IsSequence() || node.size() != 2) {
        logger->warn("cbag::sch::shape_t YAML decode: not a sequence or size != 2.  Node:\n{}",
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
            rhs = node[1].as<cbag::sch::polygon>();
            return true;
        case 2:
            rhs = node[1].as<cbag::sch::arc>();
            return true;
        case 3:
            rhs = node[1].as<cbag::sch::donut>();
            return true;
        case 4:
            rhs = node[1].as<cbag::sch::ellipse>();
            return true;
        case 5:
            rhs = node[1].as<cbag::sch::line>();
            return true;
        case 6:
            rhs = node[1].as<cbag::sch::path>();
            return true;
        case 7:
            rhs = node[1].as<cbag::sch::text_t>();
            return true;
        case 8:
            rhs = node[1].as<cbag::sch::eval_text>();
            return true;
        default:
            logger->warn("cbag::sch::shape_t YAML decode: unexpected which value: {}.  Node:\n{}",
                         value, cbagyaml::node_to_str(node));
            return false;
        }
    } catch (...) {
        logger->warn("cbag::sch::shape_t YAML decode exception.  Node:\n{}",
                     cbagyaml::node_to_str(node));
        return false;
    }
}
} // namespace YAML
