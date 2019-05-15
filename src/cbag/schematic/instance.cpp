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

#include <cstdlib>

#include <fmt/core.h>

#include <cbag/spirit/ast.h>
#include <cbag/spirit/namespace_info.h>

#include <cbag/util/name_convert.h>

#include <cbag/common/box_t.h>
#include <cbag/common/param_map_util.h>
#include <cbag/schematic/instance.h>

namespace cbag {
namespace sch {

instance::instance() = default;

instance::instance(std::string lib, std::string cell, std::string view, transformation xform,
                   coord_t xl, coord_t yl, coord_t xh, coord_t yh)
    : lib_name(std::move(lib)), cell_name(std::move(cell)), view_name(std::move(view)),
      xform(std::move(xform)), bbox(xl, yl, xh, yh), connections(), params() {}

offset_t instance::width() const { return cbag::polygon::width(bbox); }

offset_t instance::height() const { return cbag::polygon::height(bbox); }

std::unique_ptr<instance> instance::get_copy() const { return std::make_unique<instance>(*this); }

void instance::clear_params() { params.clear(); }

void instance::set_param(const std::string &name, const param_t &val) {
    cbag::set_param(params, name, val);
}

void instance::update_connection(const std::string &inst_name, cnt_t inst_size,
                                 std::string term_str, std::string net_str) {
    // check number of bits match
    auto n_term = cbag::util::parse_cdba_name_unit(term_str);
    auto n_net = cbag::util::parse_cdba_name(net_str);

    auto tot_size = inst_size * n_term.size();
    auto net_size = n_net.size();
    if (tot_size == net_size) {
        // direct connection
        connections.insert_or_assign(std::move(term_str), std::move(net_str));
    } else {
        auto result = std::div(static_cast<long>(tot_size), static_cast<long>(net_size));

        if (result.rem != 0) {
            // cannot broadcast net
            throw std::invalid_argument(fmt::format(
                "Cannot connect instance {} terminal {} to net {}", inst_name, term_str, net_str));
        }
        // broadcast net
        n_net.repeat(result.quot);
        connections.insert_or_assign(std::move(term_str),
                                     n_net.to_string(spirit::namespace_cdba{}));
    }
}

void instance::update_connection(const std::string &inst_name, std::string term, std::string net) {
    // check number of bits match
    auto nu = cbag::util::parse_cdba_name_unit(inst_name);
    update_connection(inst_name, nu.size(), std::move(term), std::move(net));
}

void instance::update_master(std::string lib, std::string cell, bool prim, bool keep_connections) {
    lib_name = std::move(lib);
    cell_name = std::move(cell);
    is_primitive = prim;

    clear_params();

    if (!keep_connections)
        connections.clear();
}

void instance::resize_nets(cnt_t old_size, cnt_t new_size) {
    auto result = std::div(static_cast<long>(new_size), static_cast<long>(old_size));
    if (result.rem != 0) {
        // new size not multiple of old size, just clear connections
        connections.clear();
    } else {
        // repeat all nets
        for (auto &pair : connections) {
            auto net = cbag::util::parse_cdba_name(pair.second);
            pair.second =
                net.repeat(static_cast<cnt_t>(result.quot)).to_string(spirit::namespace_cdba{});
        }
    }
}

} // namespace sch
} // namespace cbag
