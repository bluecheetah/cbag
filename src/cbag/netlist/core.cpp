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

#include <cbag/netlist/core.h>
#include <cbag/spirit/util.h>

namespace cbag {
namespace netlist {

namespace core {

class rename_iter {
  private:
    net_rename_map_t *new_map_ptr_ = nullptr;
    const spirit::ast::name_unit *term_ptr_ = nullptr;
    const net_rename_map_t *old_map_ptr_ = nullptr;
    const std::string *prefix_ptr_ = nullptr;
    cnt_t idx_ = 0;

  public:
    rename_iter() = default;

    rename_iter(net_rename_map_t *map, const spirit::ast::name_unit *term,
                const net_rename_map_t *old_map, const std::string *prefix)
        : new_map_ptr_(map), term_ptr_(term), old_map_ptr_(old_map), prefix_ptr_(prefix) {}

    rename_iter &operator*() { return *this; }
    rename_iter &operator++() { return *this; }
    rename_iter &operator=(spirit::ast::name_bit nb) {
        if (old_map_ptr_) {
            auto iter = old_map_ptr_->find(nb);
            if (iter == old_map_ptr_->end()) {
                nb.base = (*prefix_ptr_) + nb.base;
                new_map_ptr_->emplace((*term_ptr_)[idx_], std::move(nb));
            } else {
                new_map_ptr_->emplace((*term_ptr_)[idx_], iter->second);
            }
        } else {
            nb.base = (*prefix_ptr_) + nb.base;
            new_map_ptr_->emplace((*term_ptr_)[idx_], std::move(nb));
        }
        ++idx_;
        return *this;
    }
};

} // namespace core

void split_array_inst_nets(term_net_vec_t &term_net_vec, const std::string &inst_name,
                           cnt_t inst_size,
                           const cbag::util::sorted_map<std::string, std::string> &connections,
                           const std::vector<std::string> &terms) {
    for (const auto &term : terms) {
        auto term_iter = connections.find(term);
        if (term_iter == connections.end()) {
            throw std::invalid_argument(fmt::format(
                "Cannot find net connected to instance {} terminal {}", inst_name, term));
        }
        auto ast_term = cbag::util::parse_cdba_name_unit(term);
        auto ast_net = cbag::util::parse_cdba_name(term_iter->second);

        auto net_vec = std::vector<spirit::ast::name>();
        net_vec.reserve(inst_size);
        if (inst_size == 1) {
            net_vec.push_back(std::move(ast_net));
        } else {
            spirit::util::get_partition(ast_net, ast_term.size(), std::back_inserter(net_vec));
        }
        term_net_vec.emplace_back(ast_term, std::move(net_vec));
    }
}

void update_rename_map(net_rename_map_t &new_map, const net_rename_map_t *net_map_ptr,
                       const std::string &prefix, const spirit::ast::name_unit &term,
                       const spirit::ast::name &net) {
    spirit::util::get_name_bits(net, core::rename_iter(&new_map, &term, net_map_ptr, &prefix));
}

} // namespace netlist
} // namespace cbag
