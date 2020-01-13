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

#ifndef CBAG_NETLIST_CORE_H
#define CBAG_NETLIST_CORE_H

#include <string>
#include <type_traits>

#include <cbag/logging/logging.h>

#include <fmt/core.h>

#include <cbag/netlist/typedefs.h>
#include <cbag/schematic/cellview.h>
#include <cbag/schematic/cellview_info.h>
#include <cbag/spirit/ast.h>
#include <cbag/util/name_convert.h>

namespace cbag {

// forward declaration
struct time_struct;
struct binary_t;

namespace netlist {

// netlister base class

namespace traits {

template <typename T> struct nstream {
    using type = T;

    static void close(type &stream) {}

    static void write_header(type &stream, const std::vector<std::string> &inc_list, bool shell) {}

    static void write_end(type &stream) {}

    static void write_cv_header(type &stream, const std::string &name,
                                const sch::cellview_info &info, bool shell, bool write_subckt,
                                bool write_declarations) {}

    static void write_cv_end(type &stream, const std::string &name, bool write_subckt) {}

    static void write_unit_instance(type &stream, const std::string &prefix, cnt_t inst_idx,
                                    const spirit::ast::name_unit &name_ast,
                                    const term_net_vec_t &conn_list, const param_map &params,
                                    const sch::cellview_info &info,
                                    const net_rename_map_t *net_map_ptr) {}

    static void append_netlist(type &stream, const std::string &netlist) {}

    static void write_supply_wrapper(type &stream, const std::string &name,
                                     const sch::cellview_info &info) {}

    static net_rename_map_t new_net_rename_map() { return net_rename_map_t(); }
};

} // namespace traits

void split_array_inst_nets(term_net_vec_t &term_net_vec, const std::string &inst_name,
                           cnt_t inst_size,
                           const cbag::util::sorted_map<std::string, std::string> &connections,
                           const std::vector<std::string> &terms);

void update_rename_map(net_rename_map_t &new_map, const net_rename_map_t *net_map_ptr,
                       const std::string &prefix, const spirit::ast::name_unit &term,
                       const spirit::ast::name &net);

template <typename Stream, typename = typename traits::nstream<Stream>::type>
void write_cv_content(Stream &stream, const sch::cellview_info &info,
                      const sch::netlist_map_t &cell_map, const std::string &inst_prefix, bool flat,
                      const net_rename_map_t *net_map_ptr) {
    auto &cv = *(info.cv_ptr);

    for (auto const & [ inst_name, inst ] : cv.instances) {
        auto &cv_info = sch::get_cv_info(cell_map, inst->lib_name, inst->cell_name);
        // Only write instance if not ignore, and the name is not empty
        if (!cv_info.ignore && !cv_info.cell_name.empty()) {
            auto inst_ast = cbag::util::parse_cdba_name_unit(inst_name);
            auto n = inst_ast.size();
            auto term_net_vec = term_net_vec_t();
            split_array_inst_nets(term_net_vec, inst_name, n, inst->connections, cv_info.in_terms);
            split_array_inst_nets(term_net_vec, inst_name, n, inst->connections, cv_info.out_terms);
            split_array_inst_nets(term_net_vec, inst_name, n, inst->connections, cv_info.io_terms);

            for (decltype(n) inst_idx = 0; inst_idx < n; ++inst_idx) {
                if (flat && cv_info.cv_ptr) {
                    // flatten current non-primitive instance

                    // create rename map
                    auto new_map = traits::nstream<Stream>::new_net_rename_map();
                    for (const auto & [ term_ast, net_ast_list ] : term_net_vec) {
                        update_rename_map(new_map, net_map_ptr, inst_prefix, term_ast,
                                          net_ast_list[inst_idx]);
                    }

                    // create new prefix
                    auto new_prefix =
                        fmt::format("{}{}_", inst_prefix, inst_ast[inst_idx].to_string());

                    write_cv_content(stream, cv_info, cell_map, new_prefix, flat, &new_map);
                } else {
                    // write this instance
                    traits::nstream<Stream>::write_unit_instance(
                        stream, inst_prefix, inst_idx, inst_ast, term_net_vec, inst->params,
                        cv_info, net_map_ptr);
                }
            }
        }
    }
}

template <typename Stream, typename = typename traits::nstream<Stream>::type>
void add_cellview(Stream &stream, const std::string &name, const sch::cellview_info &info,
                  const sch::netlist_map_t &cell_map, bool shell, bool write_subckt, bool flat) {
    traits::nstream<Stream>::write_cv_header(stream, name, info, shell, write_subckt, true);
    if (!shell) {
        auto new_map = traits::nstream<Stream>::new_net_rename_map();
        write_cv_content(stream, info, cell_map, "", flat, &new_map);
    }
    traits::nstream<Stream>::write_cv_end(stream, name, write_subckt);
}

template <typename Stream, typename = typename traits::nstream<Stream>::type>
void add_cellview(Stream &stream, const std::string &netlist) {
    traits::nstream<Stream>::append_netlist(stream, netlist);
}

template <typename Stream, typename = typename traits::nstream<Stream>::type>
void add_supply_wrapper(Stream &stream, const std::string &name, const sch::cellview_info &info) {
    traits::nstream<Stream>::write_supply_wrapper(stream, name, info);
}

template <class OutIter> class write_param_visitor {
  private:
    OutIter &iter_;
    const std::string &key_;
    std::string dbl_fmt_;

  public:
    write_param_visitor(OutIter &iter, const std::string &key, uint_fast32_t precision)
        : iter_(iter), key_(key), dbl_fmt_(fmt::format("{{}}={{:.{}g}}", precision)) {}

    void operator()(const std::string &v) const {
        if (!v.empty()) {
            // if val is not given, do not write key in netlist
            *iter_ = fmt::format("{}={}", key_, v);
        }
    }
    void operator()(const int_fast32_t &v) const { *iter_ = fmt::format("{}={}", key_, v); }
    void operator()(const double_t &v) const { *iter_ = fmt::format(dbl_fmt_, key_, v); }
    void operator()(const bool &v) const {
        auto logger = cbag::get_cbag_logger();
        logger->warn("bool parameter, do nothing.");
    }
    void operator()(const time_struct &v) const {
        auto logger = cbag::get_cbag_logger();
        logger->warn("time parameter, do nothing.");
    }
    void operator()(const binary_t &v) const {
        auto logger = cbag::get_cbag_logger();
        logger->warn("binary parameter, do nothing.");
    }
};

} // namespace netlist
} // namespace cbag

#endif // CBAG_NETLIST_NETLIST_H
