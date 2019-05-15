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

#include <fmt/core.h>

#include <cbag/logging/logging.h>
#include <cbag/netlist/lstream.h>
#include <cbag/netlist/verilog.h>
#include <cbag/schematic/cellview.h>
#include <cbag/schematic/cellview_info.h>
#include <cbag/schematic/instance.h>
#include <cbag/spirit/ast.h>
#include <cbag/spirit/util.h>
#include <cbag/util/io.h>
#include <cbag/util/name_convert.h>

namespace cbag {
namespace netlist {

// verilog netlisting helper functions
namespace verilog {

using term_net_vec_t = std::vector<std::pair<std::string, std::vector<spirit::ast::name>>>;

void write_cv_ports(verilog_stream &stream, const std::vector<std::string> &terms,
                    bool &has_prev_term, const char *prefix,
                    const util::sorted_map<std::string, sch::attr_map_t> &term_net_attrs,
                    const std::string &def_type) {
    for (const auto &term : terms) {
        auto ast = util::parse_cdba_name_unit(term);

        auto type_str = sch::get_net_type(term_net_attrs, ast.base, def_type);

        lstream b;
        b << prefix << type_str;
        if (ast.is_vector()) {
            if (ast.idx_range.step != 1) {
                throw std::invalid_argument(
                    "Verilog does not support buses with non-unity step size: " + term);
            }
            b << fmt::format("[{}:{}]", ast.idx_range.start, ast.idx_range.get_stop_include());
        }
        b << ast.base;

        if (has_prev_term)
            stream.out_file << "," << std::endl;
        else
            has_prev_term = true;
        b.to_file(stream.out_file, spirit::namespace_verilog{}, false);
    }
}

void append_inst_nets(verilog_stream &stream, const std::string &inst_name,
                      const sch::instance &inst, const std::vector<std::string> &terms,
                      bool &has_prev_term) {
    for (auto const &term : terms) {
        auto term_iter = inst.connections.find(term);
        if (term_iter == inst.connections.end()) {
            throw std::invalid_argument(fmt::format(
                "Cannot find net connected to instance {} terminal {}", inst_name, term));
        }
        spirit::ast::name_unit term_ast = cbag::util::parse_cdba_name_unit(term);
        lstream b;
        b << "    .";
        b.append_last(term_ast.base).append_last("(");
        spirit::ast::name net_ast = cbag::util::parse_cdba_name(term_iter->second);
        b << net_ast.to_string(spirit::namespace_verilog{}) << ")";

        if (has_prev_term)
            stream.out_file << "," << std::endl;
        else
            has_prev_term = true;
        b.to_file(stream.out_file, spirit::namespace_verilog{}, false);
    }
}

void split_array_inst_nets(term_net_vec_t &term_net_vec, const std::string &inst_name,
                           cnt_t inst_size, const sch::instance &inst,
                           const std::vector<std::string> &terms) {
    for (const auto &term : terms) {
        auto term_iter = inst.connections.find(term);
        if (term_iter == inst.connections.end()) {
            throw std::invalid_argument(fmt::format(
                "Cannot find net connected to instance {} terminal {}", inst_name, term));
        }
        spirit::ast::name_unit ast_term = cbag::util::parse_cdba_name_unit(term);
        spirit::ast::name ast_net = cbag::util::parse_cdba_name(term_iter->second);

        std::vector<spirit::ast::name> net_vec;
        net_vec.reserve(inst_size);
        spirit::util::get_partition(ast_net, ast_term.size(), std::back_inserter(net_vec));
        term_net_vec.emplace_back(ast_term.base, std::move(net_vec));
    }
}

} // namespace verilog

verilog_stream::verilog_stream(const std::string &fname) : nstream_file(fname) {}

void traits::nstream<verilog_stream>::close(type &stream) { stream.close(); }

void traits::nstream<verilog_stream>::write_header(type &stream,
                                                   const std::vector<std::string> &inc_list,
                                                   bool shell) {
    if (!shell) {
        if (!inc_list.empty()) {
            for (auto const &fname : inc_list) {
                stream.out_file << "`include \"" << util::get_canonical_path(fname).c_str() << '"'
                                << std::endl;
            }
        }
    }
}

void traits::nstream<verilog_stream>::write_end(type &stream) {}

void traits::nstream<verilog_stream>::write_cv_header(type &stream, const std::string &name,
                                                      const sch::cellview_info &info, bool shell,
                                                      bool write_subckt) {
    auto def_type = std::string("wire");

    stream.out_file << std::endl << std::endl;

    // write module declaration
    lstream b;
    (b << "module" << name).append_last("(");
    b.to_file(stream.out_file, spirit::namespace_verilog{});

    bool has_prev_term = false;
    auto &term_net_attrs = info.term_net_attrs;
    verilog::write_cv_ports(stream, info.in_terms, has_prev_term, "    input ", term_net_attrs,
                            def_type);
    verilog::write_cv_ports(stream, info.out_terms, has_prev_term, "    output", term_net_attrs,
                            def_type);
    verilog::write_cv_ports(stream, info.io_terms, has_prev_term, "    inout ", term_net_attrs,
                            def_type);
    if (has_prev_term)
        stream.out_file << std::endl;
    stream.out_file << ");" << std::endl;

    if (!info.nets.empty() && !shell) {
        // write intermediate nets
        auto &logger = *get_cbag_logger();
        logger.info("writing verilog intermediate nets.");

        stream.out_file << std::endl;
        for (const auto &net_name : info.nets) {
            auto ast = util::parse_cdba_name_unit(net_name);

            logger.info("net: {}, base: {}, range: [{}:{}:{}], stop_include: {}", net_name,
                        ast.base, ast.idx_range.start, ast.idx_range.stop, ast.idx_range.step,
                        ast.idx_range.get_stop_include());

            auto type_str = sch::get_net_type(term_net_attrs, ast.base, def_type);
            stream.out_file << type_str;

            if (ast.is_vector()) {
                stream.out_file << fmt::format(" [{}:{}]", ast.idx_range.start,
                                               ast.idx_range.get_stop_include());
            }
            stream.out_file << ' ' << ast.base << ';' << std::endl;
        }
    }
}

void traits::nstream<verilog_stream>::write_cv_end(type &stream, const std::string &name,
                                                   bool write_subckt) {

    stream.out_file << std::endl << "endmodule" << std::endl;
}

void traits::nstream<verilog_stream>::write_instance(type &stream, const std::string &name,
                                                     const sch::instance &inst,
                                                     const sch::cellview_info &info) {
    if (inst.lib_name != "analogLib") {
        auto tag = spirit::namespace_verilog{};
        auto inst_ast = cbag::util::parse_cdba_name_unit(name);
        auto n = inst_ast.size();

        if (n == 1) {
            // normal instance, just write normally
            lstream b;
            stream.out_file << std::endl;
            b << inst.cell_name << name << "(";
            b.to_file(stream.out_file, tag);

            auto has_prev_term = false;
            verilog::append_inst_nets(stream, name, inst, info.in_terms, has_prev_term);
            verilog::append_inst_nets(stream, name, inst, info.out_terms, has_prev_term);
            verilog::append_inst_nets(stream, name, inst, info.io_terms, has_prev_term);
            if (has_prev_term)
                stream.out_file << std::endl;
            stream.out_file << ");" << std::endl;
        } else {
            // arrayed instance, need to split up nets
            auto term_net_vec = verilog::term_net_vec_t();
            verilog::split_array_inst_nets(term_net_vec, name, n, inst, info.in_terms);
            verilog::split_array_inst_nets(term_net_vec, name, n, inst, info.out_terms);
            verilog::split_array_inst_nets(term_net_vec, name, n, inst, info.io_terms);
            // write each instance
            auto stop = term_net_vec.end();
            auto last_check = stop - 1;
            for (decltype(n) inst_idx = 0; inst_idx < n; ++inst_idx) {
                lstream b;
                stream.out_file << std::endl;
                b << inst.cell_name << inst_ast.get_name_bit(inst_idx, true, tag) << "(";
                b.to_file(stream.out_file, tag);
                auto iter = term_net_vec.begin();
                for (; iter != stop; ++iter) {
                    lstream cur;
                    cur << "    .";
                    cur.append_last(iter->first);
                    cur.append_last("(");
                    cur << iter->second[inst_idx].to_string(tag);
                    if (iter == last_check)
                        cur << ")";
                    else
                        cur << "),";
                    cur.to_file(stream.out_file, tag);
                }
                stream.out_file << ");" << std::endl;
            }
        }
    }
}

void traits::nstream<verilog_stream>::append_netlist(type &stream, const std::string &netlist) {
    stream.out_file << std::endl << std::endl;
    stream.out_file << netlist;
}

} // namespace netlist
} // namespace cbag
