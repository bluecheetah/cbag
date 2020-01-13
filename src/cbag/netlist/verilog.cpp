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

#include <unordered_set>

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
#include <cbag/util/string.h>

namespace cbag {
namespace netlist {

const std::string verilog_stream::def_net_type = "wire";
const std::string verilog_stream::sup_suffix = "__w_sup";

// verilog netlisting helper functions
namespace verilog {

std::string get_net_type_str(const util::sorted_map<std::string, sch::attr_map_t> &term_net_attrs,
                             const std::string &base_name, bool is_port, bool is_inout = false) {
    auto attrs_iter = term_net_attrs.find(base_name);
    if (attrs_iter == term_net_attrs.end())
        return verilog_stream::def_net_type;

    auto &attrs = attrs_iter->second;
    auto type_iter = attrs.find("type");
    if (type_iter == attrs.end())
        return verilog_stream::def_net_type;

    auto &type_name = type_iter->second;
    if (type_name == "trireg") {
        if (is_port) {
            auto cap_size_iter = attrs.find("trireg_cap_large");
            if (cap_size_iter == attrs.end()) {
                return "trireg";
            }
            return (util::is_true(cap_size_iter->second)) ? "trireg (large)" : "trireg (small)";
        } else {
            return verilog_stream::def_net_type;
        }
    } else if (is_inout) {
        // inout can only be wire or trireg
        return verilog_stream::def_net_type;
    } else {
        return type_name;
    }
}

std::string get_unique_name(const std::unordered_set<std::string> &used_names,
                            const std::string &base) {
    auto ans = base;
    auto itere = used_names.end();
    for (auto idx = 1; used_names.find(ans) != itere; ++idx) {
        ans = fmt::format("{}_{}", base, idx);
    }
    return ans;
}

void write_cv_ports(verilog_stream &stream, const std::vector<std::string> &terms,
                    bool &has_prev_term, const char *prefix,
                    const util::sorted_map<std::string, sch::attr_map_t> &term_net_attrs,
                    const sch::cellview &cv,
                    std::vector<std::pair<std::string, std::string>> *wrap_conn_list_ptr,
                    const std::unordered_set<std::string> *used_names_ptr,
                    std::vector<std::pair<std::string, bool>> *sup_val_ptr, bool is_inout = false) {
    for (const auto &term : terms) {
        auto ast = util::parse_cdba_name_unit(term);

        bool write_term = true;
        if (wrap_conn_list_ptr) {
            auto stype = cv.get_sig_type(term);
            write_term = (stype != sig_type::power && stype != sig_type::ground);
            if (stype == sig_type::power) {
                auto val_name = get_unique_name(*used_names_ptr, ast.base + "_val");
                sup_val_ptr->emplace_back(val_name, true);
                wrap_conn_list_ptr->emplace_back(ast.base, val_name);
                write_term = false;
            } else if (stype == sig_type::ground) {
                auto val_name = get_unique_name(*used_names_ptr, ast.base + "_val");
                sup_val_ptr->emplace_back(val_name, false);
                wrap_conn_list_ptr->emplace_back(ast.base, val_name);
                write_term = false;
            } else {
                wrap_conn_list_ptr->emplace_back(ast.base, ast.base);
            }
        }
        if (!write_term)
            continue;

        lstream b;
        b << prefix << get_net_type_str(term_net_attrs, ast.base, true, is_inout);
        if (ast.is_vector()) {
            if (ast.idx_range.step != 1) {
                throw std::invalid_argument(
                    "Verilog does not support buses with non-unity step size: " + term);
            }
            b << fmt::format("[{}:{}]", ast.idx_range.start, ast.idx_range.get_stop_include());
        }
        b << ast.base;

        if (has_prev_term)
            stream << ",\n";
        else
            has_prev_term = true;
        b.to_file(stream, spirit::namespace_verilog{}, false);
    }
}

void write_cv_header(verilog_stream &stream, const std::string &name,
                     const sch::cellview_info &info, bool shell, bool write_subckt,
                     std::vector<std::pair<std::string, std::string>> *wrap_conn_list_ptr,
                     const std::unordered_set<std::string> *used_names_ptr,
                     std::vector<std::pair<std::string, bool>> *sup_val_ptr,
                     bool write_declarations) {
    if (write_declarations) {
        // when write_declarations is false, we just want the header string, so don't add
        // separation newlines.
        stream << "\n\n";
    }

    if (!info.cv_ptr) {
        throw std::runtime_error("cellview_info has null cv_ptr, cannot write netlist.");
    }
    auto &cv = *(info.cv_ptr);

    // write module declaration
    lstream b;
    (b << "module" << name).append_last("(");
    b.to_file(stream, spirit::namespace_verilog{});

    bool has_prev_term = false;
    auto &term_net_attrs = info.term_net_attrs;
    write_cv_ports(stream, info.in_terms, has_prev_term, "    input ", term_net_attrs, cv,
                   wrap_conn_list_ptr, used_names_ptr, sup_val_ptr);
    write_cv_ports(stream, info.out_terms, has_prev_term, "    output", term_net_attrs, cv,
                   wrap_conn_list_ptr, used_names_ptr, sup_val_ptr);
    write_cv_ports(stream, info.io_terms, has_prev_term, "    inout ", term_net_attrs, cv,
                   wrap_conn_list_ptr, used_names_ptr, sup_val_ptr, true);
    if (has_prev_term)
        stream << '\n';
    stream << ");";

    if (write_declarations) {
        // NOTE: only need a newline for previous line if we're writing declarations
        stream << '\n';
        if (!wrap_conn_list_ptr && !info.nets.empty() && !shell) {
            // write intermediate nets
            auto &logger = *get_cbag_logger();
            logger.info("writing verilog intermediate nets.");

            stream << '\n';
            for (const auto &net_name : info.nets) {
                auto ast = util::parse_cdba_name_unit(net_name);

                logger.info("net: {}, base: {}, range: [{}:{}:{}]", net_name, ast.base,
                            ast.idx_range.start, ast.idx_range.stop, ast.idx_range.step);

                stream << get_net_type_str(term_net_attrs, ast.base, false);

                if (ast.is_vector()) {
                    auto start = ast.idx_range.start;
                    auto stop = ast.idx_range.get_stop_include();
                    stream << fmt::format(" [{}:{}]", std::max(start, stop), std::min(start, stop));
                }
                stream << ' ' << ast.base << ";\n";
            }
        }
    }
}

void write_unit_instance(verilog_stream &stream, cnt_t inst_idx, const std::string &name,
                         const std::vector<std::pair<std::string, std::string>> &conn_list,
                         const sch::cellview_info &info) {

    auto tag = spirit::namespace_verilog{};

    auto b = lstream();
    stream << '\n';
    b << info.cell_name << name << "(";
    b.to_file(stream, tag);

    auto iter = conn_list.begin();
    auto stop = conn_list.end();
    auto last_check = stop - 1;
    for (; iter != stop; ++iter) {
        auto cur = lstream();
        cur << "    .";
        cur.append_last(iter->first);
        cur.append_last("(");
        cur << iter->second;
        if (iter == last_check)
            cur << ")";
        else
            cur << "),";
        cur.to_file(stream, tag);
    }
    stream << ");\n";
}

} // namespace verilog

verilog_stream::verilog_stream() : nstream_output() {}

verilog_stream::verilog_stream(const std::string &fname) : nstream_output(fname) {}

void traits::nstream<verilog_stream>::close(type &stream) { stream.close(); }

void traits::nstream<verilog_stream>::write_header(type &stream,
                                                   const std::vector<std::string> &inc_list,
                                                   bool shell) {
    if (!shell) {
        if (!inc_list.empty()) {
            for (auto const &fname : inc_list) {
                stream << "`include \"" << util::get_canonical_path(fname).c_str() << "\"\n";
            }
        }
        // TODO: hack
        stream << "`timescale 1ps/1ps \n";
    }
}

void traits::nstream<verilog_stream>::write_end(type &stream) {}

void traits::nstream<verilog_stream>::write_cv_header(type &stream, const std::string &name,
                                                      const sch::cellview_info &info, bool shell,
                                                      bool write_subckt, bool write_declarations) {
    verilog::write_cv_header(stream, name, info, shell, write_subckt, nullptr, nullptr, nullptr,
                             write_declarations);
}

void traits::nstream<verilog_stream>::write_cv_end(type &stream, const std::string &name,
                                                   bool write_subckt) {

    stream << "\nendmodule\n";
}

void traits::nstream<verilog_stream>::write_unit_instance(
    type &stream, const std::string &prefix, cnt_t inst_idx, const spirit::ast::name_unit &name_ast,
    const term_net_vec_t &conn_list, const param_map &params, const sch::cellview_info &info,
    const net_rename_map_t *net_map_ptr) {
    if ((net_map_ptr && !net_map_ptr->empty()) || !prefix.empty()) {
        throw std::runtime_error("Verilog netlister does not support flatten netlist yet.");
    }
    if (info.lib_name != "analogLib") {
        auto tag = spirit::namespace_verilog{};
        auto name = name_ast[inst_idx].to_string(true, tag);

        std::vector<std::pair<std::string, std::string>> conn_map;
        conn_map.reserve(conn_list.size());
        for (const auto & [ nu, name_list ] : conn_list) {
            conn_map.emplace_back(nu.base, name_list[inst_idx].to_string(tag));
        }

        verilog::write_unit_instance(stream, inst_idx, name, conn_map, info);
    }
}

void traits::nstream<verilog_stream>::append_netlist(type &stream, const std::string &netlist) {
    stream << "\n\n";
    stream << netlist;
}

void traits::nstream<verilog_stream>::write_supply_wrapper(type &stream, const std::string &name,
                                                           const sch::cellview_info &info) {
    if (!util::endswith(name, verilog_stream::sup_suffix)) {
        throw std::runtime_error("supply wrapping cell name must end in " +
                                 verilog_stream::sup_suffix);
    }

    // get all used base names
    auto used_base_names = std::unordered_set<std::string>();
    for (const auto &term : info.in_terms) {
        used_base_names.emplace(util::parse_cdba_name_unit(term).base);
    }
    for (const auto &term : info.out_terms) {
        used_base_names.emplace(util::parse_cdba_name_unit(term).base);
    }
    for (const auto &term : info.io_terms) {
        used_base_names.emplace(util::parse_cdba_name_unit(term).base);
    }

    auto sup_val = std::vector<std::pair<std::string, bool>>();
    auto wrap_name = name.substr(0, name.length() - verilog_stream::sup_suffix.length());
    auto wrap_conn_list = std::vector<std::pair<std::string, std::string>>();
    verilog::write_cv_header(stream, wrap_name, info, false, true, &wrap_conn_list,
                             &used_base_names, &sup_val, true);

    stream << '\n';
    for (const auto & [ val_name, is_pwr ] : sup_val) {
        stream << "wire " << val_name << ";\n";
    }
    stream << '\n';
    for (const auto & [ val_name, is_pwr ] : sup_val) {
        stream << "assign " << val_name << " = " << ((is_pwr) ? "1'b1;\n" : "1'b0;\n");
    }

    verilog::write_unit_instance(stream, 0, "XDUT", wrap_conn_list, info);
    traits::nstream<verilog_stream>::write_cv_end(stream, wrap_name, true);
}

net_rename_map_t traits::nstream<verilog_stream>::new_net_rename_map() {
    return net_rename_map_t();
}

} // namespace netlist
} // namespace cbag
