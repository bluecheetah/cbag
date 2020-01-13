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

#include <utility>
#include <variant>

#include <fmt/core.h>

#include <cbag/spirit/ast.h>

#include <cbag/schematic/cellview.h>
#include <cbag/schematic/cellview_info.h>
#include <cbag/schematic/instance.h>

#include <cbag/netlist/cdl.h>
#include <cbag/netlist/lstream.h>
#include <cbag/spirit/util.h>
#include <cbag/util/io.h>
#include <cbag/util/name_convert.h>

namespace cbag {
namespace netlist {

// cdl netlisting helper functions
namespace cdl {

template <typename NS>
void get_cv_term_bits(lstream &b, lstream &b2, const std::vector<std::string> &names,
                      const std::string &term_type, NS tag) {
    for (auto const &name : names) {
        spirit::ast::name_unit ast = cbag::util::parse_cdba_name_unit(name);
        auto n = ast.size();
        for (decltype(n) idx = 0; idx < n; ++idx) {
            std::string tmp = ast[idx].to_string(false, tag);
            b << tmp;
            tmp.append(term_type);
            b2 << std::move(tmp);
        }
    }
}

template <class OutIter>
void write_instance_cell_name(OutIter &&iter, const param_map &params,
                              const sch::cellview_info &info, cnt_t precision) {
    if (info.cell_name == "cds_thru") {
        *iter = "0 $[SH]";
    } else {
        *iter = "/";
        *iter = info.cell_name;
    }

    // get default parameter values
    param_map par_map(info.props);
    // update with instance parameters
    for (auto const & [ key, val ] : params) {
        par_map.insert_or_assign(key, val);
    }
    // write instance parameters
    for (auto const & [ key, val ] : par_map) {
        std::visit(write_param_visitor(iter, key, precision), val);
    }
}

template <typename NS>
void write_unit_instance_helper(lstream &b, const std::string &prefix, cnt_t inst_idx,
                                const term_net_vec_t &conn_list,
                                const net_rename_map_t *net_map_ptr, NS tag) {
    // write instance nets
    for (const auto & [ term_ast, net_ast_list ] : conn_list) {
        spirit::util::get_name_bits(net_ast_list[inst_idx],
                                    b.get_bit_inserter(prefix, net_map_ptr, tag));
    }
}

} // namespace cdl

cdl_stream::cdl_stream() : nstream_output(){};

cdl_stream::cdl_stream(const std::string &fname, bool square_bracket, cnt_t rmin, cnt_t precision)
    : nstream_output(fname), rmin_(rmin), precision_(precision), square_bracket_(square_bracket) {}

bool cdl_stream::square_bracket() const noexcept { return square_bracket_; }

cnt_t cdl_stream::rmin() const noexcept { return rmin_; }

cnt_t cdl_stream::precision() const noexcept { return precision_; }

void traits::nstream<cdl_stream>::close(type &stream) { stream.close(); }

void traits::nstream<cdl_stream>::write_header(type &stream,
                                               const std::vector<std::string> &inc_list,
                                               bool shell) {
    if (!shell) {
        if (!inc_list.empty()) {
            for (auto const &fname : inc_list) {
                stream << ".INCLUDE " << util::get_canonical_path(fname).c_str() << '\n';
            }
            stream << '\n';
        }
    }
    // write CDL control commands
    // keep analog devices (BJT)
    stream << "*.BIPOLAR\n";
    // minimum resistor value: resistors below this value are shorted
    stream << "*.RESI = " << stream.rmin() << '\n';
    // transistor width/lengths interpreted as meters
    stream << "*.SCALE METER\n";
    // these commands are printed by CDL export, but
    // ignored by LVS usually.
    stream << "*.MEGA\n";
    stream << "*.RESVAL\n";
    stream << "*.CAPVAL\n";
    stream << "*.DIOPERI\n";
    stream << "*.DIOAREA\n";
    stream << "*.EQUATION\n";

    // write SPICE control commands
    // global parameters definitions
    stream << ".PARAM\n";
}

void traits::nstream<cdl_stream>::write_end(type &stream) {}

void traits::nstream<cdl_stream>::write_cv_header(type &stream, const std::string &name,
                                                  const sch::cellview_info &info, bool shell,
                                                  bool write_subckt, bool write_declarations) {
    lstream b;
    stream << "\n\n";
    b << ".SUBCKT";
    b << name;
    lstream b2;
    b2 << "*.PININFO";
    if (stream.square_bracket()) {
        auto tag = spirit::namespace_verilog{};
        cdl::get_cv_term_bits(b, b2, info.in_terms, ":I", tag);
        cdl::get_cv_term_bits(b, b2, info.out_terms, ":O", tag);
        cdl::get_cv_term_bits(b, b2, info.io_terms, ":B", tag);
    } else {
        auto tag = spirit::namespace_cdba{};
        cdl::get_cv_term_bits(b, b2, info.in_terms, ":I", tag);
        cdl::get_cv_term_bits(b, b2, info.out_terms, ":O", tag);
        cdl::get_cv_term_bits(b, b2, info.io_terms, ":B", tag);
    }

    // write definition line
    b.to_file(stream, spirit::namespace_cdba{});
    // write pin information line
    if (write_declarations)
        b2.to_file(stream, spirit::namespace_cdl_cmd{});
}

void traits::nstream<cdl_stream>::write_cv_end(type &stream, const std::string &name,
                                               bool write_subckt) {
    stream << ".ENDS\n";
}

void traits::nstream<cdl_stream>::write_unit_instance(
    type &stream, const std::string &prefix, cnt_t inst_idx, const spirit::ast::name_unit &name_ast,
    const term_net_vec_t &conn_list, const param_map &params, const sch::cellview_info &info,
    const net_rename_map_t *net_map_ptr) {
    if (info.lib_name != "analogLib") {
        auto name = name_ast[inst_idx].to_string(true, spirit::namespace_cdba{});

        auto b = lstream();
        // write instance name
        if (info.cell_name == "cds_thru") {
            b << "R" + prefix + name;
        } else {
            b << prefix + name;
        }

        if (stream.square_bracket()) {
            cdl::write_unit_instance_helper(b, prefix, inst_idx, conn_list, net_map_ptr,
                                            spirit::namespace_verilog{});
        } else {
            cdl::write_unit_instance_helper(b, prefix, inst_idx, conn_list, net_map_ptr,
                                            spirit::namespace_cdba{});
        }
        // write instance cell name
        cdl::write_instance_cell_name(b.get_back_inserter(), params, info, stream.precision());
        b.to_file(stream, spirit::namespace_cdba{});
    }
}

void traits::nstream<cdl_stream>::append_netlist(type &stream, const std::string &netlist) {
    stream << "\n\n";
    stream << netlist;
}

void traits::nstream<cdl_stream>::write_supply_wrapper(type &stream, const std::string &name,
                                                       const sch::cellview_info &info) {
    throw std::runtime_error("supply wrapping not supporting in CDL netlist.");
}

net_rename_map_t traits::nstream<cdl_stream>::new_net_rename_map() { return net_rename_map_t(); }

} // namespace netlist
} // namespace cbag
