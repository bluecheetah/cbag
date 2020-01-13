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

#include <map>
#include <utility>
#include <variant>

#include <fmt/core.h>

#include <cbag/spirit/ast.h>

#include <cbag/schematic/cellview.h>
#include <cbag/schematic/cellview_info.h>
#include <cbag/schematic/instance.h>

#include <cbag/netlist/lstream.h>
#include <cbag/netlist/spectre.h>
#include <cbag/spirit/util.h>
#include <cbag/util/io.h>
#include <cbag/util/name_convert.h>

namespace cbag {
namespace netlist {

// spectre netlisting helper functions
namespace spectre {

using term_net_vec_t = std::vector<std::pair<cnt_t, std::vector<std::string>>>;

void get_cv_pins(lstream &b, const std::vector<std::string> &names) {
    for (auto const &name : names) {
        spirit::ast::name_unit ast = cbag::util::parse_cdba_name_unit(name);
        auto n = ast.size();
        for (decltype(n) idx = 0; idx < n; ++idx) {
            std::string tmp = ast[idx].to_string(false, spirit::namespace_spectre{});
            b << tmp;
        }
    }
}

template <class OutIter>
void write_instance_cell_name(OutIter &&iter, const param_map &params,
                              const sch::cellview_info &info, spectre_stream &stream) {
    auto &cur_cell_name = (info.lib_name == "analogLib" || info.lib_name == "basic")
                              ? stream.get_cell_name(info.cell_name)
                              : info.cell_name;
    *iter = cur_cell_name;

    // get default parameter values
    param_map par_map(info.props);
    // update with instance parameters
    for (auto const & [ key, val ] : params) {
        par_map.insert_or_assign(key, val);
    }

    // write instance parameters
    auto precision = stream.precision();
    if (info.lib_name == "analogLib") {
        for (auto const & [ key, val ] : par_map) {
            auto &prop_name = stream.get_prop_name(cur_cell_name, key);
            if (!prop_name.empty()) {
                std::visit(write_param_visitor(iter, prop_name, precision), val);
            }
        }
    } else {
        for (auto const & [ key, val ] : par_map) {
            std::visit(write_param_visitor(iter, key, precision), val);
        }
    }
}

std::unordered_map<std::string, std::string> new_cell_name_map() {
    // map CDF cell names to Spectre cell names
    auto cell_map = std::unordered_map<std::string, std::string>();
    cell_map["cap"] = "capacitor";
    cell_map["cds_thru"] = "iprobe";
    cell_map["idc"] = "isource";
    cell_map["ind"] = "inductor";
    cell_map["ipulse"] = "isource";
    cell_map["isin"] = "isource";
    cell_map["res"] = "resistor";
    cell_map["switch"] = "relay";
    cell_map["vdc"] = "vsource";
    cell_map["vpulse"] = "vsource";
    cell_map["vpwlf"] = "vsource";
    cell_map["vsin"] = "vsource";
    return cell_map;
}

std::unordered_map<std::string, std::string> new_prop_name_map() {
    // map CDF properties to Spectre properties
    auto prop_map = std::unordered_map<std::string, std::string>();
    prop_map["acm"] = "mag";
    prop_map["acp"] = "phase";
    prop_map["egain"] = "gain";
    prop_map["fgain"] = "gain";
    prop_map["fileName"] = "file";
    prop_map["ggain"] = "gm";
    prop_map["hgain"] = "rm";
    prop_map["i1"] = "val0";
    prop_map["i2"] = "val1";
    prop_map["ia"] = "ampl";
    prop_map["idc"] = "dc";
    prop_map["maxm"] = "max";
    prop_map["minm"] = "min";
    prop_map["pacm"] = "pacmag";
    prop_map["pacp"] = "pacphase";
    prop_map["per"] = "period";
    prop_map["pw"] = "width";
    prop_map["rc"] = "rclosed";
    prop_map["ro"] = "ropen";
    prop_map["srcType"] = "type";
    prop_map["td"] = "delay";
    prop_map["tf"] = "fall";
    prop_map["tr"] = "rise";
    prop_map["v1"] = "val0";
    prop_map["v2"] = "val1";
    prop_map["va"] = "ampl";
    prop_map["vdc"] = "dc";
    prop_map["vref"] = "probe";
    prop_map["xfm"] = "xfmag";
    return prop_map;
}

std::unordered_map<std::string, std::unordered_map<std::string, std::string>> new_cell_prop_map() {
    // port specific parameters
    auto cp_map = std::unordered_map<std::string, std::unordered_map<std::string, std::string>>();
    auto prop_map = std::unordered_map<std::string, std::string>();
    prop_map["filenums"] = "";
    prop_map["pm"] = "";
    cp_map.emplace("port", std::move(prop_map));
    return cp_map;
}

} // namespace spectre

spectre_stream::spectre_stream()
    : nstream_output(), cell_name_map_(spectre::new_cell_name_map()),
      prop_name_map_(spectre::new_prop_name_map()), cell_prop_map_(spectre::new_cell_prop_map()) {}

spectre_stream::spectre_stream(const std::string &fname, cnt_t precision)
    : nstream_output(fname), precision_(precision), cell_name_map_(spectre::new_cell_name_map()),
      prop_name_map_(spectre::new_prop_name_map()), cell_prop_map_(spectre::new_cell_prop_map()) {}

const std::string &spectre_stream::get_cell_name(const std::string &cell_name) const {
    auto iter = cell_name_map_.find(cell_name);
    return (iter == cell_name_map_.end()) ? cell_name : iter->second;
}

const std::string &spectre_stream::get_prop_name(const std::string &cell_name,
                                                 const std::string &prop_name) const {
    auto map_iter = cell_prop_map_.find(cell_name);
    if (map_iter != cell_prop_map_.end()) {
        auto &map_ref = map_iter->second;
        auto iter = map_ref.find(prop_name);
        if (iter != map_ref.end())
            return iter->second;
    }
    auto iter = prop_name_map_.find(prop_name);
    return (iter == prop_name_map_.end()) ? prop_name : iter->second;
}

cnt_t spectre_stream::precision() const noexcept { return precision_; }

void traits::nstream<spectre_stream>::close(type &stream) { stream.close(); }

void traits::nstream<spectre_stream>::write_header(type &stream,
                                                   const std::vector<std::string> &inc_list,
                                                   bool shell) {
    // set language
    stream << "simulator lang=spectre\n";

    if (!shell) {
        if (!inc_list.empty()) {
            for (auto const &fname : inc_list) {
                stream << "include \"" << util::get_canonical_path(fname).c_str() << "\"\n";
            }
            stream << '\n';
        }
    }
}

void traits::nstream<spectre_stream>::write_end(type &stream) {}

void traits::nstream<spectre_stream>::write_cv_header(type &stream, const std::string &name,
                                                      const sch::cellview_info &info, bool shell,
                                                      bool write_subckt, bool write_declarations) {
    stream << "\n\n";
    if (write_subckt) {
        lstream b;
        b << "subckt";
        b << name;
        spectre::get_cv_pins(b, info.in_terms);
        spectre::get_cv_pins(b, info.out_terms);
        spectre::get_cv_pins(b, info.io_terms);

        // write definition line
        b.to_file(stream, spirit::namespace_spectre{});
    }
}

void traits::nstream<spectre_stream>::write_cv_end(type &stream, const std::string &name,
                                                   bool write_subckt) {
    if (write_subckt) {
        stream << "ends " << name << '\n';
    } else {
        stream << '\n';
    }
}

void traits::nstream<spectre_stream>::write_unit_instance(
    type &stream, const std::string &prefix, cnt_t inst_idx, const spirit::ast::name_unit &name_ast,
    const term_net_vec_t &conn_list, const param_map &params, const sch::cellview_info &info,
    const net_rename_map_t *net_map_ptr) {

    auto tag = spirit::namespace_spectre{};
    auto name = name_ast[inst_idx].to_string(true, tag);

    auto b = lstream();
    // write instance name
    b << (prefix + name);

    // write instance nets
    for (const auto & [ term_ast, net_ast_list ] : conn_list) {
        spirit::util::get_name_bits(net_ast_list[inst_idx],
                                    b.get_bit_inserter(prefix, net_map_ptr, tag));
    }
    // write instance cell name
    spectre::write_instance_cell_name(b.get_back_inserter(), params, info, stream);
    b.to_file(stream, spirit::namespace_cdba{});
}

void traits::nstream<spectre_stream>::append_netlist(type &stream, const std::string &netlist) {
    stream << "\n\n";
    stream << netlist;
}

void traits::nstream<spectre_stream>::write_supply_wrapper(type &stream, const std::string &name,
                                                           const sch::cellview_info &info) {
    throw std::runtime_error("supply wrapping not supporting in spectre netlist.");
}

net_rename_map_t traits::nstream<spectre_stream>::new_net_rename_map() {
    auto ans = net_rename_map_t();
    ans.emplace(spirit::ast::name_bit("gnd!"), spirit::ast::name_bit("0"));
    return ans;
}

} // namespace netlist
} // namespace cbag
