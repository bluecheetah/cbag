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

#ifndef CBAG_NETLIST_NETLIST_H
#define CBAG_NETLIST_NETLIST_H

#include <sstream>
#include <string>
#include <unordered_set>
#include <vector>

#include <cbag/enum/design_output.h>
#include <cbag/enum/supply_wrap.h>
#include <cbag/logging/logging.h>
#include <cbag/netlist/cdl.h>
#include <cbag/netlist/core.h>
#include <cbag/netlist/spectre.h>
#include <cbag/netlist/verilog.h>
#include <cbag/schematic/cellview_info.h>
#include <cbag/util/io.h>

namespace cbag {
namespace netlist {

template <class N> using IsNetlister = typename traits::nstream<N>::type;

void read_prim_info(const std::string &prim_fname, std::vector<std::string> &inc_list,
                    sch::netlist_map_t &netlist_map, std::string &append_file,
                    design_output out_type);

bool gen_supply_wrapper(const std::string &name, supply_wrap sup_mode,
                        const std::unordered_set<std::string> &top_set);

template <class ContentList, class N, typename = IsNetlister<N>>
void write_netlist_helper(const ContentList &name_cv_list, N &&stream, bool flat, bool shell,
                          bool top_subckt, supply_wrap sup_mode,
                          const std::unordered_set<std::string> &top_set,
                          sch::netlist_map_t &netlist_map, const std::vector<std::string> &inc_list,
                          const std::string &append_file, spdlog::logger &logger) {

    traits::nstream<N>::write_header(stream, inc_list, shell);

    if (!shell && !append_file.empty()) {
        if (util::is_file(append_file)) {
            std::ifstream text(append_file);
            std::stringstream buffer;
            buffer << text.rdbuf();
            traits::nstream<N>::append_netlist(stream, buffer.str());
        } else {
            throw std::invalid_argument(
                fmt::format("Cannot find primitives append file: {}", append_file));
        }
    }

    auto stop = name_cv_list.end();
    if (flat && !shell) {
        // flatten netlist
        // first, populate cv_info map
        for (auto iter = name_cv_list.begin(); iter != stop; ++iter) {
            auto & [ cur_name, cv_netlist_pair ] = *iter;
            auto & [ cv_ptr, cur_netlist ] = cv_netlist_pair;
            if (cv_ptr) {
                // NOTE: we don't have to compute net attributes if a custom netlist is given
                auto cv_info =
                    sch::get_cv_netlist_info(*cv_ptr, cur_name, netlist_map, cur_netlist.empty());
                logger.info("Adding cellview {}__{} to netlist cell map", cv_info.lib_name,
                            cur_name);
                sch::record_cv_info(netlist_map, std::string(cv_ptr->cell_name),
                                    std::move(cv_info));
            }
        }
        // then, just netlist the top cellview
        auto iter = std::next(name_cv_list.begin(), name_cv_list.size() - 1);
        auto & [ cur_name, cv_netlist_pair ] = *iter;
        auto & [ cv_ptr, cur_netlist ] = cv_netlist_pair;
        if (cv_ptr) {
            auto &cv_info = get_cv_info(netlist_map, cv_ptr->lib_name, cv_ptr->cell_name);
            if (cur_netlist.empty()) {
                add_cellview(stream, cur_name, cv_info, netlist_map, false, top_subckt, true);
            } else {
                add_cellview(stream, cur_netlist);
            }
            if (gen_supply_wrapper(cur_name, sup_mode, top_set)) {
                add_supply_wrapper(stream, cur_name, cv_info);
            }
        }
    } else {
        // normal hierarchical netlist
        auto last_idx = name_cv_list.size() - 1;
        decltype(name_cv_list.size()) idx = 0;
        for (auto iter = name_cv_list.begin(); iter != stop; ++iter, ++idx) {
            auto is_top = (idx == last_idx);
            auto write_subckt = top_subckt || !is_top;
            auto & [ cur_name, cv_netlist_pair ] = *iter;
            auto & [ cv_ptr, cur_netlist ] = cv_netlist_pair;
            if (cv_ptr) {
                logger.info("Netlisting cellview: {}__{}", cv_ptr->lib_name, cur_name);
                // NOTE: we don't have to compute net attributes if a custom netlist is given
                auto cv_info =
                    sch::get_cv_netlist_info(*cv_ptr, cur_name, netlist_map, cur_netlist.empty());

                if (!shell || is_top) {
                    if (cur_netlist.empty()) {
                        // add this cellview to netlist
                        add_cellview(stream, cur_name, cv_info, netlist_map, shell, write_subckt,
                                     false);
                    } else {
                        add_cellview(stream, cur_netlist);
                    }
                    if (gen_supply_wrapper(cur_name, sup_mode, top_set)) {
                        add_supply_wrapper(stream, cur_name, cv_info);
                    }
                }

                // add this cellview to netlist map
                logger.info("Adding cellview {}__{} to netlist cell map", cv_info.lib_name,
                            cur_name);
                sch::record_cv_info(netlist_map, std::string(cv_ptr->cell_name),
                                    std::move(cv_info));
            }
        }
    }
    // build final netlist
    traits::nstream<N>::write_end(stream);
    traits::nstream<N>::close(stream);
}

template <class ContentList>
void write_netlist(const ContentList &name_cv_list, const std::unordered_set<std::string> &top_set,
                   const std::string &fname, design_output format, sch::netlist_map_t &netlist_map,
                   const std::string &append_file, const std::vector<std::string> &inc_list,
                   bool flat, bool shell, bool top_subckt, bool square_bracket, cnt_t rmin,
                   cnt_t precision, supply_wrap sup_mode) {
    auto logger = cbag::get_cbag_logger();

    if (top_set.size() > 1) {
        if (!top_subckt)
            throw std::invalid_argument(
                "can only have top_subckt = false if there's only one top cell.");
        if (shell)
            throw std::invalid_argument("can only have shell = true if there's only one top cell.");
        if (flat)
            throw std::invalid_argument("can only have flat = true if there's only one top cell.");
    }

    switch (format) {
    case design_output::CDL:
        logger->info("Writing CDL netlist: {}", fname);
        write_netlist_helper(name_cv_list, cdl_stream(fname, square_bracket, rmin, precision), flat,
                             shell, top_subckt, sup_mode, top_set, netlist_map, inc_list,
                             append_file, *logger);
        break;
    case design_output::SPECTRE:
        logger->info("Writing Spectre netlist: {}", fname);
        write_netlist_helper(name_cv_list, spectre_stream(fname, precision), flat, shell,
                             top_subckt, sup_mode, top_set, netlist_map, inc_list, append_file,
                             *logger);
        break;
    case design_output::VERILOG:
        logger->info("Writing Verilog netlist: {}", fname);
        write_netlist_helper(name_cv_list, verilog_stream(fname), flat, shell, top_subckt, sup_mode,
                             top_set, netlist_map, inc_list, append_file, *logger);
        break;
    case design_output::SYSVERILOG:
        logger->info("Writing System Verilog netlist: {}", fname);
        write_netlist_helper(name_cv_list, verilog_stream(fname), flat, shell, top_subckt, sup_mode,
                             top_set, netlist_map, inc_list, append_file, *logger);
        break;
    default:
        throw std::invalid_argument(
            fmt::format("Unrecognized design output code: {}", static_cast<enum_t>(format)));
    }
}

std::string get_cv_header(const sch::cellview &cv, const std::string &cell_name,
                          design_output format);

} // namespace netlist
} // namespace cbag
#endif // CBAG_CBAG_FWD_H
