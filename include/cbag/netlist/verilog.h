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

#ifndef CBAG_NETLIST_VERILOG_H
#define CBAG_NETLIST_VERILOG_H

#include <string>
#include <unordered_set>

#include <cbag/enum/supply_wrap.h>
#include <cbag/netlist/core.h>
#include <cbag/netlist/nstream_output.h>

namespace cbag {
namespace netlist {

class verilog_stream : public nstream_output {
  public:
    static const std::string def_net_type;
    static const std::string sup_suffix;

    verilog_stream();

    verilog_stream(const std::string &fname);
};

template <> struct traits::nstream<verilog_stream> {
    using type = verilog_stream;

    static void close(type &stream);

    static void write_header(type &stream, const std::vector<std::string> &inc_list, bool shell);

    static void write_end(type &stream);

    static void write_cv_header(type &stream, const std::string &name,
                                const sch::cellview_info &info, bool shell, bool write_subckt,
                                bool write_declarations);

    static void write_cv_end(type &stream, const std::string &name, bool write_subckt);

    static void write_unit_instance(type &stream, const std::string &prefix, cnt_t inst_idx,
                                    const spirit::ast::name_unit &name_ast,
                                    const term_net_vec_t &conn_list, const param_map &params,
                                    const sch::cellview_info &info,
                                    const net_rename_map_t *net_map_ptr);

    static void append_netlist(type &stream, const std::string &netlist);

    static void write_supply_wrapper(type &stream, const std::string &name,
                                     const sch::cellview_info &info);

    static net_rename_map_t new_net_rename_map();
};

} // namespace netlist
} // namespace cbag

#endif // CBAG_NETLIST_VERILOG_H
