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

#ifndef CBAG_SCHEMATIC_CELLVIEW_FWD_H
#define CBAG_SCHEMATIC_CELLVIEW_FWD_H

#include <memory>
#include <vector>

#include <cbag/util/sorted_map.h>

#include <cbag/common/box_t.h>
#include <cbag/common/param_map.h>
#include <cbag/common/typedefs.h>
#include <cbag/enum/sig_type.h>
#include <cbag/schematic/shape_t.h>
#include <cbag/schematic/term_t.h>

namespace cbag {
namespace sch {

struct instance;

using inst_map_t = cbag::util::sorted_map<std::string, std::unique_ptr<instance>>;
using conn_list_t = std::vector<std::pair<std::string, std::string>>;

/** A schematic or symbol cell view
 */
struct cellview {
  public:
    std::string lib_name;
    std::string cell_name;
    std::string view_name;
    box_t bbox;
    term_t terminals;
    std::vector<shape_t> shapes;
    inst_map_t instances;
    param_map props;
    param_map app_defs;
    std::unique_ptr<cellview> sym_ptr = nullptr;

    cellview();

    cellview(const std::string &yaml_fname, const std::string &sym_view = "");

    cellview(std::string lib_name, std::string cell_name, std::string view_name, coord_t xl,
             coord_t yl, coord_t xh, coord_t yh);

    void to_file(const std::string &fname) const;

    std::unique_ptr<cellview> get_copy() const;

    sig_type get_sig_type(const std::string &term_name) const;

    void clear_params();

    void set_param(std::string name, const param_t &val);

    void rename_pin(const std::string &old_name, const std::string &new_name,
                    bool is_symbol = false);

    void add_pin(const std::string &new_name, enum_t term_type_code, enum_t sig_type_code,
                 bool is_symbol = false);

    void set_pin_attribute(const std::string &pin_name, const std::string &key,
                           const std::string &val);

    bool remove_pin(const std::string &name, bool is_smybol = false);

    void rename_instance(const std::string &old_name, std::string new_name);

    bool remove_instance(const std::string &name);
};

} // namespace sch
} // namespace cbag

#endif
