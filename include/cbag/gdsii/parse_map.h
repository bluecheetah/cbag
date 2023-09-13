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

#ifndef CBAG_GDSII_PARSE_MAP_H
#define CBAG_GDSII_PARSE_MAP_H

#include <string>

#include <cbag/enum/blockage_type.h>
#include <cbag/enum/boundary_type.h>
#include <cbag/layout/tech.h>
#include <cbag/util/io.h>
#include <cbag/util/string.h>

namespace cbag {
namespace gdsii {

void check_has_next(const util::token_iterator &iter, const std::string &fname, int count);

uint16_t to_int(std::string &&s, const std::string &fname, int count);

template <typename F>
void process_layer_map(const std::string &fname, const layout::tech &tech, F fun) {
    auto file = util::open_file_read(fname);
    std::string line;
    auto count = 1;
    while (std::getline(file, line)) {
        // ignore comments
        if (!line.empty() && line[0] != '#') {
            util::token_iterator iter(line, " \t");
            if (iter.has_next()) {
                // the line has non-white space content
                auto val1 = iter.get_next();
                check_has_next(iter, fname, count);
                auto val2 = iter.get_next();
                check_has_next(iter, fname, count);
                auto glay = to_int(iter.get_next(), fname, count);
                check_has_next(iter, fname, count);
                auto gpurp = to_int(iter.get_next(), fname, count);
                // if we still have values left, that means they are
                // color specifications.  Ignore those entries since
                // we use MnCA/MnCB layers to denote colors.
                auto lay = tech.get_layer_id(val1);
                auto purp = tech.get_purpose_id(val2);
                if (lay && purp) {
                    fun(std::make_pair(*lay, *purp), std::make_pair(glay, gpurp));
                }
            }
        }
        ++count;
    }
}

template <typename F1, typename F2>
void process_object_map(const std::string &fname, const layout::tech &tech, F1 bnd_fun,
                        F2 blk_fun) {

    auto file = util::open_file_read(fname);
    std::string line;
    auto count = 1;
    while (std::getline(file, line)) {
        // ignore comments
        if (!line.empty() && line[0] != '#') {
            util::token_iterator iter(line, " \t");
            if (iter.has_next()) {
                // the line has non-white space content
                auto obj_type = iter.get_next();
                if (obj_type == "Boundary") {
                    check_has_next(iter, fname, count);
                    auto bnd_type = iter.get_next();
                    check_has_next(iter, fname, count);
                    auto glay = to_int(iter.get_next(), fname, count);
                    check_has_next(iter, fname, count);
                    auto gpurp = to_int(iter.get_next(), fname, count);

                    if (bnd_type == "PR") {
                        bnd_fun(boundary_type::PR, std::make_pair(glay, gpurp));
                    } else if (bnd_type == "snap") {
                        bnd_fun(boundary_type::snap, std::make_pair(glay, gpurp));
                    }
                } else if (obj_type == "layerBlockage") {
                    check_has_next(iter, fname, count);
                    auto blk_type = iter.get_next();
                    check_has_next(iter, fname, count);
                    auto lay_name = iter.get_next();
                    check_has_next(iter, fname, count);
                    auto glay = to_int(iter.get_next(), fname, count);
                    check_has_next(iter, fname, count);
                    auto gpurp = to_int(iter.get_next(), fname, count);

                    auto oa_lay_id = tech.get_layer_id(lay_name);
                    if (oa_lay_id) {
                        if (blk_type == "routing") {
                            blk_fun(std::make_pair(*oa_lay_id, blockage_type::routing),
                                    std::make_pair(glay, gpurp));
                        } else if (blk_type == "fill") {
                            blk_fun(std::make_pair(*oa_lay_id, blockage_type::fill),
                                    std::make_pair(glay, gpurp));
                        }
                    }
                }
            }
        }
        ++count;
    }
}

} // namespace gdsii
} // namespace cbag

#endif
