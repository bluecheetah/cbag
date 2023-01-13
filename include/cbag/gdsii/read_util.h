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

#ifndef CBAG_GDSII_READ_UTIL_H
#define CBAG_GDSII_READ_UTIL_H

#include <fstream>
#include <string>
#include <tuple>
#include <unordered_map>

#include <cbag/common/box_t.h>
#include <cbag/common/layer_t.h>
#include <cbag/common/point_t.h>
#include <cbag/common/transformation.h>
#include <cbag/common/typedefs.h>
#include <cbag/gdsii/record_type.h>
#include <cbag/gdsii/typedefs.h>
#include <cbag/layout/polygons.h>
#include <cbag/logging/logging.h>
#include <cbag/util/sfinae.h>

namespace cbag {

namespace layout {
class instance;
class cellview;
} // namespace layout

namespace gdsii {

std::tuple<record_type, std::size_t> read_record_header(std::istream &stream);

template <typename T, util::IsInt<T> = 0> T read_bytes(std::istream &stream) {
    constexpr auto unit_size = sizeof(T);
    auto ans = static_cast<T>(0);
    for (std::size_t bidx = 0, shft = (unit_size - 1) * 8; bidx < unit_size; ++bidx, shft -= 8) {
        auto tmp = static_cast<T>(stream.get());
        ans |= tmp << shft;
    }
    return ans;
}

template <record_type R, std::size_t unit_size = 1, std::size_t num_data = 0>
std::size_t check_record_header(std::istream &stream) {
    auto[actual, size] = read_record_header(stream);
    if (actual != R)
        throw std::runtime_error(
            fmt::format("got gds record {}, expected {}", to_string(actual), to_string(R)));

    auto ans = size / unit_size;
    auto mod = size % unit_size;
    if (mod != 0)
        throw std::runtime_error(
            fmt::format("gds record size {} not divisible by unit size {}", size, unit_size));

    if (num_data != 0 && ans != num_data) {
        throw std::runtime_error(
            fmt::format("gds record has {} elements, expected {}", ans, num_data));
    }
    return ans;
}

template <record_type R> std::string read_name(spdlog::logger &logger, std::istream &stream) {
    std::string ans;
    auto num = check_record_header<R, sizeof(char)>(stream);
    ans.reserve(num);
    for (decltype(num) idx = 0; idx < num; ++idx) {
        ans.push_back(read_bytes<char>(stream));
    }
    if (ans[num - 1] == '\0') {
        ans.pop_back();
    }
    return ans;
}

template <record_type R, std::size_t unit_size = 1, std::size_t num_data = 0>
void read_skip(std::istream &stream) {
    auto num = check_record_header<R, unit_size, num_data>(stream);
    stream.ignore(num * unit_size);
}

std::tuple<record_type, std::size_t> read_record_header(std::istream &stream);

point_t read_point(std::istream &stream);

std::string read_inst_name(spdlog::logger &logger, std::istream &stream, std::size_t &cnt);

void read_header(spdlog::logger &logger, std::istream &stream);

void read_lib_begin(spdlog::logger &logger, std::istream &stream);

std::string read_lib_name(spdlog::logger &logger, std::istream &stream);

void read_units(spdlog::logger &logger, std::istream &stream);

std::string read_struct_name(spdlog::logger &logger, std::istream &stream);

transformation read_transform(spdlog::logger &logger, std::istream &stream);

std::tuple<transformation, double> read_transform_info(spdlog::logger &logger,
                                                       std::istream &stream);

std::tuple<uint16_t, uint16_t> read_col_row(spdlog::logger &logger, std::istream &stream);

std::tuple<gds_layer_t, transformation, std::string, double> read_text(spdlog::logger &logger,
                                                                       std::istream &stream);

std::tuple<gds_layer_t, layout::poly_t> read_box(spdlog::logger &logger, std::istream &stream);

std::tuple<gds_layer_t, layout::poly_t> read_boundary(spdlog::logger &logger, std::istream &stream);

std::tuple<gds_layer_t, offset_t, enum_t, offset_t, offset_t, std::vector<point_t>> read_path(spdlog::logger &logger,
                                                                                              std::istream &stream);

layout::instance read_instance(
    spdlog::logger &logger, std::istream &stream, std::size_t &cnt,
    const std::unordered_map<std::string, std::shared_ptr<const layout::cellview>> &master_map);

layout::instance read_arr_instance(
    spdlog::logger &logger, std::istream &stream, std::size_t &cnt,
    const std::unordered_map<std::string, std::shared_ptr<const layout::cellview>> &master_map);

bool print_record(std::istream &stream);

} // namespace gdsii
} // namespace cbag

#endif
