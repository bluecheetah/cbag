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

#include <iostream>
#include <iterator>

#include <fmt/core.h>

#include <cbag/common/transformation_util.h>
#include <cbag/gdsii/math.h>
#include <cbag/gdsii/read_util.h>
#include <cbag/gdsii/typedefs.h>
#include <cbag/layout/cellview.h>
#include <cbag/layout/instance.h>
#include <cbag/layout/label.h>

namespace cbag {
namespace gdsii {

std::tuple<record_type, std::size_t> read_record_header(std::istream &stream) {
    auto size = static_cast<std::size_t>(read_bytes<uint16_t>(stream)) - 4;
    auto record_val = read_bytes<uint16_t>(stream);

    return {static_cast<record_type>(record_val), size};
}

std::tuple<record_type, std::size_t> peek_record_header(std::istream &stream) {
    auto ans = read_record_header(stream);
    stream.seekg(-4, std::ios::cur);
    return ans;
}

std::tuple<record_type, std::size_t> print_record_header(std::istream &stream) {
    auto ans = read_record_header(stream);
    std::cout << to_string(std::get<0>(ans)) << ": " << std::get<1>(ans) << std::endl;
    return ans;
}

template <record_type R> uint16_t read_int(spdlog::logger &logger, std::istream &stream) {
    check_record_header<R, sizeof(uint16_t), 1>(stream);
    return read_bytes<uint16_t>(stream);
}

std::tuple<uint16_t, uint16_t> read_col_row(spdlog::logger &logger, std::istream &stream) {
    check_record_header<record_type::COLROW, sizeof(uint16_t), 2>(stream);
    auto nx = read_bytes<uint16_t>(stream);
    auto ny = read_bytes<uint16_t>(stream);
    return {nx, ny};
}

template <record_type R> double read_double(spdlog::logger &logger, std::istream &stream) {
    check_record_header<R, sizeof(uint64_t), 1>(stream);
    return gds_to_double(read_bytes<uint64_t>(stream));
}

template <record_type R> void read_grp_begin(spdlog::logger &logger, std::istream &stream) {
    read_skip<R, sizeof(tval_t), 12>(stream);
}

void read_header(spdlog::logger &logger, std::istream &stream) {
    read_skip<record_type::HEADER, sizeof(uint16_t), 1>(stream);
}

void read_lib_begin(spdlog::logger &logger, std::istream &stream) {
    read_grp_begin<record_type::BGNLIB>(logger, stream);
}

std::string read_lib_name(spdlog::logger &logger, std::istream &stream) {
    return read_name<record_type::LIBNAME>(logger, stream);
}

void read_units(spdlog::logger &logger, std::istream &stream) {
    read_skip<record_type::UNITS, sizeof(uint64_t), 2>(stream);
}

std::string read_struct_name(spdlog::logger &logger, std::istream &stream) {
    return read_name<record_type::STRNAME>(logger, stream);
}

void read_ele_end(spdlog::logger &logger, std::istream &stream) {
    check_record_header<record_type::ENDEL, sizeof(uint16_t), 0>(stream);
}

std::tuple<transformation, double> read_transform_info(spdlog::logger &logger,
                                                       std::istream &stream) {
    auto rec_peek = peek_record_header(stream);
    if (std::get<0>(rec_peek) != record_type::STRANS) {
        return {transformation(0, 0, orientation::R0), 1.0};
    }

    auto bit_flag = read_int<record_type::STRANS>(logger, stream);

    auto orient = orientation::R0;
    if ((bit_flag & (1 << 15)) != 0) {
        orient = orientation::MX;
    }
    auto ans = transformation(0, 0, orient);

    double mag = 1.0;
    double ang_dbl = 0.0;
    rec_peek = peek_record_header(stream);
    switch (std::get<0>(rec_peek)) {
    case record_type::MAG:
        mag = read_double<record_type::MAG>(logger, stream);
        rec_peek = peek_record_header(stream);
        if (std::get<0>(rec_peek) == record_type::ANGLE)
            ang_dbl = read_double<record_type::ANGLE>(logger, stream);
        break;
    case record_type::ANGLE:
        ang_dbl = read_double<record_type::ANGLE>(logger, stream);
        break;
    default:
        break;
    }

    auto angle = static_cast<int>(ang_dbl);
    switch (angle) {
    case 0:
        break;
    case 90:
        ans += orientation::R90;
        break;
    case 180:
        ans += orientation::R180;
        break;
    case 270:
        ans += orientation::R270;
        break;
    default:
        throw std::runtime_error("GDS rotation angle not supported: " + std::to_string(angle));
    }
    return {ans, mag};
}

point_t read_point(std::istream &stream) {
    auto x = read_bytes<int32_t>(stream);
    auto y = read_bytes<int32_t>(stream);
    return {x, y};
}

std::tuple<transformation, double> read_transform_mag(spdlog::logger &logger,
                                                      std::istream &stream) {
    auto ans = read_transform_info(logger, stream);
    check_record_header<record_type::XY, sizeof(int32_t), 2>(stream);
    auto pt = read_point(stream);
    std::get<0>(ans).move_by(x(pt), y(pt));

    return ans;
}

transformation read_transform(spdlog::logger &logger, std::istream &stream) {
    return std::get<0>(read_transform_mag(logger, stream));
}

std::tuple<gds_layer_t, transformation, std::string, double> read_text(spdlog::logger &logger,
                                                                       std::istream &stream) {
    auto glay = read_int<record_type::LAYER>(logger, stream);
    auto gpurp = read_int<record_type::TEXTTYPE>(logger, stream);

    auto rec_peek = peek_record_header(stream);
    if (std::get<0>(rec_peek) == record_type::PRESENTATION)
        read_skip<record_type::PRESENTATION, sizeof(uint16_t), 1>(stream);
    auto[xform, mag] = read_transform_mag(logger, stream);

    auto text = read_name<record_type::STRING>(logger, stream);
    read_ele_end(logger, stream);

    return {gds_layer_t{glay, gpurp}, std::move(xform), std::move(text), mag};
}

std::tuple<gds_layer_t, layout::poly_t> read_box(spdlog::logger &logger, std::istream &stream) {
    auto glay = read_int<record_type::LAYER>(logger, stream);
    auto gpurp = read_int<record_type::BOXTYPE>(logger, stream);
    check_record_header<record_type::XY, sizeof(int32_t), 10>(stream);

    std::vector<point_t> pt_vec;
    pt_vec.reserve(4);
    pt_vec.emplace_back(read_point(stream));
    pt_vec.emplace_back(read_point(stream));
    pt_vec.emplace_back(read_point(stream));
    pt_vec.emplace_back(read_point(stream));
    read_point(stream);
    read_ele_end(logger, stream);

    return {gds_layer_t{glay, gpurp}, layout::poly_t(std::move(pt_vec))};
}

std::tuple<gds_layer_t, layout::poly_t> read_boundary(spdlog::logger &logger,
                                                      std::istream &stream) {
    auto glay = read_int<record_type::LAYER>(logger, stream);
    auto gpurp = read_int<record_type::DATATYPE>(logger, stream);
    // divide by 2 to get number of points instead of number of coordinates.
    auto num = check_record_header<record_type::XY, sizeof(int32_t)>(stream) / 2;

    std::vector<point_t> pt_vec;
    pt_vec.reserve(num);
    for (decltype(num) idx = 0; idx < num; ++idx) {
        pt_vec.emplace_back(read_point(stream));
    }

    // a PR boundary may have a property attached to it
    auto[rtype, rsize] = read_record_header(stream);
    switch (rtype) {
    case record_type::ENDEL:
        break;
    case record_type::PROPATTR: {
        read_bytes<uint16_t>(stream);
        read_name<record_type::PROPVALUE>(logger, stream);
        read_ele_end(logger, stream);
        break;
    }
    default:
        throw std::runtime_error(
            fmt::format("Unexpected gds record type {} at end of boundary.", to_string(rtype)));
    }

    return {gds_layer_t{glay, gpurp}, layout::poly_t(std::move(pt_vec))};
}

gds_layer_t read_path(spdlog::logger &logger, std::istream &stream) {
    auto glay = read_int<record_type::LAYER>(logger, stream);
    auto gpurp = read_int<record_type::DATATYPE>(logger, stream);

    auto rec_peek = peek_record_header(stream);
    if (std::get<0>(rec_peek) == record_type::PATHTYPE) {
        read_int<record_type::PATHTYPE>(logger, stream);
    }
    rec_peek = peek_record_header(stream);
    if (std::get<0>(rec_peek) == record_type::WIDTH) {
        check_record_header<record_type::WIDTH, sizeof(int32_t), 1>(stream);
        read_bytes<int32_t>(stream);
    }

    // divide by 2 to get number of points instead of number of coordinates.
    auto num = check_record_header<record_type::XY, sizeof(int32_t)>(stream) / 2;

    for (decltype(num) idx = 0; idx < num; ++idx) {
        read_point(stream);
    }
    read_ele_end(logger, stream);

    return {glay, gpurp};
}

std::string read_inst_name(spdlog::logger &logger, std::istream &stream, std::size_t &cnt) {
    auto[rtype, rsize] = read_record_header(stream);
    std::string inst_name;
    switch (rtype) {
    case record_type::ENDEL:
        inst_name = "X" + std::to_string(cnt);
        ++cnt;
        break;
    case record_type::PROPATTR: {
        auto prop_code = read_bytes<uint16_t>(stream);
        auto val = read_name<record_type::PROPVALUE>(logger, stream);
        if (prop_code != PROP_INST_NAME) {
            if (prop_code != PROP_COMMENT)
                logger.warn("Unexpected gds property code {} for instance, with prop value = {}",
                            prop_code, val);
            inst_name = "X" + std::to_string(cnt);
            ++cnt;
        } else {
            inst_name = val;
        }
        read_ele_end(logger, stream);
        break;
    }
    default:
        throw std::runtime_error(
            fmt::format("Unexpected gds record type {} at end of instance.", to_string(rtype)));
    }
    return inst_name;
}

layout::instance read_instance(
    spdlog::logger &logger, std::istream &stream, std::size_t &cnt,
    const std::unordered_map<std::string, std::shared_ptr<const layout::cellview>> &master_map) {
    auto cell_name = read_name<record_type::SNAME>(logger, stream);

    auto iter = master_map.find(cell_name);
    if (iter == master_map.end()) {
        auto msg = fmt::format("Cannot find layout cellview {} in GDS file.", cell_name);
        logger.error(msg);
        throw std::runtime_error(msg);
    }
    auto master = iter->second;
    auto xform = read_transform(logger, stream);
    auto inst_name = read_inst_name(logger, stream, cnt);
    return {std::move(inst_name), master, std::move(xform)};
}

layout::instance read_arr_instance(
    spdlog::logger &logger, std::istream &stream, std::size_t &cnt,
    const std::unordered_map<std::string, std::shared_ptr<const layout::cellview>> &master_map) {
    auto cell_name = read_name<record_type::SNAME>(logger, stream);

    auto iter = master_map.find(cell_name);
    if (iter == master_map.end())
        throw std::runtime_error(
            fmt::format("Cannot find layout cellview {} in GDS file.", cell_name));
    auto master = iter->second;

    auto[xform, mag] = read_transform_info(logger, stream);

    auto[gds_nx, gds_ny] = read_col_row(logger, stream);
    check_record_header<record_type::XY, sizeof(int32_t), 6>(stream);
    auto p0 = read_point(stream);
    auto p1 = read_point(stream);
    auto p2 = read_point(stream);
    auto inst_name = read_inst_name(logger, stream, cnt);

    xform.move_by(p0[0], p0[1]);
    auto xform_inv = get_inverse(xform);
    auto p1_inv = xform_inv.transform(p1[0], p1[1]);
    auto p2_inv = xform_inv.transform(p2[0], p2[1]);
    auto gds_spx = p1_inv[0] / gds_nx;
    auto gds_spy = p2_inv[1] / gds_ny;

    auto[nx, ny, spx, spy] = cbag::convert_gds_array(xform, gds_nx, gds_ny, gds_spx, gds_spy);

    return {std::move(inst_name), master, std::move(xform), nx, ny, spx, spy};
}

void print_time(std::istream &stream) {
    struct tm val;
    auto timeinfo = &val;
    timeinfo->tm_year = read_bytes<uint16_t>(stream);
    timeinfo->tm_mon = read_bytes<uint16_t>(stream) - 1;
    timeinfo->tm_mday = read_bytes<uint16_t>(stream);
    timeinfo->tm_hour = read_bytes<uint16_t>(stream);
    timeinfo->tm_min = read_bytes<uint16_t>(stream);
    timeinfo->tm_sec = read_bytes<uint16_t>(stream);

    auto time_obj = std::mktime(timeinfo);
    std::cout << std::asctime(std::localtime(&time_obj));
}

bool print_record(std::istream &stream) {
    auto[record, size] = print_record_header(stream);
    auto ans = false;
    switch (record) {
    // empty records
    case record_type::ENDLIB:
        ans = true;
    case record_type::ENDSTR:
    case record_type::PATH:
    case record_type::BOUNDARY:
    case record_type::SREF:
    case record_type::AREF:
    case record_type::TEXT:
    case record_type::ENDEL:
    case record_type::BOX:
        break;
    // bit flag records
    case record_type::PRESENTATION:
    case record_type::STRANS:
        std::cout << fmt::format("bit flag: {:#06x}", read_bytes<uint16_t>(stream)) << std::endl;
        break;
    case record_type::HEADER:
    case record_type::LAYER:
    case record_type::PATHTYPE:
    case record_type::DATATYPE:
    case record_type::TEXTTYPE:
    case record_type::PROPATTR:
    case record_type::BOXTYPE:
        std::cout << fmt::format("value: {}", read_bytes<uint16_t>(stream)) << std::endl;
        break;
    case record_type::BGNLIB:
    case record_type::BGNSTR:
        std::cout << "modification time: " << std::endl;
        print_time(stream);
        std::cout << "access time: " << std::endl;
        print_time(stream);
        break;
    case record_type::COLROW: {
        auto nx = read_bytes<uint16_t>(stream);
        auto ny = read_bytes<uint16_t>(stream);
        std::cout << fmt::format("nx: {}, ny: {}", nx, ny) << std::endl;
        break;
    }

    case record_type::WIDTH:
        std::cout << fmt::format("value: {}", read_bytes<int32_t>(stream)) << std::endl;
        break;
    case record_type::XY: {
        auto num_pts = size / 4 / 2;
        for (std::size_t idx = 0; idx < num_pts; ++idx) {
            auto x = read_bytes<int32_t>(stream);
            auto y = read_bytes<int32_t>(stream);
            std::cout << fmt::format("xy: ({}, {})", x, y) << std::endl;
        }
        break;
    }

    case record_type::UNITS:
        std::cout << fmt::format("resolution: {}", gds_to_double(read_bytes<uint64_t>(stream)))
                  << std::endl;
        std::cout << fmt::format("res unit: {}", gds_to_double(read_bytes<uint64_t>(stream)))
                  << std::endl;
        break;
    case record_type::MAG:
    case record_type::ANGLE:
        std::cout << fmt::format("value: {}", gds_to_double(read_bytes<uint64_t>(stream)))
                  << std::endl;
        break;
    case record_type::LIBNAME:
    case record_type::STRNAME:
    case record_type::SNAME:
    case record_type::STRING:
    case record_type::PROPVALUE: {
        std::string ans;
        ans.reserve(size);
        for (std::size_t idx = 0; idx < size; ++idx) {
            ans.push_back(read_bytes<char>(stream));
        }
        std::cout << "value: " << ans << std::endl;
        break;
    }
    default:
        throw std::invalid_argument(
            fmt::format("Unknown record type: {:#06x}", static_cast<uint16_t>(record)));
    }

    return ans;
}

} // namespace gdsii
} // namespace cbag
