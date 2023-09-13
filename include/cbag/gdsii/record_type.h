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

#ifndef CBAG_GDSII_RECORD_TYPE_H
#define CBAG_GDSII_RECORD_TYPE_H

#include <cstdint>
#include <stdexcept>
#include <string>

#include <fmt/core.h>

namespace cbag {
namespace gdsii {

enum class record_type : uint16_t {
    HEADER = 0x0002,
    BGNLIB = 0x0102,
    LIBNAME = 0x0206,
    UNITS = 0x0305,
    ENDLIB = 0x0400,
    BGNSTR = 0x0502,
    STRNAME = 0x0606,
    ENDSTR = 0x0700,
    BOUNDARY = 0x0800,
    PATH = 0x0900,
    SREF = 0x0A00,
    AREF = 0x0B00,
    TEXT = 0x0C00,
    LAYER = 0x0D02,
    DATATYPE = 0x0E02,
    WIDTH = 0x0F03,
    XY = 0x1003,
    ENDEL = 0x1100,
    SNAME = 0x1206,
    COLROW = 0x1302,
    TEXTTYPE = 0x1602,
    PRESENTATION = 0x1701,
    STRING = 0x1906,
    STRANS = 0x1A01,
    MAG = 0x1B05,
    ANGLE = 0x1C05,
    PATHTYPE = 0x2102,
    PROPATTR = 0x2B02,
    PROPVALUE = 0x2C06,
    BOX = 0x2D00,
    BOXTYPE = 0x2E02,
    BEGINEXTN = 0x3003,
    ENDEXTN = 0x3103,
};

constexpr auto PROP_INST_NAME = 1;
constexpr auto PROP_USER_STR = 126;
constexpr auto PROP_COMMENT = 112;

inline std::string to_string(record_type rec) {
    switch (rec) {
    case record_type::HEADER:
        return "HEADER";
    case record_type::BGNLIB:
        return "BGNLIB";
    case record_type::LIBNAME:
        return "LIBNAME";
    case record_type::UNITS:
        return "UNITS";
    case record_type::ENDLIB:
        return "ENDLIB";
    case record_type::BGNSTR:
        return "BGNSTR";
    case record_type::STRNAME:
        return "STRNAME";
    case record_type::ENDSTR:
        return "ENDSTR";
    case record_type::BOUNDARY:
        return "BOUNDARY";
    case record_type::PATH:
        return "PATH";
    case record_type::SREF:
        return "SREF";
    case record_type::AREF:
        return "AREF";
    case record_type::TEXT:
        return "TEXT";
    case record_type::LAYER:
        return "LAYER";
    case record_type::DATATYPE:
        return "DATATYPE";
    case record_type::WIDTH:
        return "WIDTH";
    case record_type::XY:
        return "XY";
    case record_type::ENDEL:
        return "ENDEL";
    case record_type::SNAME:
        return "SNAME";
    case record_type::COLROW:
        return "COLROW";
    case record_type::TEXTTYPE:
        return "TEXTTYPE";
    case record_type::PRESENTATION:
        return "PRESENTATION";
    case record_type::STRING:
        return "STRING";
    case record_type::STRANS:
        return "STRANS";
    case record_type::MAG:
        return "MAG";
    case record_type::ANGLE:
        return "ANGLE";
    case record_type::PATHTYPE:
        return "PATHTYPE";
    case record_type::PROPATTR:
        return "PROPATTR";
    case record_type::PROPVALUE:
        return "PROPVALUE";
    case record_type::BOX:
        return "BOX";
    case record_type::BOXTYPE:
        return "BOXTYPE";
    default:
        throw std::invalid_argument(
            fmt::format("Unknown record type: {:#06x}", static_cast<uint16_t>(rec)));
    }
}

} // namespace gdsii
} // namespace cbag

#endif
