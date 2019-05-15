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

#ifndef CBAG_YAML_ENUM_H
#define CBAG_YAML_ENUM_H

#include <yaml-cpp/yaml.h>

#include <cbag/enum/design_output.h>
#include <cbag/enum/end_style.h>
#include <cbag/enum/font_t.h>
#include <cbag/enum/path_style.h>
#include <cbag/enum/sig_type.h>
#include <cbag/enum/term_attr_type.h>
#include <cbag/enum/term_type.h>
#include <cbag/enum/text_align.h>
#include <cbag/enum/text_disp_format.h>
#include <cbag/polygon/enum.h>
#include <cbag/yaml/orientation.h>

namespace YAML {

template <typename T> using IsEnum = std::enable_if_t<std::is_enum_v<T>, int>;

template <typename T, IsEnum<T> = 0> struct convert_enum {
    static Node encode(const T &rhs) {
        Node n;
        n = static_cast<int>(rhs);
        return n;
    }

    static bool decode(const Node &node, T &rhs) {
        if (node.IsScalar()) {
            try {
                rhs = static_cast<T>(node.as<int>());
                return true;
            } catch (...) {
                return false;
            }
        } else {
            return false;
        }
    }
};

template <> struct convert<cbag::design_output> : public convert_enum<cbag::design_output> {};

template <> struct convert<cbag::path_style> : public convert_enum<cbag::path_style> {};

template <> struct convert<cbag::text_align> : public convert_enum<cbag::text_align> {};

template <> struct convert<cbag::font_t> : public convert_enum<cbag::font_t> {};

template <> struct convert<cbag::term_attr_type> : public convert_enum<cbag::term_attr_type> {};

template <> struct convert<cbag::text_disp_format> : public convert_enum<cbag::text_disp_format> {};

template <> struct convert<cbag::sig_type> : public convert_enum<cbag::sig_type> {};

template <> struct convert<cbag::term_type> : public convert_enum<cbag::term_type> {};

template <> struct convert<cbag::end_style> : public convert_enum<cbag::end_style> {};

} // namespace YAML

#endif // CBAG_YAML_PRIMITIVES_H
