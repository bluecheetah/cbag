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

#ifndef CBAG_LAYOUT_PT_VECTOR_H
#define CBAG_LAYOUT_PT_VECTOR_H

#include <vector>

#include <cbag/common/point_t.h>
#include <cbag/common/typedefs.h>
#include <cbag/layout/pt_list.h>

namespace cbag {
namespace layout {

using pt_vector = std::vector<point_t>;

namespace traits {

template <> struct pt_list<pt_vector> {
    using coordinate_type = coord_t;

    static std::size_t size(const pt_vector &vec) { return vec.size(); }
    static coordinate_type x(const pt_vector &vec, std::size_t idx) { return vec[idx][0]; }
    static coordinate_type y(const pt_vector &vec, std::size_t idx) { return vec[idx][1]; }
    static void set_x(pt_vector &vec, std::size_t idx, coordinate_type val) { vec[idx][0] = val; }
    static void set_y(pt_vector &vec, std::size_t idx, coordinate_type val) { vec[idx][1] = val; }
    static auto begin(const pt_vector &vec) -> decltype(vec.begin()) { return vec.begin(); }
    static auto end(const pt_vector &vec) -> decltype(vec.end()) { return vec.end(); }
};

} // namespace traits

} // namespace layout
} // namespace cbag

#endif
