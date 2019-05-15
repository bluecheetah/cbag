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

#ifndef CBAG_LAYOUT_WIRE_WIDTH_H
#define CBAG_LAYOUT_WIRE_WIDTH_H

#include <iterator>
#include <tuple>
#include <vector>

#include <cbag/common/typedefs.h>

namespace cbag {
namespace layout {

class tech;

/** A class that represents the "width" of a "conceptual wire".
 * A "conceptual wire" consists of one or more wires, since some technologies
 * have discrete width choices, so wide wires have to be realized with multiple
 * small wires.  This class encapsulates the "width" detail of a wire.
 */
class wire_width {
  private:
    using vec_type = std::vector<std::tuple<htr_t, offset_t>>;
    vec_type widths_;

  public:
    class width_iter {
      public:
        using iterator_category = std::forward_iterator_tag;
        using value_type = offset_t;
        using difference_type = std::ptrdiff_t;
        using pointer = const value_type *;
        using reference = const value_type &;

      private:
        vec_type::const_iterator vec_iter;

      public:
        width_iter();

        explicit width_iter(vec_type::const_iterator iter);

        width_iter &operator++();
        reference operator*() const;
        bool operator==(const width_iter &rhs) const;
        bool operator!=(const width_iter &rhs) const;
    };

    wire_width();

    explicit wire_width(vec_type &&widths);

    const std::tuple<htr_t, offset_t> &operator[](std::size_t idx) const;

    std::size_t size() const;

    vec_type::const_iterator begin() const;

    vec_type::const_iterator end() const;

    width_iter begin_width() const;

    width_iter end_width() const;

    offset_t get_edge_wire_width() const;

    offset_t get_total_width(offset_t half_pitch) const;
};

} // namespace layout
} // namespace cbag

#endif
