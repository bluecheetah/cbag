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

#ifndef CBAG_SPIRIT_AST_H
#define CBAG_SPIRIT_AST_H

#include <array>
#include <cstdint>
#include <iterator>
#include <optional>
#include <string>
#include <variant>
#include <vector>

#include <boost/spirit/home/x3/support/ast/position_tagged.hpp>

#include <cbag/common/typedefs.h>
#include <cbag/spirit/namespace_info.h>

namespace x3 = boost::spirit::x3;

namespace cbag {
namespace spirit {
namespace ast {

/** Represents a range of indices at regular interval.
 *
 *  step size of 0 means that this range is empty; it doesn't contain any item.
 *  the step field is always non-negative.  However, if stop < start, then it is
 *  implied that the index decreases.
 *
 *  the stop field is inclusive.  However, get_stop_exclude() method will return
 *  an exclusive stop value if needed.
 */
struct range : x3::position_tagged {
  public:
    cnt_t start = 0;
    cnt_t stop = 0;
    cnt_t step = 0;

    class const_iterator {
      public:
        using iterator_category = std::forward_iterator_tag;
        using value_type = cnt_t;
        using difference_type = std::ptrdiff_t;
        using pointer = value_type *;
        using reference = const value_type &;

      private:
        cnt_t val_ = 0;
        cnt_t step_ = 0;
        bool up_ = false;

      public:
        const_iterator();

        const_iterator(cnt_t val, cnt_t step, bool up);

        const_iterator &operator++();
        reference operator*() const;
        bool operator==(const const_iterator &rhs) const;
        bool operator!=(const const_iterator &rhs) const;
    };

    range();

    range(cnt_t start, cnt_t stop, cnt_t step);

    bool empty() const noexcept;

    cnt_t size() const noexcept;

    std::array<cnt_t, 2> bounds() const noexcept;

    cnt_t get_stop_include() const;

    cnt_t get_stop_exclude() const;

    const_iterator begin() const;

    const_iterator end() const;

    // no bounds checking
    cnt_t operator[](cnt_t index) const;
};

struct name_bit {
  public:
    std::string base;
    std::optional<cnt_t> idx;

    name_bit();

    explicit name_bit(std::string base);

    name_bit(std::string base, cnt_t idx);

    bool operator==(const name_bit &rhs) const noexcept;

    std::size_t get_hash() const noexcept;

    std::string to_string() const;
    std::string to_string(bool is_id, namespace_cdba) const;
    std::string to_string(bool is_id, namespace_verilog) const;
    std::string to_string(bool is_id, namespace_spectre) const;
};

/** Represents a unit name; either a scalar or vector name.
 *
 *  POssible formats are "foo", "bar[2]", "baz[3:1]"
 *
 *  An empty base means that this name_unit is empty.
 */
struct name_unit : x3::position_tagged {
  public:
    std::string base;
    range idx_range;

    name_unit();

    name_unit(std::string base, range idx_range);

    bool empty() const;

    cnt_t size() const;

    bool is_vector() const;

    // precondition: 0 <= idx < size
    name_bit operator[](cnt_t index) const;

    std::string to_string(namespace_cdba) const;
    std::string to_string(namespace_verilog) const;
    std::string to_string(namespace_spectre) const;
};

std::string to_string(const std::string &base, std::array<cnt_t, 2> bounds, namespace_cdba);
std::string to_string(const std::string &base, std::array<cnt_t, 2> bounds, namespace_spectre);

struct name_rep;

/** Represents a list of name_rep's.
 *
 *  An empty rep_list means that this name is empty.
 */
struct name : x3::position_tagged {
  public:
    std::vector<name_rep> rep_list;

    name();

    name(std::vector<name_rep> rep_list);

    bool empty() const;

    bool is_name_rep() const;

    bool is_name_unit() const;

    cnt_t size() const;

    std::string to_string(namespace_cdba) const;
    std::string to_string(namespace_verilog) const;

    name &repeat(cnt_t mult);
};

/** Represents a repeated name
 *
 *  Possible formats are "<*3>foo", "<*3>(a,b)", or just name_unit.
 *  A mult of 0 means this name_rep is empty.
 */
struct name_rep : x3::position_tagged {
  public:
    cnt_t mult = 1;
    std::variant<name_unit, name> data;

    name_rep();

    name_rep(cnt_t mult, std::variant<name_unit, name> data);

    name_rep(cnt_t mult, name_unit nu);

    name_rep(cnt_t mult, name na);

    bool empty() const;

    bool is_name_unit() const;

    cnt_t size() const;

    cnt_t data_size() const;

    std::string to_string(namespace_cdba) const;
    std::string to_string(namespace_verilog) const;
};

} // namespace ast
} // namespace spirit
} // namespace cbag

// define hash function for name_bit
namespace std {

template <> struct hash<cbag::spirit::ast::name_bit> {
    std::size_t operator()(const cbag::spirit::ast::name_bit &k) const;
};

} // namespace std

#endif // CBAG_SPIRIT_AST_H
