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

#include <algorithm>
#include <cassert>

#include <fmt/core.h>

#include <cbag/spirit/ast.h>
#include <cbag/spirit/namespace_info.h>
#include <cbag/spirit/util.h>
#include <cbag/util/overload.h>

namespace cbag {
namespace spirit {
namespace ast {

range::const_iterator::const_iterator() = default;

range::const_iterator::const_iterator(cnt_t val, cnt_t step, bool up)
    : val_(val), step_(step), up_(up) {}

range::const_iterator &range::const_iterator::operator++() {
    if (up_)
        val_ += step_;
    else
        val_ -= step_;
    return *this;
}

range::const_iterator::reference range::const_iterator::operator*() const { return val_; }

bool range::const_iterator::operator==(const range::const_iterator &rhs) const {
    return val_ == rhs.val_ && step_ == rhs.step_ && up_ == rhs.up_;
}

bool range::const_iterator::operator!=(const range::const_iterator &rhs) const {
    return !(*this == rhs);
}

range::range() {}

range::range(cnt_t start, cnt_t stop, cnt_t step) : start(start), stop(stop), step(step) {}

bool range::empty() const noexcept { return step == 0; }

cnt_t range::size() const noexcept {
    if (step == 0) {
        return 0;
    } else if (stop >= start) {
        return (stop - start + step) / step;
    } else {
        return (start - stop + step) / step;
    }
}

std::array<cnt_t, 2> range::bounds() const noexcept {
    if (step == 0)
        return {0, 0};
    else if (stop >= start)
        return {start, get_stop_include() + 1};
    else
        return {get_stop_include(), start + 1};
}

cnt_t range::get_stop_include() const {
    auto n = size();
    assert(n > 0);
    return operator[](n - 1);
}

cnt_t range::get_stop_exclude() const { return operator[](size()); }

range::const_iterator range::begin() const { return {start, step, stop >= start}; }
range::const_iterator range::end() const { return {get_stop_exclude(), step, stop >= start}; }

cnt_t range::operator[](cnt_t index) const {
    return (stop >= start) ? start + step * index : start - step * index;
}

name_unit::name_unit() = default;

name_unit::name_unit(std::string base, range idx_range)
    : base(std::move(base)), idx_range(std::move(idx_range)) {}

bool name_unit::empty() const { return base.empty(); }

cnt_t name_unit::size() const { return std::max(idx_range.size(), static_cast<cnt_t>(1)); }

bool name_unit::is_vector() const { return !idx_range.empty(); }

std::string name_unit::to_string(namespace_cdba) const {
    if (empty())
        return "";
    if (is_vector()) {
        auto n = idx_range.size();
        if (n == 1)
            return fmt::format("{}<{}>", base, idx_range.start);
        if (idx_range.step == 1)
            return fmt::format("{}<{}:{}>", base, idx_range.start, idx_range.stop);
        return fmt::format("{}<{}:{}:{}>", base, idx_range.start, idx_range.stop, idx_range.step);
    } else {
        return base;
    }
}

std::string name_unit::to_string(namespace_verilog) const {
    if (empty())
        return "";
    if (is_vector()) {
        auto n = idx_range.size();
        if (n == 1)
            return fmt::format("{}[{}]", base, idx_range.start);
        if (idx_range.step == 1)
            return fmt::format("{}[{}:{}]", base, idx_range.start, idx_range.stop);
        // verilog does not support skip indexing
        std::string ans = "{";
        ans.reserve(2 + (base.size() + 4) * n);
        auto iter = idx_range.begin();
        auto stop = idx_range.end();
        ans.append(fmt::format("{}[{}]", base, *iter));
        ++iter;
        for (; iter != stop; ++iter) {
            ans.append(fmt::format(",{}[{}]", base, *iter));
        }
        ans.append("}");
        return ans;
    } else {
        return base;
    }
}

// precondition: bounds[1] >= bounds[0]
std::string to_string(const std::string &base, std::array<cnt_t, 2> bounds, namespace_cdba) {
    if (base.empty() || bounds[0] == bounds[1])
        return base;
    if (bounds[1] - bounds[0] == 1)
        return fmt::format("{}<{}>", base, bounds[0]);
    return fmt::format("{}<{}:{}>", base, bounds[0], bounds[1] - 1);
}

std::string get_name_bit_helper(const name_unit &nu, cnt_t index, bool is_id, const char *fmt_str) {
    assert(0 <= index && index < nu.size());
    if (nu.is_vector()) {
        if (is_id)
            return fmt::format("{}_{}", nu.base, nu.idx_range[index]);
        return fmt::format(fmt_str, nu.base, nu.idx_range[index]);
    }
    return nu.base;
}

std::string name_unit::get_name_bit(cnt_t index, bool is_id, namespace_cdba) const {
    return get_name_bit_helper(*this, index, is_id, "{}<{}>");
}

std::string name_unit::get_name_bit(cnt_t index, bool is_id, namespace_verilog) const {
    return get_name_bit_helper(*this, index, is_id, "{}[{}]");
}

name_rep::name_rep() = default;

name_rep::name_rep(cnt_t mult, std::variant<name_unit, name> data)
    : mult(mult), data(std::move(data)) {}

name_rep::name_rep(cnt_t mult, name_unit nu) : mult(mult), data(std::move(nu)) {}

name_rep::name_rep(cnt_t mult, name na) : mult(mult), data(std::move(na)) {}

bool name_rep::empty() const { return mult == 0; }

bool name_rep::is_name_unit() const {
    return mult == 0 || (mult == 1 && std::holds_alternative<name_unit>(data));
}

cnt_t name_rep::size() const { return mult * data_size(); }

cnt_t name_rep::data_size() const {
    return std::visit([](const auto &arg) { return arg.size(); }, data);
}

std::string name_rep::to_string(namespace_cdba ns) const {
    if (empty())
        return "";
    std::string base = std::visit([&ns](const auto &arg) { return arg.to_string(ns); }, data);
    if (mult == 1)
        return base;

    if (std::holds_alternative<name_unit>(data))
        return fmt::format("<*{}>{}", mult, base);
    return fmt::format("<*{}>({})", mult, base);
}

std::string name_rep::to_string(namespace_verilog ns) const {
    if (empty())
        return "";
    cnt_t m = mult;
    return std::visit(overload{
                          [&m, &ns](const name_unit &arg) {
                              std::string base = arg.to_string(ns);
                              if (m == 1)
                                  return base;
                              if (arg.size() > 1 && arg.idx_range.step > 1)
                                  // step size > 1: name_unit has bracket around it already
                                  return fmt::format("{{{}{}}}", m, base);
                              // need to add bracket around name_unit
                              return fmt::format("{{{}{{{}}}}}", m, base);
                          },
                          [&m, &ns](const name &arg) {
                              std::string base = arg.to_string(ns);
                              if (m == 1)
                                  return base;
                              return fmt::format("{{{}{}}}", m, base);
                          },
                      },
                      data);
}

name::name() = default;

name::name(std::vector<name_rep> rep_list) : rep_list(std::move(rep_list)) {}

bool name::empty() const { return rep_list.empty(); }

bool name::is_name_rep() const { return rep_list.size() == 1; }

bool name::is_name_unit() const { return rep_list.size() == 1 && rep_list[0].is_name_unit(); }

cnt_t name::size() const {
    cnt_t tot = 0;
    for (auto const &nr : rep_list) {
        tot += nr.size();
    }
    return tot;
}

template <class NS> std::string name_to_string_helper(const name &na, NS ns) {
    std::size_t n = na.rep_list.size();
    if (n == 0)
        return "";
    std::string ans = na.rep_list.front().to_string(ns);
    for (std::size_t idx = 1; idx < n; ++idx) {
        ans.append(",");
        ans.append(na.rep_list[idx].to_string(ns));
    }
    return ans;
}

std::string name::to_string(namespace_cdba ns) const { return name_to_string_helper(*this, ns); }

std::string name::to_string(namespace_verilog ns) const {
    std::string base = name_to_string_helper(*this, ns);
    if (is_name_rep())
        return base;
    return fmt::format("{{{}}}", base);
}

name &name::repeat(cnt_t mult) {
    if (!empty()) {
        switch (mult) {
        case 0:
            rep_list.clear();
            break;
        case 1:
            break;
        default:
            if (rep_list.size() == 1) {
                rep_list[0].mult *= mult;
            } else {
                std::vector<name_rep> other = std::move(rep_list);
                rep_list.clear();
                rep_list.emplace_back(mult, name(std::move(other)));
            }
        }
    }
    return *this;
}

} // namespace ast
} // namespace spirit
} // namespace cbag
