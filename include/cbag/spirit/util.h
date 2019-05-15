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

#ifndef CBAG_SPIRIT_UTIL_H
#define CBAG_SPIRIT_UTIL_H

#include <fmt/core.h>

#include <cbag/spirit/ast.h>
#include <cbag/spirit/namespace_info.h>

namespace cbag {
namespace spirit {
namespace util {

using name = ast::name;
using name_unit = ast::name_unit;
using name_rep = ast::name_rep;
using range = ast::range;

template <typename NS>
using IsNameSpace =
    std::enable_if_t<std::is_same_v<std::remove_reference_t<NS>, spirit::namespace_cdba>, int>;

template <class OutIter>
void get_name_bits_helper(const name_unit &obj, OutIter &&iter, const char *fmt_str) {
    if (obj.is_vector()) {
        for (const auto &idx : obj.idx_range) {
            *iter = fmt::format(fmt_str, obj.base, idx);
        }
    } else {
        *iter = obj.base;
    }
}

template <class OutIter> void get_name_bits(const name_unit &obj, OutIter &&iter, namespace_cdba) {
    get_name_bits_helper(obj, std::forward<OutIter>(iter), "{}<{}>");
}

template <class OutIter>
void get_name_bits(const name_unit &obj, OutIter &&iter, namespace_verilog) {
    get_name_bits_helper(obj, std::forward<OutIter>(iter), "{}[{}]");
}

template <class OutIter, class NS, typename = IsNameSpace<NS>>
void get_name_bits(const name &obj, OutIter &&iter, NS ns);

template <class OutIter, class NS, typename = IsNameSpace<NS>>
void get_name_bits(const name_rep &obj, OutIter &&iter, NS ns) {
    auto n = obj.size();
    if (n == 0)
        return;
    else if (obj.mult == 1) {
        std::visit(
            [&ns, &iter](const auto &arg) { get_name_bits(arg, std::forward<OutIter>(iter), ns); },
            obj.data);
    } else {
        std::vector<std::string> cache = std::visit(
            [&ns](const auto &arg) {
                std::vector<std::string> ans;
                ans.reserve(arg.size());
                util::get_name_bits(arg, std::back_inserter(ans), ns);
                return ans;
            },
            obj.data);
        for (decltype(obj.mult) cnt = 0; cnt < obj.mult; ++cnt) {
            for (auto &name_bit : cache) {
                *iter = name_bit;
            }
        }
    }
}

template <class OutIter, class NS, typename>
void get_name_bits(const name &obj, OutIter &&iter, NS ns) {
    for (const auto &name_rep : obj.rep_list) {
        get_name_bits(name_rep, iter, ns);
    }
}

template <class OutIter>
void get_partition_helper(OutIter &&iter, const name_unit &obj, std::vector<name_rep> &rep_list,
                          cnt_t &cum_size, cnt_t chunk) {
    auto cur_size = obj.idx_range.size();
    if (cur_size > 0) {
        // this is a vector name unit
        auto step = obj.idx_range.step;
        cnt_t start_idx = 0;
        // this vector name could be very long, use for loop to partition
        // into chunks
        for (decltype(cur_size) stop_idx = chunk - cum_size; stop_idx <= cur_size;
             start_idx = stop_idx, stop_idx += chunk) {
            rep_list.emplace_back(1, name_unit(obj.base, {obj.idx_range[start_idx],
                                                          obj.idx_range[stop_idx - 1], step}));
            *iter = name(std::move(rep_list));
            rep_list.clear();
        }
        // check for residual chunks, and set cum_size properly
        if (start_idx < cur_size) {
            rep_list.emplace_back(
                1, name_unit(obj.base, {obj.idx_range[start_idx], obj.idx_range.stop, step}));
            cum_size = cur_size - start_idx;
        } else {
            cum_size = 0;
        }
    } else {
        // this is a scalar name unit, just add itself
        rep_list.emplace_back(1, obj);
        ++cum_size;
        if (cum_size == chunk) {
            *iter = name(std::move(rep_list));
            rep_list.clear();
            cum_size = 0;
        }
    }
}

template <class OutIter>
void get_partition_helper(OutIter &&iter, const name &obj, std::vector<name_rep> &rep_list,
                          cnt_t &cum_size, cnt_t chunk);

template <class OutIter>
void get_partition_helper(OutIter &&iter, const name_rep &obj, std::vector<name_rep> &rep_list,
                          cnt_t &cum_size, cnt_t chunk) {
    auto data_n = obj.data_size();

    auto remain_rep = obj.mult;
    while (remain_rep > 0) {
        auto result = std::div(static_cast<long>(chunk - cum_size), static_cast<long>(data_n));
        auto mult1 = static_cast<cnt_t>(result.quot);
        if (mult1 >= remain_rep) {
            // this name_rep is used up
            rep_list.emplace_back(remain_rep, obj.data);
            cum_size += remain_rep * data_n;
            remain_rep = 0;
        } else {
            // still has some remaining
            if (mult1 > 0) {
                rep_list.emplace_back(mult1, obj.data);
                cum_size += mult1 * data_n;
            }
            remain_rep -= mult1;
            if (result.rem != 0) {
                // we need to partition the data
                std::visit(
                    [&iter, &rep_list, &cum_size, &chunk](const auto &v) {
                        get_partition_helper(iter, v, rep_list, cum_size, chunk);
                    },
                    obj.data);
                remain_rep -= 1;
            }
        }
        if (cum_size == chunk) {
            // assign to output iterator
            *iter = name(std::move(rep_list));
            rep_list.clear();
            cum_size = 0;
        }
    }
}

template <class OutIter>
void get_partition_helper(OutIter &&iter, const name &obj, std::vector<name_rep> &rep_list,
                          cnt_t &cum_size, cnt_t chunk) {
    for (const auto &nr : obj.rep_list) {
        get_partition_helper(iter, nr, rep_list, cum_size, chunk);
    }
}

template <class OutIter> void get_partition(const name &obj, cnt_t chunk, OutIter &&iter) {
    std::vector<name_rep> temp;
    cnt_t cum_size = 0;
    get_partition_helper(std::forward<OutIter>(iter), obj, temp, cum_size, chunk);
    if (cum_size != 0) {
        *iter = name(std::move(temp));
    }
}

} // namespace util
} // namespace spirit
} // namespace cbag

#endif // CBAG_SPIRIT_AST_H
