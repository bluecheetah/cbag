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

#ifndef CBAG_UTIL_INTERVAL_H
#define CBAG_UTIL_INTERVAL_H

#include <iterator>
#include <type_traits>
#include <utility>

#include <fmt/core.h>

#include <cbag/common/typedefs.h>
#include <cbag/util/sorted_vector.h>

namespace cbag {
namespace util {

namespace traits {

template <typename T> struct coordinate_type {};
template <typename T> struct interval {};

template <> struct coordinate_type<std::array<offset_t, 2>> {
    using coord_type = offset_t;
    using len_type = offset_t;
};
template <> struct interval<std::array<offset_t, 2>> {
    using intv_type = std::array<offset_t, 2>;
    using coordinate_type = coordinate_type<intv_type>::coord_type;

    static intv_type &intv(intv_type &i) { return i; }
    static const intv_type &intv(const intv_type &i) { return i; }
    static coordinate_type start(const intv_type &i) { return i[0]; }
    static coordinate_type stop(const intv_type &i) { return i[1]; }
    static void set_start(intv_type &i, coordinate_type val) { i[0] = val; }
    static void set_stop(intv_type &i, coordinate_type val) { i[1] = val; }
    static intv_type construct(coordinate_type start, coordinate_type stop) {
        return {start, stop};
    }
};

template <> struct coordinate_type<offset_t> {
    using coord_type = offset_t;
    using len_type = offset_t;
};
template <> struct interval<offset_t> {
    using coordinate_type = coordinate_type<offset_t>::coord_type;

    static coordinate_type start(coordinate_type i) { return i; }
    static coordinate_type stop(coordinate_type i) { return i + 1; }
};

} // namespace traits

template <typename T> bool nonempty(const T &i) {
    return traits::interval<T>::start(i) < traits::interval<T>::stop(i);
}

template <typename T>
void transform(T &i, typename traits::coordinate_type<T>::len_type scale,
               typename traits::coordinate_type<T>::len_type shift) {
    auto start = traits::interval<T>::start(i);
    auto stop = traits::interval<T>::stop(i);
    if (scale >= 0) {
        traits::interval<T>::set_start(i, scale * start + shift);
        traits::interval<T>::set_stop(i, scale * stop + shift);
    } else {
        traits::interval<T>::set_start(i, scale * stop + shift);
        traits::interval<T>::set_stop(i, scale * start + shift);
    }
}

template <typename T1, typename T2> bool operator==(const T1 &lhs, const T2 &rhs) {
    return traits::interval<T1>::start(lhs) == traits::interval<T2>::start(rhs) &&
           traits::interval<T1>::stop(lhs) == traits::interval<T2>::stop(rhs);
}

struct intv_comp {
    using is_transparent = void;

    template <typename T1, typename T2> bool operator()(const T1 &lhs, const T2 &rhs) const {
        return traits::interval<T1>::stop(lhs) <= traits::interval<T2>::start(rhs);
    }
};

template <class Interval = std::array<offset_t, 2>> class disjoint_intvs {
  public:
    using value_type = Interval;
    using coord_type = typename traits::coordinate_type<Interval>::coord_type;
    using len_type = typename traits::coordinate_type<Interval>::len_type;
    using vector_type = sorted_vector<value_type, intv_comp>;
    using size_type = typename vector_type::size_type;
    using iterator = typename vector_type::iterator;
    using const_iterator = typename vector_type::const_iterator;

    class const_intv_iterator {
      public:
        using iterator_category = std::forward_iterator_tag;
        using value_type = const typename traits::interval<disjoint_intvs::value_type>::intv_type;
        using difference_type = typename const_iterator::difference_type;
        using pointer = value_type *;
        using reference = value_type &;

      private:
        const_iterator iter_;

      public:
        const_intv_iterator() = default;
        const_intv_iterator(const_iterator val) : iter_(std::move(val)) {}

        bool operator==(const const_intv_iterator &other) const { return iter_ == other.iter_; }
        bool operator!=(const const_intv_iterator &other) const { return iter_ != other.iter_; }

        reference operator*() const { return traits::interval<Interval>::intv(*iter_); }
        pointer operator->() const { return &operator*(); }

        const_intv_iterator &operator++() {
            ++iter_;
            return *this;
        }
        const_intv_iterator operator++(int) {
            const_intv_iterator ans(iter_);
            operator++();
            return ans;
        }
    };

  private:
    vector_type data_;

  public:
    disjoint_intvs() = default;

    explicit disjoint_intvs(std::vector<value_type> &&data) : data_(std::move(data)) {}

    explicit disjoint_intvs(vector_type &&data) : data_(std::move(data)) {}

  private:
    static void intersect_helper(disjoint_intvs::vector_type &ans, const value_type &intv,
                                 disjoint_intvs::const_iterator &first,
                                 const disjoint_intvs::const_iterator &last,
                                 const intv_comp &comp) {
        auto[start_iter, stop_iter] = std::equal_range(first, last, intv, comp);
        first = (start_iter == stop_iter) ? start_iter : stop_iter - 1;
        for (; start_iter != stop_iter; ++(start_iter)) {
            coord_type start = std::max(traits::interval<Interval>::start(intv),
                                        traits::interval<Interval>::start(*start_iter));
            coord_type stop = std::min(traits::interval<Interval>::stop(intv),
                                       traits::interval<Interval>::stop(*start_iter));
            if (start < stop)
                ans.push_back(traits::interval<Interval>::construct(start, stop));
        }
    }

    int _emplace_helper(value_type &&item, bool abut, bool merge, bool check_only,
                        std::size_t search_size) {
        auto &comp = data_.get_compare();

        auto i_start = traits::interval<Interval>::start(item);
        auto i_stop = traits::interval<Interval>::stop(item);

        auto start1 = data_.begin();
        auto[start_iter, stop_iter] =
            (merge || !abut)
                ? std::equal_range(start1, start1 + search_size,
                                   std::array<coord_type, 2>{i_start - 1, i_stop + 1}, comp)
                : std::equal_range(start1, start1 + search_size, item, comp);

        auto cur_idx = start_iter - start1;
        if (start_iter == stop_iter) {
            // no overlapping or abutting intervals
            if (!check_only)
                data_._insert_force(start_iter, std::move(item));

            return cur_idx;
        } else if (merge) {
            if (!check_only) {
                // have overlapping/abutting intervals, and we want to merge
                auto ovl_start = traits::interval<Interval>::start(*start_iter);
                auto ovl_stop = traits::interval<Interval>::stop(*(stop_iter - 1));
                traits::interval<Interval>::set_start(item, std::min(i_start, ovl_start));
                traits::interval<Interval>::set_stop(item, std::max(i_stop, ovl_stop));
                // modify the first overlapping interval
                *start_iter = std::move(item);
                // erase the rest
                ++(start_iter);
                if (stop_iter > start_iter)
                    data_.erase(start_iter, stop_iter);
            }
            return cur_idx;
        } else {
            // has overlap, and not merging; adding failed.
            return -(cur_idx + 1);
        }
    }

  public:
    const_iterator begin() const { return data_.begin(); }
    const_iterator end() const { return data_.end(); }
    auto at_front() const -> typename vector_type::const_reference { return data_.at_front(); }
    auto at_back() const -> typename vector_type::const_reference { return data_.at_back(); }

    const_intv_iterator intv_begin() const { return const_intv_iterator(begin()); }
    const_intv_iterator intv_end() const { return const_intv_iterator(end()); }

    template <class K> std::pair<const_iterator, const_iterator> overlap_range(const K &key) const {
        return data_.equal_range(key);
    }
    template <class K> const_iterator upper_bound(const K &key) const {
        return data_.upper_bound(key);
    }
    bool empty() const { return data_.empty(); }
    size_type size() const { return data_.size(); }
    coord_type start() const { return traits::interval<Interval>::start(data_.at_front()); }
    coord_type stop() const { return traits::interval<Interval>::stop(data_.at_back()); }
    template <class K> const_iterator find_exact(const K &key) const {
        return data_.find_exact(key);
    }
    template <class K> bool contains(const K &key) const {
        return data_.find_exact(key) != data_.end();
    }
    template <class K> bool overlaps(const K &key) const { return data_.equal_size(key) > 0; }
    template <class K> bool covers(const K &key) const { return data_.equal_size(key) == 1; }

    disjoint_intvs get_intersection(const disjoint_intvs &other) const {
        auto iter1 = data_.begin();
        auto iter2 = other.data_.begin();
        auto end1 = data_.end();
        auto end2 = other.data_.end();
        const auto &comp = data_.get_compare();

        vector_type ans;
        auto size1 = end1 - iter1;
        auto size2 = end2 - iter2;
        while (size1 > 0 && size2 > 0) {
            if (size1 <= size2) {
                intersect_helper(ans, *iter1, iter2, end2, comp);
                ++iter1;
            } else {
                intersect_helper(ans, *iter2, iter1, end1, comp);
                ++iter2;
            }
            size1 = end1 - iter1;
            size2 = end2 - iter2;
        }
        return disjoint_intvs(std::move(ans));
    }

    template <class K> disjoint_intvs get_complement(const K &key) const {
        coord_type lower = traits::interval<K>::start(key);
        coord_type upper = traits::interval<K>::stop(key);
        vector_type ans;
        if (data_.empty()) {
            ans.push_back(traits::interval<Interval>::construct(lower, upper));
        } else {
            coord_type a = start();
            coord_type b = stop();
            if (a < lower || upper < b) {
                throw std::out_of_range(
                    fmt::format("disjoint_intvs interval [{:d}, {:d}) not covered by [{:d}, {:d})",
                                a, b, lower, upper));
            }
            for (const auto &item : data_) {
                coord_t i_start = traits::interval<Interval>::start(item);
                if (lower < i_start)
                    ans.push_back(traits::interval<Interval>::construct(lower, i_start));
                lower = traits::interval<Interval>::stop(item);
            }
            if (lower < upper)
                ans.push_back(traits::interval<Interval>::construct(lower, upper));
        }
        return disjoint_intvs(std::move(ans));
    }

    disjoint_intvs get_transform(coord_type scale = 1, coord_type shift = 0) const {
        vector_type ans;
        const_iterator first, last;
        if (scale > 0) {
            for (auto first = data_.begin(); first != data_.end(); ++first) {
                value_type item(*first);
                transform(item, scale, shift);
                ans.push_back(std::move(item));
            }
        } else if (scale < 0) {
            for (auto first = data_.rbegin(); first != data_.rend(); ++first) {
                value_type item(*first);
                transform(item, scale, shift);
                ans.push_back(std::move(item));
            }
        } else {
            return {};
        }
        return disjoint_intvs(std::move(ans));
    }

    disjoint_intvs get_copy() const { return disjoint_intvs(*this); }

    template <class K> std::pair<iterator, iterator> overlap_range(const K &key) {
        return data_.equal_range(key);
    }

    void clear() noexcept { data_.clear(); }

    template <class K> bool remove(const K &key) {
        auto iter = data_.find_exact(key);
        if (iter == data_.end())
            return false;
        data_.erase(iter);
        return true;
    }

    template <class K> bool remove_overlaps(const K &key) {
        auto[start_iter, stop_iter] = overlap_range(key);
        if (start_iter == stop_iter)
            return false;
        data_.erase(start_iter, stop_iter);
        return true;
    }

    template <class K> bool subtract(const K &key) {
        auto[start_iter, stop_iter] = overlap_range(key);
        auto overlap_size = stop_iter - start_iter;
        if (overlap_size == 0)
            return false;

        auto k_start = traits::interval<K>::start(key);
        auto k_stop = traits::interval<K>::stop(key);
        auto test = traits::interval<Interval>::start(*start_iter);
        if (test < k_start) {
            auto first_stop = traits::interval<Interval>::stop(*start_iter);
            // perform subtraction on first interval
            traits::interval<Interval>::set_stop(*start_iter, k_start);
            if ((--overlap_size) == 0) {
                if (first_stop > k_stop) {
                    // the given interval is a strict subset of one element
                    // need to break the one element into two
                    auto copy = *start_iter;
                    traits::interval<Interval>::set_start(copy, k_stop);
                    traits::interval<Interval>::set_stop(copy, first_stop);
                    data_._insert_force(start_iter + 1, std::move(copy));
                }
                // we're done as there's no more interval
                return true;
            } else {
                ++start_iter;
            }
        }

        auto last_iter = stop_iter - 1;
        test = traits::interval<Interval>::stop(*last_iter);
        if (k_stop < test) {
            // perform subtraction on last interval
            traits::interval<Interval>::set_start(*last_iter, k_stop);
            stop_iter = last_iter;
            if ((--overlap_size) == 0)
                return true;
        }

        // erase all completely overlapped intervals
        data_.erase(start_iter, stop_iter);
        return true;
    }

    disjoint_intvs &operator+=(const disjoint_intvs &other) {
        auto search_size = data_.size();

        auto stop = other.data_.rend();
        for (auto iter = other.data_.rbegin(); iter != stop; ++iter) {
            auto cur_idx = _emplace_helper(value_type(*iter), true, true, false, search_size);
            search_size = cur_idx + 1;
        }

        return *this;
    }

    template <class... Args> bool emplace(bool merge, bool abut, bool check_only, Args &&... args) {
        auto code = _emplace_helper(value_type(std::forward<Args>(args)...), abut, merge,
                                    check_only, data_.size());
        return code >= 0;
    }

    void expand_space(coord_type lower, coord_type upper, len_type delta) {
        if (empty())
            return;

        // fix first element
        auto iter_start = data_.begin();
        auto start = traits::interval<Interval>::start(*iter_start);
        auto stop = traits::interval<Interval>::stop(*iter_start);
        if (start == lower) {
            if (stop != upper)
                traits::interval<Interval>::set_stop(*iter_start, std::max(start, stop - delta));
            else {
                // no space in this interval
                return;
            }
            ++iter_start;
        }
        // fix last element
        auto iter_end = data_.end();
        auto &tmp = *(iter_end - 1);
        stop = traits::interval<Interval>::stop(tmp);
        if (stop == upper) {
            start = traits::interval<Interval>::start(tmp);
            traits::interval<Interval>::set_start(tmp, std::min(stop, start + delta));
            --iter_end;
        }
        // handle rest of the elements
        for (auto iter = iter_start; iter < iter_end; ++iter) {
            start = traits::interval<Interval>::start(*iter);
            stop = traits::interval<Interval>::stop(*iter);

            auto len = std::max(0, stop - start - delta * 2);
            auto new_start = (start + stop - len) / 2;
            traits::interval<Interval>::set_start(*iter, new_start);
            traits::interval<Interval>::set_stop(*iter, new_start + len);
        }
    }

    void fix_drc(coord_type lower, coord_type upper, len_type min_len, len_type min_sp) {
        auto marker = static_cast<int>(data_.size() - 1);

        if (marker >= 0 && upper - lower < min_len) {
            // we have some intervals, but total space is less than min length.
            // only solution is to fill everything up.
            auto &intv = data_._get(0);
            traits::interval<Interval>::set_start(intv, lower);
            traits::interval<Interval>::set_stop(intv, upper);
            data_.erase(data_.begin() + 1, data_.end());
            return;
        }

        for (; marker >= 0; --marker) {
            auto &cur_intv = data_._get(marker);
            auto start = traits::interval<Interval>::start(cur_intv);
            auto stop = traits::interval<Interval>::stop(cur_intv);
            auto pre_ptr = (marker > 0) ? &(data_._get(marker - 1)) : nullptr;
            auto post_ptr =
                (marker < static_cast<int>(data_.size() - 1)) ? &(data_._get(marker + 1)) : nullptr;

            if (pre_ptr) {
                // check minimum space violation with previous interval
                auto pre_stop = traits::interval<Interval>::stop(*pre_ptr);
                if (start - pre_stop < min_sp) {
                    // merge with previous interval, then move down
                    traits::interval<Interval>::set_stop(*pre_ptr, stop);
                    data_.erase(data_.begin() + marker);
                    continue;
                }
            }
            // at this point we, know min_sp is satisfied with previous block.
            auto cur_len = stop - start;
            if (cur_len < min_len) {
                // fix minimum length rule
                auto delta = min_len - cur_len;
                auto stop_max = upper;
                if (post_ptr) {
                    // check if its better to merge with post_intv to solve min length rule
                    auto post_start = traits::interval<Interval>::start(*post_ptr);
                    if (delta >= post_start - stop) {
                        // merge with post_intv
                        auto post_stop = traits::interval<Interval>::stop(*post_ptr);
                        traits::interval<Interval>::set_stop(cur_intv, post_stop);
                        data_.erase(data_.begin() + marker + 1);
                        continue;
                    }
                    stop_max = post_start - min_sp;
                }
                if (pre_ptr) {
                    // check if its better to merge with pre_intv to solve min length rule
                    auto pre_stop = traits::interval<Interval>::stop(*pre_ptr);
                    if (delta >= start - pre_stop) {
                        // merge with pre_intv
                        traits::interval<Interval>::set_stop(*pre_ptr, stop);
                        data_.erase(data_.begin() + marker);
                        continue;
                    }
                }

                // we don't have any easy merges with adjacent blocks, try to expand
                // first determine the upper edge to keep min_sp and upper limit satisfied
                auto new_stop = std::min(stop_max, (start + stop + min_len) / 2);
                auto new_start = new_stop - min_len;
                // check lower limit rule
                if (new_start < lower) {
                    if (lower + min_len > stop_max) {
                        // cannot meet lower limit rule and min_sp rule with post_intv,
                        // so merge with post_intv
                        auto post_stop = traits::interval<Interval>::stop(*post_ptr);
                        traits::interval<Interval>::set_stop(cur_intv, post_stop);
                        data_.erase(data_.begin() + marker + 1);
                        continue;
                    } else {
                        // expand but clip to lower limit
                        traits::interval<Interval>::set_start(cur_intv, lower);
                        traits::interval<Interval>::set_stop(cur_intv, lower + min_len);
                        // since we're clipped to the lower limit, we're done here
                        data_.erase(data_.begin(), data_.begin() + marker);
                        return;
                    }
                }

                // check min_sp rule with previous block
                if (!pre_ptr || traits::interval<Interval>::stop(*pre_ptr) + min_sp <= new_start) {
                    // expand this interval and be done
                    traits::interval<Interval>::set_start(cur_intv, new_start);
                    traits::interval<Interval>::set_stop(cur_intv, new_stop);
                } else {
                    // this interval cannot be expanded to meet min space or limit rules
                    // merge with adjacent intervals
                    auto merge_bot = bool(pre_ptr);
                    if (pre_ptr && post_ptr) {
                        // merge with the closer interval
                        auto post_start = traits::interval<Interval>::start(*post_ptr);
                        auto pre_stop = traits::interval<Interval>::stop(*pre_ptr);
                        merge_bot = (start - pre_stop) < (post_start - stop);
                    }
                    if (merge_bot) {
                        traits::interval<Interval>::set_stop(*pre_ptr, stop);
                        data_.erase(data_.begin() + marker);
                    } else {
                        auto post_stop = traits::interval<Interval>::stop(*post_ptr);
                        traits::interval<Interval>::set_stop(cur_intv, post_stop);
                        data_.erase(data_.begin() + marker + 1);
                    }
                }
            }
        }
    }

    bool operator==(const disjoint_intvs &rhs) const noexcept { return data_ == rhs.data_; }

    typename vector_type::const_reference operator[](std::size_t idx) const { return data_[idx]; }
}; // namespace util

} // namespace util
} // namespace cbag

#endif
