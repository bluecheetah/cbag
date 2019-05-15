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

#ifndef CBAG_UTIL_SORTED_MAP_H
#define CBAG_UTIL_SORTED_MAP_H

#include <utility>

#include <cbag/util/sorted_vector.h>

namespace cbag {
namespace util {

template <class Key, class T, class Compare = std::less<Key>> class sorted_map {
  public:
    struct value_compare;
    using key_type = Key;
    using mapped_type = T;
    using value_type = std::pair<key_type, mapped_type>;
    using vector_type = sorted_vector<value_type, value_compare>;
    using size_type = typename vector_type::size_type;
    using difference_type = typename vector_type::difference_type;
    using key_compare = Compare;
    using reference = value_type &;
    using const_reference = const value_type &;
    using iterator = typename vector_type::iterator;
    using const_iterator = typename vector_type::const_iterator;
    using reverse_iterator = typename vector_type::reverse_iterator;
    using const_reverse_iterator = typename vector_type::const_reverse_iterator;

    template <typename V>
    using IsValue =
        std::enable_if_t<std::is_same_v<value_type, std::remove_cv_t<std::remove_reference_t<V>>>>;

    template <typename V>
    using IsKey =
        std::enable_if_t<std::is_same_v<key_type, std::remove_cv_t<std::remove_reference_t<V>>>>;

    struct value_compare {
        using is_transparent = void;

        key_compare key_comp_;

        template <class K> bool operator()(const value_type &lhs, const K &rhs) const {
            return key_comp_(lhs.first, rhs);
        }
        template <class K> bool operator()(const K &lhs, const value_type &rhs) const {
            return key_comp_(lhs, rhs.first);
        }
        bool operator()(const value_type &lhs, const value_type &rhs) const {
            return key_comp_(lhs.first, rhs.first);
        }
        template <class K> bool operator()(const K &lhs, const K &rhs) const {
            return key_comp_(lhs, rhs);
        }
    };

  private:
    vector_type data_;

  public:
    sorted_map() = default;

    sorted_map(std::initializer_list<value_type> init) : data_(init) {}

    size_type size() const noexcept { return data_.size(); }
    size_type capacity() const noexcept { return data_.capacity(); }
    bool empty() const noexcept { return data_.empty(); }
    const_iterator begin() const noexcept { return data_.begin(); }
    const_iterator end() const noexcept { return data_.end(); }
    const_iterator cbegin() const noexcept { return data_.begin(); }
    const_iterator cend() const noexcept { return data_.end(); }
    iterator begin() noexcept { return data_.begin(); }
    iterator end() noexcept { return data_.end(); }

    template <class K> const_iterator lower_bound(const K &x) const { return data_.lower_bound(x); }

    template <class K> iterator lower_bound(const K &x) { return data_.lower_bound(x); }

    template <class K> const_iterator upper_bound(const K &x) const { return data_.upper_bound(x); }

    template <class K> std::pair<const_iterator, const_iterator> equal_range(const K &x) const {
        return data_.equal_range(x);
    }

    friend bool operator==(const sorted_map &lhs, const sorted_map &rhs) {
        return lhs.data_ == rhs.data_;
    }
    friend bool operator!=(const sorted_map &lhs, const sorted_map &rhs) {
        return lhs.data_ != rhs.data_;
    }

    template <class K> const_iterator find(const K &x) const { return data_.find(x); }
    template <class K> iterator find(const K &x) { return data_.find(x); }

    void clear() noexcept { data_.clear(); }
    void reserve(size_type n) { data_.reserve(n); }

    template <class... Args> std::pair<iterator, bool> emplace(Args &&... args) {
        return data_.emplace_unique(std::forward<Args>(args)...);
    }

    template <class... Args> void emplace_back(Args &&... args) {
        data_.emplace_back(std::forward<Args>(args)...);
    }

    template <class V, typename = IsValue<V>> std::pair<iterator, bool> insert(V &&val) {
        return data_.insert_unique(std::forward<V>(val));
    }

    template <class InputIt> void insert(InputIt first, InputIt last) {
        for (; first != last; ++first) {
            emplace(first->first, first->second);
        }
    }

    template <class V, class M, typename = IsKey<V>>
    std::pair<iterator, bool> insert_or_assign(V &&k, M &&obj) {
        auto iter = data_.lower_bound(k);
        if (iter == data_.end() || data_.get_compare()(k, *iter))
            return {data_._insert_force(iter, value_type(std::forward<V>(k), std::forward<M>(obj))),
                    true};
        iter->second = std::forward<M>(obj);
        return {iter, false};
    }

    size_type erase(const key_type &key) {
        auto iter_range = data_.equal_range(key);
        size_type ans = iter_range.second - iter_range.first;
        if (ans > 0) {
            data_.erase(iter_range.first, iter_range.second);
        }
        return ans;
    }

    iterator replace_key(const key_type &old_key, const key_type &new_key) {
        iterator iter = data_.lower_bound(old_key);
        iterator end_iter = data_.end();
        if (iter == end_iter || iter->first != old_key)
            return end_iter;
        if (old_key == new_key)
            return iter;

        auto &comp = data_.get_compare();
        if (comp(old_key, new_key)) {
            // get replacement position
            auto iter_inc = iter + 1;
            auto pos = std::lower_bound(iter_inc, end_iter, new_key, comp);
            if (pos != end_iter && pos->first == new_key) {
                // new key is already in the map
                return end_iter;
            }
            if (pos == iter_inc) {
                // replacement position is the same
                iter->first = new_key;
                return iter;
            }
            // circular shift [iter, pos) down
            auto ans = data_.circ_shift(iter, pos, false);
            ans->first = new_key;
            return ans;
        } else {
            auto start_iter = data_.begin();
            // get replacement position
            auto pos = std::upper_bound(start_iter, iter, new_key, comp);
            if (pos != start_iter && (pos - 1)->first == new_key) {
                // new key is already in the map
                return end_iter;
            }
            if (pos == iter) {
                // replacement position is the same
                iter->first = new_key;
                return iter;
            }
            // circular shift [pos, iter + 1) up
            auto ans = data_.circ_shift(pos, iter + 1, true);
            ans->first = new_key;
            return ans;
        }
    }
};

} // namespace util
} // namespace cbag

#endif
