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

#ifndef CBAG_UTIL_UNIQUE_HEAP_H
#define CBAG_UTIL_UNIQUE_HEAP_H

#include <queue>
#include <type_traits>
#include <unordered_set>

namespace cbag {
namespace util {

template <class T, class Hash = std::hash<T>> class unique_heap {
  private:
    template <class U> using IsT = std::enable_if_t<std::is_same_v<T, std::decay_t<U>>, int>;

    std::priority_queue<T> heap;
    std::unordered_set<T, Hash> items;

  public:
    unique_heap() = default;

    bool empty() const { return heap.empty(); }

    std::size_t size() const { return heap.size(); }

    template <class U, IsT<U> = 0> void push(U &&val) {
        auto result = items.insert(std::forward<U>(val));
        if (result.second) {
            heap.push(*(result.first));
        }
    }

    template <class... Args> void emplace(Args &&... args) {
        auto result = items.emplace(std::forward<Args>(args)...);
        if (result.second) {
            heap.push(*(result.first));
        }
    }

    const T &top() const { return heap.top(); }

    void pop() {
        items.erase(top());
        heap.pop();
    }
};

} // namespace util
} // namespace cbag

#endif
