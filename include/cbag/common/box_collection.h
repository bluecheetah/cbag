// SPDX-License-Identifier: Apache-2.0
/*
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

#ifndef CBAG_COMMON_BOX_COLLECTION_H
#define CBAG_COMMON_BOX_COLLECTION_H

#include <vector>

#include <cbag/common/box_array.h>

namespace cbag {

class box_collection {
  private:
    using vector_type = std::vector<box_array>;
    vector_type data_;
    std::size_t nbox_ = 0;

  public:
    using const_iterator = vector_type::const_iterator;

    box_collection();

    std::size_t size() const noexcept;

    std::size_t num_box() const noexcept;

    void append(const box_t &base, cnt_t nx, cnt_t ny, offset_t spx, offset_t spy);

    auto begin() const -> const_iterator;

    auto end() const -> const_iterator;
};

} // namespace cbag

#endif
