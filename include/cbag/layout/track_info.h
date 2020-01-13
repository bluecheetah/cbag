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

#ifndef CBAG_LAYOUT_TRACK_INFO_H
#define CBAG_LAYOUT_TRACK_INFO_H

#include <array>
#include <vector>

#include <cbag/util/interval.h>

#include <cbag/common/typedefs.h>
#include <cbag/polygon/enum.h>

#include <cbag/layout/wire_width.h>

namespace cbag {
namespace layout {

class tech;
class routing_grid;

class track_info {
  private:
    orientation_2d dir_ = orientation_2d::HORIZONTAL;
    offset_t w_ = 0;
    offset_t sp_ = 0;
    offset_t offset_ = 0;

  public:
    track_info();

    track_info(orientation_2d tr_dir, offset_t tr_w, offset_t tr_sp, offset_t tr_off);

    bool operator==(const track_info &rhs) const noexcept;

    std::size_t get_hash() const noexcept;

    bool compatible(const track_info &rhs) const noexcept;

    orientation_2d get_direction() const noexcept;

    offset_t get_width() const noexcept;

    offset_t get_space() const noexcept;

    offset_t get_pitch() const noexcept;

    offset_t get_offset() const noexcept;
};

} // namespace layout
} // namespace cbag

#endif
