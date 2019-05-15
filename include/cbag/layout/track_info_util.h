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

#ifndef CBAG_LAYOUT_TRACK_INFO_UTIL_H
#define CBAG_LAYOUT_TRACK_INFO_UTIL_H

#include <cbag/common/transformation.h>
#include <cbag/enum/round_mode.h>
#include <cbag/layout/track_info.h>

namespace cbag {
namespace layout {

htr_t coord_to_htr(offset_t coord, offset_t pitch, offset_t off, round_mode mode, bool even);

htr_t coord_to_htr(const track_info &tr_info, offset_t coord, round_mode mode = round_mode::NONE,
                   bool even = false);

htr_t find_next_htr(const track_info &tr_info, offset_t coord, cnt_t ntr,
                    round_mode mode = round_mode::NONE, bool even = false);

offset_t htr_to_coord(htr_t htr, offset_t pitch, offset_t off) noexcept;

offset_t htr_to_coord(const track_info &tr_info, htr_t htr) noexcept;

cnt_t get_min_space(const track_info &tr_info, const tech &t, level_t level, cnt_t num_tr,
                    bool same_color, bool even);

htr_t transform_htr(const track_info &tr_info, htr_t htr, const transformation &xform);

} // namespace layout
} // namespace cbag

#endif
