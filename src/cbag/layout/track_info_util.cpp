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
#include <cstdlib>

#include <fmt/core.h>

#include <cbag/common/transformation_util.h>
#include <cbag/layout/tech_util.h>
#include <cbag/layout/track_info_util.h>
#include <cbag/layout/wire_width.h>
#include <cbag/util/math.h>

namespace cbag {
namespace layout {

htr_t coord_to_htr(offset_t coord, offset_t pitch, offset_t off, round_mode mode, bool even) {
    auto modulus = pitch >> (1 - even);
    auto delta = coord - off;
    auto div_res = std::div(delta, modulus);
    auto q = div_res.quot;
    auto r = div_res.rem;
    if (r < 0) {
        --q;
        r += modulus;
    }
    switch (mode) {
    case round_mode::LESS:
        q -= (r == 0);
        break;
    case round_mode::LESS_EQ:
        break;
    case round_mode::NEAREST: {
        auto mod2 = modulus / 2;
        // round to nearest, if exactly in middle,
        // (meaning p2 is even and r == p4), round to even.
        q += (r > mod2 || (r == mod2 && (modulus & 1) == 0 && ((q + 1) & 1) == 0));
        break;
    }
    case round_mode::GREATER_EQ:
        q += (r != 0);
        break;
    case round_mode::GREATER:
        ++q;
        break;
    default:
        if (r != 0)
            throw std::invalid_argument(fmt::format(
                "Coordinate {} is not on track.  (pitch = {}, off = {})", coord, modulus, off));
    }
    return q << even;
}

htr_t coord_to_htr(const track_info &tr_info, offset_t coord, round_mode mode, bool even) {
    return coord_to_htr(coord, tr_info.get_pitch(), tr_info.get_offset(), mode, even);
}

offset_t htr_to_coord(htr_t htr, offset_t pitch, offset_t off) noexcept {
    return (pitch * htr) / 2 + off;
}

offset_t htr_to_coord(const track_info &tr_info, htr_t htr) noexcept {
    return htr_to_coord(htr, tr_info.get_pitch(), tr_info.get_offset());
}

htr_t transform_htr(const track_info &tr_info, htr_t htr, const transformation &xform) {
    if (swaps_xy(xform.orient()))
        throw std::invalid_argument("Cannot transform track index when axes are swapped.");

    auto pdir = perpendicular(tr_info.get_direction());
    auto scale = axis_scale(xform.orient())[to_int(pdir)];
    auto offset = coord_to_htr(tr_info, xform.coord(pdir) + tr_info.get_offset());
    return scale * htr + scale - 1 + offset;
}

} // namespace layout
} // namespace cbag
