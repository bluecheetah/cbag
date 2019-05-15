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

#include <cbag/common/box_t.h>
#include <cbag/common/transformation_util.h>
#include <cbag/common/vector.h>
#include <cbag/layout/via.h>

namespace cbag {
namespace layout {

struct via::helper {
    static box_t get_box(const via &self, const vector &offset, const vector &enc) {
        auto nx = self.params.num[0];
        auto ny = self.params.num[1];
        auto via_w = nx * self.params.cut_dim[0] + (nx - 1) * self.params.cut_spacing[0];
        auto via_h = ny * self.params.cut_dim[1] + (ny - 1) * self.params.cut_spacing[1];

        auto xl = static_cast<coord_t>(offset[0] - (via_w / 2) - enc[0]);
        auto yl = static_cast<coord_t>(offset[1] - (via_h / 2) - enc[1]);
        auto xh = static_cast<coord_t>(xl + via_w + enc[0]);
        auto yh = static_cast<coord_t>(yl + via_h + enc[1]);

        box_t r{xl, yl, xh, yh};
        transform(r, self.xform);
        return r;
    }
};

via::via() = default;

via::via(cbag::transformation xform, std::string via_id, via_param params)
    : via_id(std::move(via_id)), params(std::move(params)), xform(std::move(xform)) {}

const std::string &via::get_via_id() const { return via_id; }

const via_param &via::get_params() const { return params; }

bool via::operator==(const via &rhs) const noexcept {
    return via_id == rhs.via_id && params == rhs.params && xform == rhs.xform;
}

} // namespace layout
} // namespace cbag
