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

#include <cassert>
#include <cmath>
#include <ctime>

#include <cbag/gdsii/math.h>

namespace cbag {
namespace gdsii {

// from IEEE double standard
// I did not use cfloat macro because it counts a hidden bit.
constexpr auto dbl_mantissa = 52;
constexpr auto one_64b = static_cast<uint64_t>(1);
constexpr auto msb_64b = one_64b << 63;

union double_conv {
    double d_val;
    uint64_t i_val;
};

uint64_t double_to_gds(double val) {
    if (val == 0)
        return 0;

    int exp_dbl;
    double_conv frac;
    frac.d_val = std::frexp(val, &exp_dbl);

    // at this point, 0.5 <= |frac| < 1, |frac| * 2^(exp) = |val|

    // get sign bit
    auto frac_bits = frac.i_val;
    auto sgn = frac_bits & msb_64b;

    // get mantissa bits (adding implicit 1), and convert number of
    // bits to gds_mantissa
    constexpr auto tmp_flag = one_64b << dbl_mantissa;
    constexpr auto delta = gds_mantissa - (dbl_mantissa + 1);
    auto mantissa = tmp_flag | (frac_bits & (tmp_flag - 1));

    if constexpr (delta > 0) {
        mantissa <<= delta;
    } else if constexpr (delta < 0) {
        mantissa >>= delta;
    }

    // convert exponent (need to be multiple of 4)
    auto shift_extra = 4 - (exp_dbl & (4 - 1));
    mantissa >>= shift_extra;

    exp_dbl += (256 + shift_extra);
    assert(0 <= exp_dbl && exp_dbl <= gds_exp_max);
    auto exp = (((uint64_t)exp_dbl) >> 2) << gds_mantissa;

    return sgn | exp | mantissa;
}

double gds_to_double(uint64_t val) {
    if (val == 0)
        return 0;

    constexpr auto man_flag = one_64b << gds_mantissa;

    auto sgn = val & msb_64b;
    auto mantissa = val & (man_flag - 1);
    auto exp = static_cast<int64_t>((val & (~msb_64b)) >> gds_mantissa);
    double ans = ((double)mantissa) * std::pow(16.0, exp - 64 - (gds_mantissa / 4));
    if (sgn != 0) {
        ans *= -1;
    }
    return ans;
}

} // namespace gdsii
} // namespace cbag
