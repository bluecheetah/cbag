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

#include <catch2/catch.hpp>

#include <cbag/gdsii/math.h>

TEST_CASE("No precision lost in gds <-> double conversion", "[gds]") {
    double value = GENERATE(values<double>({
        2.65732619e5,
        -2.65732619e5,
        13.7,
        1.6,
        1.0,
        0.0,
        0.1,
        1e-3,
        1.54e-5,
        1e-6,
        1e-9,
        -1.54e-5,
        -1e-6,
        -1e-9,
        90.0,
        180.0,
        270.0,
    }));

    // round trip gds conversion
    auto gds_code = cbag::gdsii::double_to_gds(value);
    auto val2 = cbag::gdsii::gds_to_double(gds_code);
    REQUIRE(value == val2);

    // make sure really bit-by-bit matches
    // uint64_t val_bits = *reinterpret_cast<uint64_t *>(&value);
    // uint64_t val2_bits = *reinterpret_cast<uint64_t *>(&val2);
    // REQUIRE(val_bits == val2_bits);
}

TEST_CASE("double -> gds conversion on specific cases", "[gds]") {
    std::pair<double, uint64_t> value = GENERATE(values<std::pair<double, uint64_t>>({
        {5e-4, 0x3e20c49ba5e353f8},
        {5e-10, 0x39225c17d04dad2a},
        {-5e-4, 0xbe20c49ba5e353f8},
        {-5e-10, 0xb9225c17d04dad2a},
        {0, 0x0000000000000000},
        {1.0, 0x4110000000000000},
        {10.0, 0x420a000000000000},
        {-3.0, 0xc130000000000000},
        {1e3, 0x433e800000000000},
        {1e5, 0x45186a0000000000},
        {-1e5, 0xc5186a0000000000},
    }));

    auto gds_code = cbag::gdsii::double_to_gds(value.first);
    REQUIRE(gds_code == value.second);
}
