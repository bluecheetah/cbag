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

#include <yaml-cpp/tuple.h>

#include <cbag/common/transformation_util.h>
#include <cbag/tests/util/io.h>
#include <cbag/yaml/transformation.h>

TEST_CASE("convert_array", "[transformation]") {
    using data_type =
        std::tuple<cbag::transformation, cbag::cnt_t, cbag::cnt_t, cbag::offset_t, cbag::offset_t,
                   cbag::cnt_t, cbag::cnt_t, cbag::offset_t, cbag::offset_t>;
    auto fname = "tests/data/cbag/common/transformation/convert_array.yaml";

    auto[xform, nx, ny, spx, spy, e_nx, e_ny, e_spx, e_spy] =
        GENERATE_COPY(read_test_vector<data_type>(fname));

    auto[oa_nx, oa_ny, oa_spx, oa_spy] = cbag::convert_array(xform, nx, ny, spx, spy);

    REQUIRE(oa_nx == e_nx);
    REQUIRE(oa_ny == e_ny);
    REQUIRE(oa_spx == e_spx);
    REQUIRE(oa_spy == e_spy);
}

TEST_CASE("convert_gds_array", "[transformation]") {
    using data_type =
        std::tuple<cbag::transformation, cbag::cnt_t, cbag::cnt_t, cbag::offset_t, cbag::offset_t>;
    auto fname = "tests/data/cbag/common/transformation/convert_gds_array.yaml";

    auto[xform, nx, ny, spx, spy] = GENERATE_COPY(read_test_vector<data_type>(fname));

    auto[oa_nx, oa_ny, oa_spx, oa_spy] = cbag::convert_array(xform, nx, ny, spx, spy);
    auto[nx2, ny2, spx2, spy2] = cbag::convert_gds_array(xform, oa_nx, oa_ny, oa_spx, oa_spy);

    REQUIRE(nx2 == nx);
    REQUIRE(ny2 == ny);
    REQUIRE(spx2 == spx);
    REQUIRE(spy2 == spy);
}
