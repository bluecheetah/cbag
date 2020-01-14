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

#include <sstream>

#include <catch2/catch.hpp>

#include <yaml-cpp/tuple.h>

#include <cbag/common/transformation_util.h>
#include <cbag/gdsii/read.h>
#include <cbag/gdsii/read_util.h>
#include <cbag/gdsii/write.h>
#include <cbag/gdsii/write_util.h>
#include <cbag/logging/logging.h>
#include <cbag/tests/util/io.h>
#include <cbag/yaml/transformation.h>

#include <cbag/tests/layout/util.h>

TEST_CASE("Read/write 64-bit integers", "[gds]") {
    using data_type = uint64_t;
    auto fname = "tests/data/cbag/gdsii/io/int.yaml";

    auto val = GENERATE_COPY(read_test_vector<data_type>(fname));

    std::stringstream stream;

    cbag::gdsii::write_bytes(stream, val);
    auto ans = cbag::gdsii::read_bytes<data_type>(stream);
    REQUIRE(ans == val);
}

TEST_CASE("Read/write transformation objects", "[gds]") {
    using data_type = cbag::transformation;
    auto fname = "tests/data/cbag/gdsii/io/xform.yaml";

    auto xform = GENERATE_COPY(read_test_vector<data_type>(fname));

    auto logger = cbag::get_cbag_logger();
    auto stream = std::stringstream();

    cbag::gdsii::write_transform(*logger, stream, xform, 1);
    auto ans = cbag::gdsii::read_transform(*logger, stream);

    CAPTURE(ans, xform);

    REQUIRE(ans == xform);
}

TEST_CASE("Read/write arrayed instances", "[gds]") {
    using data_type =
        std::tuple<cbag::transformation, cbag::cnt_t, cbag::cnt_t, cbag::offset_t, cbag::offset_t>;
    auto fname = "tests/data/cbag/gdsii/io/arr_inst.yaml";
    auto lay_map = "tests/data/test_gds/gds.layermap";
    auto obj_map = "tests/data/test_gds/gds.objectmap";
    auto lib_name = "GDS_TEST";
    auto master_name = "arr_inst";
    auto expect_name = "top_cv";
    auto inst_name = "XDUT";

    auto[xform, nx, ny, spx, spy] = GENERATE_COPY(read_test_vector<data_type>(fname));

    CAPTURE(xform, nx, ny, spx, spy);

    auto[tech_info, grid] = make_tech_grid();
    auto tr_colors = make_tr_colors(*tech_info);
    auto master = make_cv(grid, tr_colors, master_name);
    auto expect = make_cv(grid, tr_colors, expect_name);

    auto lay = tech_info->get_lay_purp_list(1)[0];
    master->add_shape(lay, cbag::box_t(0, 0, 10, 50));
    auto inst = cbag::layout::instance(inst_name, master, xform, nx, ny, spx, spy);
    expect->add_object(inst);

    auto sstream = std::stringstream();
    auto cv_list = std::vector<std::pair<std::string, std::shared_ptr<const c_cellview>>>();
    cv_list.reserve(2);
    cv_list.emplace_back(master_name, master);
    cv_list.emplace_back(expect_name, expect);

    cbag::gdsii::implement_gds_stream(sstream, lib_name, lay_map, obj_map, cv_list);
    sstream.flush();

    auto cv_read_list = std::vector<std::shared_ptr<const c_cellview>>();
    cv_read_list.reserve(2);
    cbag::gdsii::read_gds_stream(sstream, lay_map, obj_map, grid, tr_colors,
                                 std::back_inserter(cv_read_list));
    REQUIRE(cv_read_list.size() == 2);

    auto &cv_read = *(cv_read_list[1]);
    REQUIRE(cv_read.begin_inst() != cv_read.end_inst());

    auto &inst_read = cv_read.begin_inst()->second;
    REQUIRE(inst_read.get_inst_name() == inst_name);
    REQUIRE(inst_read.xform == inst.xform);
    REQUIRE(inst_read.nx == inst.nx);
    REQUIRE(inst_read.ny == inst.ny);
    REQUIRE(inst_read.spx == inst.spx);
    REQUIRE(inst_read.spy == inst.spy);

    REQUIRE(cv_read == *expect);
}
