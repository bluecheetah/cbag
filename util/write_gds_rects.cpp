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

#include <iostream>
#include <vector>

#include <yaml-cpp/yaml.h>

#include <cbag/common/box_t.h>
#include <cbag/gdsii/write.h>
#include <cbag/gdsii/write_util.h>
#include <cbag/logging/logging.h>
#include <cbag/util/io.h>
#include <cbag/yaml/rectangle_data.h>

void write_rects(const std::string &yaml_fname, const std::string &out_fname) {
    auto lib_name = "LIB_TEST";
    auto cell_name = "TOP_TEST";
    auto resolution = 1e-9;
    auto user_unit = 1e-6;
    auto lay_id = 100;
    auto purp_id = 0;

    auto node = YAML::LoadFile(yaml_fname);
    auto rect_vec = node.as<std::vector<cbag::box_t>>();

    auto &logger = *cbag::get_cbag_logger();
    auto stream = cbag::util::open_file_write(out_fname, true);
    auto time_vec = cbag::gdsii::get_gds_time();

    cbag::gdsii::write_gds_start(logger, stream, lib_name, resolution, user_unit, time_vec);
    cbag::gdsii::write_struct_begin(logger, stream, time_vec);
    cbag::gdsii::write_struct_name(logger, stream, cell_name);

    for (const auto &r : rect_vec) {
        cbag::gdsii::write_box(logger, stream, lay_id, purp_id, r);
    }

    cbag::gdsii::write_struct_end(logger, stream);
    cbag::gdsii::write_gds_stop(logger, stream);
    stream.close();
}

int main(int argc, char *argv[]) {
    if (argc != 3) {
        std::cout << "Usage: write_gds_rects <yaml_fname> <out_fname>" << std::endl;
    } else {
        write_rects(argv[1], argv[2]);
    }

    return 0;
}
