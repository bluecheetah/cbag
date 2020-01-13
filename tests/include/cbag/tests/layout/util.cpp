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

#include <cbag/tests/layout/util.h>

std::shared_ptr<const c_tech> make_tech(const std::string &tech_fname) {
    if (tech_fname.empty()) {
        return std::make_shared<const c_tech>(
            cbag::layout::make_tech("tests/data/tech_config/tech_params.yaml"));
    }
    return std::make_shared<const c_tech>(cbag::layout::make_tech(tech_fname));
}

std::tuple<std::shared_ptr<const c_tech>, std::shared_ptr<const c_grid>>
make_tech_grid(const std::string &tech_fname, const std::string &grid_fname) {
    auto tech_info = make_tech(tech_fname);
    auto grid = std::shared_ptr<const c_grid>();
    if (grid_fname.empty()) {
        grid = std::make_shared<const c_grid>(
            cbag::layout::make_grid(tech_info, "tests/data/tech_config/grid.yaml"));
    } else {
        grid = std::make_shared<const c_grid>(cbag::layout::make_grid(tech_info, grid_fname));
    }
    return {std::move(tech_info), std::move(grid)};
}

std::shared_ptr<const cbag::layout::track_coloring> make_tr_colors(const c_tech &tech) {
    return std::make_shared<const cbag::layout::track_coloring>(
        cbag::layout::make_track_coloring(tech));
}

std::shared_ptr<c_cellview>
make_cv(const std::shared_ptr<const c_grid> &grid,
        const std::shared_ptr<const cbag::layout::track_coloring> &tr_colors,
        const std::string &cell_name) {
    if (cell_name.empty()) {
        return std::make_shared<c_cellview>(grid, tr_colors, "CBAG_TEST");
    }
    return std::make_shared<c_cellview>(grid, tr_colors, cell_name);
}
