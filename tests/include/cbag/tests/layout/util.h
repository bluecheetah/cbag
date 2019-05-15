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

#ifndef CBAG_TEST_LAYOUT_UTIL
#define CBAG_TEST_LAYOUT_UTIL

#include <memory>
#include <tuple>

#include <cbag/layout/routing_grid_util.h>
#include <cbag/layout/tech_util.h>

using c_tech = cbag::layout::tech;
using c_grid = cbag::layout::routing_grid;

inline std::shared_ptr<const c_tech> make_tech() {
    return std::make_shared<const c_tech>(
        cbag::layout::make_tech("tests/data/test_layout/tech_params.yaml"));
}

inline std::tuple<std::shared_ptr<const c_tech>, std::shared_ptr<const c_grid>> make_tech_grid() {
    auto tech_info = make_tech();
    auto grid = std::make_shared<const c_grid>(
        cbag::layout::make_grid(tech_info, "tests/data/test_layout/grid.yaml"));
    return {std::move(tech_info), std::move(grid)};
}

#endif
