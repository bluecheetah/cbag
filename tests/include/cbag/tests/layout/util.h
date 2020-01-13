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
#include <vector>

#include <cbag/layout/cellview_util.h>
#include <cbag/layout/routing_grid_util.h>
#include <cbag/layout/tech_util.h>

using c_tech = cbag::layout::tech;
using c_grid = cbag::layout::routing_grid;
using c_cellview = cbag::layout::cellview;

std::shared_ptr<const c_tech> make_tech(const std::string &tech_fname = "");

std::tuple<std::shared_ptr<const c_tech>, std::shared_ptr<const c_grid>>
make_tech_grid(const std::string &tech_fname = "", const std::string &grid_fname = "");

std::shared_ptr<const cbag::layout::track_coloring> make_tr_colors(const c_tech &tech);

std::shared_ptr<c_cellview>
make_cv(const std::shared_ptr<const c_grid> &grid,
        const std::shared_ptr<const cbag::layout::track_coloring> &tr_colors,
        const std::string &cell_name = "");
#endif
