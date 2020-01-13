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

#ifndef CBAG_LAYOUT_CONVEX_H
#define CBAG_LAYOUT_CONVEX_H

#include <algorithm>
#include <array>

#include <cbag/polygon/polygon_45_data.h>
#include <cbag/polygon/polygon_set_data.h>

#include <cbag/common/box_t.h>
#include <cbag/common/point_t.h>

namespace cbag {
namespace layout {
namespace convex {

using poly_45_t = cbag::polygon::polygon_45_data<cbag::coord_t>;

inline std::array<cbag::coord_t, 2> minmax(cbag::coord_t a, cbag::coord_t b) {
    auto tmp = std::minmax(a, b);
    return {{tmp.first, tmp.second}};
}

inline std::array<bool, 2> _get_xy_flag(int code) {
    return {{(code & 2) != 0, static_cast<bool>(code & 1)}};
}

template <int C> void _update_stairs_box(cbag::box_t &box, cbag::coord_t xp, cbag::coord_t yp) {
    auto[y_flag, x_flag] = _get_xy_flag(C);
    auto xb = box[0][x_flag];
    auto yb = box[1][y_flag];
    if ((xp - xb) * (2 * x_flag - 1) > 0) {
        box[0][x_flag] = xp;
        box[1][y_flag ^ 1] = yp;
    } else if (xp == xb) {
        box[1][y_flag ^ 1] = minmax(yp, box[1][y_flag ^ 1])[y_flag];
    }
    if ((yp - yb) * (2 * y_flag - 1) > 0) {
        box[0][x_flag ^ 1] = xp;
        box[1][y_flag] = yp;
    } else if (yp == yb) {
        box[0][x_flag ^ 1] = minmax(xp, box[0][x_flag ^ 1])[x_flag];
    }
}

template <typename Iter> std::array<cbag::box_t, 4> get_stairs_bbox(Iter iter, Iter stop) {
    if (iter == stop)
        throw std::runtime_error("No points given.");

    auto xp = x(*iter);
    auto yp = y(*iter);
    auto box_arr = std::array<cbag::box_t, 4>();
    box_arr[0] = cbag::box_t(xp, yp, xp, yp);
    box_arr[1] = box_arr[0];
    box_arr[2] = box_arr[0];
    box_arr[3] = box_arr[0];
    ++iter;
    for (; iter != stop; ++iter) {
        xp = x(*iter);
        yp = y(*iter);
        _update_stairs_box<0>(box_arr[0], xp, yp);
        _update_stairs_box<1>(box_arr[1], xp, yp);
        _update_stairs_box<2>(box_arr[2], xp, yp);
        _update_stairs_box<3>(box_arr[3], xp, yp);
    }

    return box_arr;
}

inline std::vector<point_t> get_stairs(std::vector<point_t> data, int code) {
    // sort data
    auto[y_flag, x_flag] = _get_xy_flag(code);
    auto x_inc = !x_flag;
    auto y_inc = !y_flag;

    std::sort(data.begin(), data.end(), [x_inc, y_inc](const point_t &a, const point_t &b) {
        if (x_inc) {
            return (a[0] < b[0]) ||
                   ((a[0] == b[0]) && ((y_inc && a[1] < b[1]) || (!y_inc && a[1] > b[1])));
        } else {
            return (a[0] > b[0]) ||
                   ((a[0] == b[0]) && ((y_inc && a[1] < b[1]) || (!y_inc && a[1] > b[1])));
        }
    });

    auto ans = std::vector<point_t>();
    ans.reserve(data.size());

    // get first point
    auto iter = data.begin();
    auto stop = data.end();
    auto xp = (*iter)[0];
    auto yp = (*iter)[1];
    ans.emplace_back(xp, yp);

    while (iter != stop && (*iter)[0] == xp)
        ++iter;

    // get stairs
    while (iter != stop) {
        auto x = (*iter)[0];
        auto y = (*iter)[1];
        if ((y_inc && y < yp) || (!y_inc && y > yp)) {
            // found next right stair vertex
            ans.emplace_back(x, yp);
            ans.emplace_back(x, y);
            xp = x;
            yp = y;

            while (iter != stop && (*iter)[0] == xp)
                ++iter;
        } else {
            ++iter;
        }
    }
    return ans;
}

template <bool R> void _insert_pts(std::vector<point_t> &ans, const std::vector<point_t> &pts) {
    if (R) {
        auto inc = static_cast<int>(ans.back() == pts.back());
        ans.insert(ans.end(), pts.rbegin() + inc, pts.rend());
    } else {
        auto inc = static_cast<int>(ans.back() == pts.front());
        ans.insert(ans.end(), pts.begin() + inc, pts.end());
    }
}

template <typename Iter> poly_45_t get_cr_convex_hull(Iter start, Iter stop) {
    // get bounding box for the 4 corners
    auto box_arr = get_stairs_bbox(start, stop);

    // sort points into stair boxes
    auto pts_arr = std::array<std::vector<point_t>, 4>();
    for (; start != stop; ++start) {
        auto pt = point_t(x(*start), y(*start));
        if (contains(box_arr[0], pt)) {
            pts_arr[0].push_back(pt);
        }
        if (contains(box_arr[1], pt)) {
            pts_arr[1].push_back(pt);
        }
        if (contains(box_arr[2], pt)) {
            pts_arr[2].push_back(pt);
        }
        if (contains(box_arr[3], pt)) {
            pts_arr[3].push_back(pt);
        }
    }
    // get stair case points
    pts_arr[0] = get_stairs(std::move(pts_arr[0]), 0);
    pts_arr[1] = get_stairs(std::move(pts_arr[1]), 1);
    pts_arr[2] = get_stairs(std::move(pts_arr[2]), 2);
    pts_arr[3] = get_stairs(std::move(pts_arr[3]), 3);

    // stitch stair case together
    auto &ans = pts_arr[0];
    ans.reserve(ans.size() + pts_arr[1].size() + pts_arr[2].size() + pts_arr[3].size());
    _insert_pts<true>(ans, pts_arr[1]);
    _insert_pts<false>(ans, pts_arr[3]);
    _insert_pts<true>(ans, pts_arr[2]);

    return poly_45_t(std::move(ans));
}

} // namespace convex
} // namespace layout
} // namespace cbag

#endif
