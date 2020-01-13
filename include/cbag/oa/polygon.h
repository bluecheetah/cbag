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

#ifndef CBAG_OA_POLYGON_H
#define CBAG_OA_POLYGON_H

#include <cbag/polygon/point_concept.h>
#include <cbag/polygon/tag.h>

#include <oa/oaDesignDB.h>

#if __has_include(<cbag/oa/color.h>)

#include <cbag/oa/color.h>

#else

namespace cbagoa {

oa::oaIntAppDef<oa::oaShape> *get_color_def_ptr() { return nullptr; }

void set_color(oa::oaIntAppDef<oa::oaShape> *ptr, oa::oaShape *shape, oa::oaByte color) {}

} // namespace cbagoa

#endif

namespace spdlog {
class logger;
}

namespace cbagoa {

class oa_polygon {
  public:
    oa::oaPointArray pt_arr;

    oa_polygon() = default;

    const oa::oaPoint *begin() const { return pt_arr.getElements(); }
    const oa::oaPoint *end() const { return pt_arr.getElements() + pt_arr.getNumElements(); }

    std::size_t size() const { return pt_arr.getNumElements(); }

    void scale_by(int scale) {
        for (std::size_t idx = 0; idx < pt_arr.getSize(); ++idx) {
            pt_arr[idx].set(pt_arr[idx].x() * scale, pt_arr[idx].y() * scale);
        }
    }
};

} // namespace cbagoa

namespace cbag {
namespace polygon {

template <> struct tag<oa::oaPoint> { using type = point_tag; };

template <> struct point_traits<oa::oaPoint> {
    using point_type = oa::oaPoint;
    using coordinate_type = oa::oaCoord;

    static coordinate_type get(const point_type &point, orientation_2d orient) {
        return (orient == orientation_2d::X) ? point.x() : point.y();
    }

    static void set(point_type &point, orientation_2d orient, coordinate_type value) {
        if (orient == orientation_2d::X) {
            point.x() = value;
        } else {
            point.y() = value;
        }
    }

    static point_type construct(coordinate_type x, coordinate_type y) { return point_type{x, y}; }
};

template <> struct tag<cbagoa::oa_polygon> { using type = polygon_tag; };

template <> struct polygon_traits<cbagoa::oa_polygon> {
    using polygon_type = cbagoa::oa_polygon;
    using coordinate_type = oa::oaCoord;
    using point_type = oa::oaPoint;
    using iterator_type = const point_type *;

    static iterator_type begin_points(const polygon_type &t) { return t.begin(); }

    static iterator_type end_points(const polygon_type &t) { return t.end(); }

    static std::size_t size(const polygon_type &t) { return t.pt_arr.getNumElements(); }

    static void set_points(polygon_type &t, std::vector<point_type> &&data) {
        set_points(t, data.begin(), data.end(), data.size());
    }

    template <typename iT, IsPoint<typename iT::value_type> = 0>
    static void set_points(polygon_type &t, iT start, iT stop, std::size_t n = 3) {
        t.pt_arr.setNumElements(0);
        for (; start != stop; ++start) {
            t.pt_arr.append(oa::oaPoint(x(*start), y(*start)));
        }
    }

    template <typename iT, IsPoint<typename iT::value_type> = 0>
    static polygon_type construct(iT start, iT stop, std::size_t n = 3) {
        auto ans = polygon_type();
        set_points(ans, start, stop, n);
        return ans;
    }
};

} // namespace polygon
} // namespace cbag

#endif
