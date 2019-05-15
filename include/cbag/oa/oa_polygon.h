#ifndef CBAG_OA_OA_POLYGON_H
#define CBAG_OA_OA_POLYGON_H

#include <cbag/polygon/point_concept.h>
#include <cbag/polygon/tag.h>

#include <oa/oaDesignDB.h>

#if __has_include(<cbag/oa/oa_color.h>)
#include <cbag/oa/oa_color.h>
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
