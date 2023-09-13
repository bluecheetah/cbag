#include <cbag/layout/path.h>

namespace cbag {
namespace layout {

path::path(layer_t lay_purp, offset_t width, std::vector<point_t> pt_vec,
           enum_t path_type, offset_t begin_extn, offset_t end_extn) noexcept
           : lay_purp(std::move(lay_purp)), path_type(std::move(path_type)), width(width),
             begin_extn(begin_extn), end_extn(end_extn), pt_vec(pt_vec) {}

layer_t path::get_layer_t() const { return lay_purp; }

enum_t path::get_path_type() const { return path_type; }

offset_t path::get_width() const noexcept { return width; }

offset_t path::get_begin_extn() const noexcept { return begin_extn; }
offset_t path::get_end_extn() const noexcept { return end_extn; }

std::vector<point_t> path::get_pt_vec() const { return pt_vec; }

bool path::operator==(const path &rhs) const noexcept {
    return lay_purp == rhs.lay_purp && path_type == rhs.path_type && width == rhs.width &&
           begin_extn == rhs.begin_extn && end_extn == rhs.end_extn && pt_vec == rhs.pt_vec;
}

}  // namespace layout
} // namespace cbag
