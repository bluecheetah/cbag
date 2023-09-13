#ifndef CBAG_LAYOUT_PATH_H
#define CBAG_LAYOUT_PATH_H

#include <vector>

#include <cbag/common/typedefs.h>
#include <cbag/common/layer_t.h>
#include <cbag/common/point_t.h>

namespace cbag {
namespace layout {

class path {
    private:
        layer_t lay_purp;
        enum_t path_type;
        offset_t width;
        offset_t begin_extn;
        offset_t end_extn;
        std::vector<point_t> pt_vec;

    public:
        explicit path(layer_t lay_purp, offset_t width, std::vector<point_t> pt_vec,
                      enum_t path_type = 0, offset_t begin_extn = 0, offset_t end_extn = 0) noexcept;

        layer_t get_layer_t() const;
        enum_t get_path_type() const;
        offset_t get_width() const noexcept;
        offset_t get_begin_extn() const noexcept;
        offset_t get_end_extn() const noexcept;
        std::vector<point_t> get_pt_vec() const;

        bool operator==(const path &rhs) const noexcept;
};

} // namespace layout
} // namespace cbag

#endif
