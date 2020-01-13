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

#include <unordered_set>

#include <boost/container_hash/hash.hpp>

#include <yaml-cpp/yaml.h>

#include <cbag/gdsii/main.h>
#include <cbag/gdsii/read.h>
#include <cbag/gdsii/read_util.h>
#include <cbag/gdsii/write.h>
#include <cbag/gdsii/write_util.h>

#include <cbag/logging/logging.h>

namespace cbag {
namespace gdsii {

bool gds_equal(const std::string &lhs_file, const std::string &rhs_file) {

    if (lhs_file == rhs_file)
        return true;

    auto lhs_stream = util::open_file_read(lhs_file, true);
    auto rhs_stream = util::open_file_read(rhs_file, true);

    auto &logger = *(get_cbag_logger());
    auto lhs_lib = read_gds_start(logger, lhs_stream);
    auto rhs_lib = read_gds_start(logger, rhs_stream);
    if (lhs_lib != rhs_lib)
        return false;

    auto is_done = false;
    std::size_t max_iter = 1000000000;
    std::size_t ridx = 0;
    for (; !is_done && ridx < max_iter; ++ridx) {
        auto[lhs_rtype, lhs_size] = read_record_header(lhs_stream);
        auto[rhs_rtype, rhs_size] = read_record_header(rhs_stream);
        if (lhs_rtype != rhs_rtype || lhs_size != rhs_size) {
            lhs_stream.close();
            rhs_stream.close();
            return false;
        }
        if (lhs_rtype == record_type::ENDLIB)
            return true;

        if (lhs_rtype == record_type::BGNSTR) {
            // do not compare contents of BGNSTR, as it's just
            // modification time
            lhs_stream.ignore(lhs_size);
            rhs_stream.ignore(rhs_size);
        } else {
            auto lim = lhs_size / 2;
            for (std::size_t idx = 0; idx < lim; ++idx) {
                auto lhs_val = read_bytes<uint16_t>(lhs_stream);
                auto rhs_val = read_bytes<uint16_t>(rhs_stream);
                if (lhs_val != rhs_val)
                    return false;
            }
        }
    }

    if (ridx == max_iter) {
        logger.warn("GDS reading finished due to maximum number of records read.");
    }

    lhs_stream.close();
    rhs_stream.close();
    return true;
}

} // namespace gdsii
} // namespace cbag
