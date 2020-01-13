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

void remove_gds_labels(const std::string &in_fname, const std::string &out_fname,
                       const std::string &specs_fname, const std::string &lay_map,
                       const std::string &obj_map, const layout::tech &tech) {
    if (in_fname == out_fname) {
        throw std::runtime_error("Cannot read and write to the same GDS file.");
    }

    auto &logger = *(get_cbag_logger());
    logger.info("Beginning removing labels from {}, saving to {}", in_fname, out_fname);

    auto lookup = gds_lookup(tech, lay_map, obj_map);
    auto node = YAML::LoadFile(specs_fname);
    auto lay_set = std::unordered_set<gds_layer_t, boost::hash<gds_layer_t>>();
    for (const auto &item : node) {
        auto lay_str = item[0].as<std::string>();
        auto purp_str = item[1].as<std::string>();
        auto lay_id_opt = tech.get_layer_id(lay_str);
        auto purp_id_opt = tech.get_purpose_id(purp_str);
        if (lay_id_opt && purp_id_opt) {
            auto gds_opt = lookup.get_gds_layer(layer_t{*lay_id_opt, *purp_id_opt});
            if (gds_opt) {
                lay_set.emplace(*gds_opt);
            } else {
                logger.warn("Layer ({}, {}) has no gds mapping, skipping.", lay_str, purp_str);
            }
        } else {
            logger.warn("Unknown layer ({}, {}), skipping.", lay_str, purp_str);
        }
    }

    auto istream = util::open_file_read(in_fname, true);
    auto ostream = util::open_file_write(out_fname, true);
    auto is_done = false;
    std::size_t max_iter = 1000000000;
    std::size_t ridx = 0;
    for (; !is_done && ridx < max_iter; ++ridx) {
        auto[rtype, rsize] = read_record_header(istream);
        logger.info("Reading GDS record {} with size: {}", to_string(rtype), rsize);
        if (rsize & 1) {
            throw std::runtime_error("Have odd number of bytes in record.");
        }
        auto lim = rsize / 2;
        switch (rtype) {
        case record_type::ENDLIB: {
            write_lib_end(logger, ostream);
            is_done = true;
            break;
        }
        case record_type::TEXT: {
            auto[lay_purp, xform, text, mag] = read_text(logger, istream);
            auto iter = lay_set.find(lay_purp);
            if (iter == lay_set.end()) {
                // write this text
                write_text(logger, ostream, std::get<0>(lay_purp), std::get<1>(lay_purp), text,
                           xform, 1, mag);
            }
            break;
        }
        default: {
            write_bytes(ostream, static_cast<uint16_t>(rsize + 4));
            write_bytes(ostream, static_cast<uint16_t>(rtype));
            for (std::size_t idx = 0; idx < lim; ++idx) {
                auto val = read_bytes<uint16_t>(istream);
                write_bytes(ostream, val);
            }
        }
        }
    }

    if (ridx == max_iter) {
        logger.warn("GDS reading finished due to maximum number of records read.");
    }

    ostream.close();
    istream.close();
    logger.info("GDS label removal done.");
}

} // namespace gdsii
} // namespace cbag
