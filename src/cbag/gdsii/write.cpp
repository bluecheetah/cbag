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

#include <fmt/core.h>

#include <cbag/common/box_t.h>
#include <cbag/common/transformation_util.h>
#include <cbag/gdsii/parse_map.h>
#include <cbag/gdsii/write.h>
#include <cbag/gdsii/write_util.h>
#include <cbag/layout/cellview.h>
#include <cbag/layout/polygons.h>
#include <cbag/layout/via_util.h>

#include <cbag/util/io.h>

namespace cbag {
namespace gdsii {

class rect_writer {
  private:
    spdlog::logger &logger;
    std::ostream &stream;
    glay_t layer;
    gpurp_t purpose;
    int scale;

  public:
    rect_writer(spdlog::logger &logger, std::ostream &stream, glay_t layer, gpurp_t purpose,
                int scale)
        : logger(logger), stream(stream), layer(layer), purpose(purpose), scale(scale) {}

    rect_writer &operator=(const box_t &box) {
        write_box(logger, stream, layer, purpose, box, scale);
        return *this;
    }

    rect_writer &operator*() { return *this; }

    rect_writer &operator++() { return *this; }
};

gds_lookup::gds_lookup(const layout::tech &tech, const std::string &lay_map_file,
                       const std::string &obj_map_file) {
    process_layer_map(lay_map_file, tech, [this](layer_t &&lay, gds_layer_t &&glay) {
        lay_map_.emplace(std::move(lay), std::move(glay));
    });

    process_object_map(obj_map_file, tech,
                       [this](boundary_type btype, gds_layer_t &&glay) {
                           bnd_map_.emplace(btype, std::move(glay));
                       },
                       [this](blockage_t &&btype, gds_layer_t &&glay) {
                           blk_map_.emplace(std::move(btype), std::move(glay));
                       });
}

std::optional<gds_layer_t> gds_lookup::get_gds_layer(layer_t key) const {
    auto iter = lay_map_.find(key);
    if (iter == lay_map_.end())
        return {};
    return iter->second;
}

std::optional<gds_layer_t> get_gds_layer(const gds_lookup &lookup, lay_t lay, purp_t purp) {
    return lookup.get_gds_layer(std::make_pair(lay, purp));
}

std::optional<gds_layer_t> gds_lookup::get_gds_layer(boundary_type bnd_type) const {
    auto iter = bnd_map_.find(bnd_type);
    if (iter == bnd_map_.end())
        return {};
    return iter->second;
}

std::optional<gds_layer_t> gds_lookup::get_gds_layer(lay_t lay_id, blockage_type blk_type) const {
    auto iter = blk_map_.find(std::make_pair(lay_id, blk_type));
    if (iter == blk_map_.end())
        return {};
    return iter->second;
}

std::vector<tval_t> get_gds_time() {
    auto ep_time = std::time(nullptr);
    auto loc_time = std::localtime(&ep_time);
    return {
        static_cast<tval_t>(loc_time->tm_year), static_cast<tval_t>(loc_time->tm_mon + 1),
        static_cast<tval_t>(loc_time->tm_mday), static_cast<tval_t>(loc_time->tm_hour),
        static_cast<tval_t>(loc_time->tm_min),  static_cast<tval_t>(loc_time->tm_sec),
    };
}

void write_gds_start(spdlog::logger &logger, std::ostream &stream, const std::string &lib_name,
                     double resolution, double user_unit, const std::vector<tval_t> &time_vec) {
    write_header(logger, stream);
    write_lib_begin(logger, stream, time_vec);
    write_lib_name(logger, stream, lib_name);
    write_units(logger, stream, resolution, user_unit);
}

void write_gds_stop(spdlog::logger &logger, std::ostream &stream) { write_lib_end(logger, stream); }

void write_lay_geometry(spdlog::logger &logger, std::ostream &stream, glay_t lay, gpurp_t purp,
                        const layout::poly_set_t &geo, int scale) {
    auto lambda = [&logger, &stream, lay, purp, scale](const auto &v) {
        write_polygon(logger, stream, lay, purp, v, scale);
    };
    polygon::apply_polygons<coord_t, decltype(lambda), layout::poly_t>(geo, std::move(lambda));
}

void write_lay_via(spdlog::logger &logger, std::ostream &stream, const layout::tech &tech,
                   const gds_lookup &lookup, const layout::via &v, int scale) {
    auto[lay1_key, cut_key, lay2_key] = tech.get_via_layer_purpose(v.get_via_id());
    auto gkey1 = lookup.get_gds_layer(lay1_key);
    if (!gkey1) {
        logger.warn("Cannot find layer/purpose ({}, {}) in layer map.  Skipping via.",
                    lay1_key.first, lay1_key.second);
        return;
    }
    auto gkey2 = lookup.get_gds_layer(lay2_key);
    if (!gkey2) {
        logger.warn("Cannot find layer/purpose ({}, {}) in layer map.  Skipping via.",
                    lay2_key.first, lay2_key.second);
        return;
    }
    auto gkeyc = lookup.get_gds_layer(cut_key);
    if (!gkeyc) {
        logger.warn("Cannot find layer/purpose ({}, {}) in layer map.  Skipping via.",
                    cut_key.first, cut_key.second);
        return;
    }
    auto[glay1, gpurp1] = *gkey1;
    auto[gcl, gcp] = *gkeyc;
    auto[glay2, gpurp2] = *gkey2;
    write_box(logger, stream, glay1, gpurp1, layout::get_bot_box(v), scale);
    write_box(logger, stream, glay2, gpurp2, layout::get_top_box(v), scale);
    get_via_cuts(v, rect_writer(logger, stream, gcl, gcp, scale));
}

void write_lay_pin(spdlog::logger &logger, std::ostream &stream, glay_t lay, gpurp_t purp, gpurp_t purp_l,
                   const layout::pin &pin, bool make_pin_obj, double resolution, int scale) {
    auto box = pin.bbox();
    if (!is_physical(box)) {
        logger.warn("non-physical bbox {} on pin layer ({}, {}), skipping.", to_string(box), lay,
                    purp);
        return;
    }
    auto xc = xm(box);
    auto yc = ym(box);
    auto w = width(box);
    auto text_h = height(box);
    transformation xform;
    if (text_h > w) {
        xform = transformation(xc, yc, orientation::R90);
        text_h = w;
    } else {
        xform = transformation(xc, yc, orientation::R0);
    }

    write_text(logger, stream, lay, purp_l, pin.label(), xform, text_h, resolution, scale);
    if (make_pin_obj) {
        write_box(logger, stream, lay, purp, box, scale);
    }
}

void write_lay_label(spdlog::logger &logger, std::ostream &stream, const layout::label &lab,
                     double resolution, int scale) {
    auto[lay, purp] = lab.get_key();
    write_text(logger, stream, lay, purp, lab.get_text(), lab.get_xform(), lab.get_height(),
               resolution, scale);
}

void write_lay_cellview(spdlog::logger &logger, std::ostream &stream, const std::string &cell_name,
                        const cbag::layout::cellview &cv,
                        const std::unordered_map<std::string, std::string> &rename_map,
                        const std::vector<tval_t> &time_vec, const gds_lookup &lookup, int scale) {
    write_struct_begin(logger, stream, time_vec);
    write_struct_name(logger, stream, cell_name);

    logger.info("Export layout instances.");
    for (auto iter = cv.begin_inst(); iter != cv.end_inst(); ++iter) {
        auto & [ inst_name, inst ] = *iter;
        write_instance(logger, stream, inst.get_cell_name(&rename_map), inst_name, inst.xform,
                       scale, inst.nx, inst.ny, inst.spx, inst.spy);
    }

    logger.info("Export layout geometries.");
    for (auto iter = cv.begin_geometry(); iter != cv.end_geometry(); ++iter) {
        auto & [ layer_key, geo ] = *iter;
        auto gkey = lookup.get_gds_layer(layer_key);
        if (!gkey) {
            logger.warn("Cannot find layer/purpose ({}, {}) in layer map.  Skipping geometry.",
                        layer_key.first, layer_key.second);
        } else {
            auto[glay, gpurp] = *gkey;
            write_lay_geometry(logger, stream, glay, gpurp, geo, scale);
        }
    }

    logger.info("Export layout vias.");
    auto tech_ptr = cv.get_tech();
    auto resolution = tech_ptr->get_resolution();
    for (auto iter = cv.begin_via(); iter != cv.end_via(); ++iter) {
        write_lay_via(logger, stream, *tech_ptr, lookup, *iter, scale);
    }

    logger.info("Export layout pins.");
    auto purp = tech_ptr->get_pin_purpose();
    auto purp_l = tech_ptr->get_label_purpose();
    auto make_pin_obj = tech_ptr->get_make_pin();
    for (auto iter = cv.begin_pin(); iter != cv.end_pin(); ++iter) {
        auto & [ lay, pin_list ] = *iter;
        auto gkey = get_gds_layer(lookup, lay, purp);
        auto gkey_l = get_gds_layer(lookup, lay, purp_l);
        if (!gkey) {
            logger.warn("Cannot find layer/purpose ({}, {}) in layer map.  Skipping pins.", lay,
                        purp);
        } else if (!gkey_l) {
            logger.warn("Cannot find layer/purpose ({}, {}) in layer map.  Skipping labels.", lay,
                        purp_l);
        } else {
            auto[glay, gpurp] = *gkey;
            auto[glay_l, gpurp_l] = *gkey_l;
            for (const auto &pin : pin_list) {
                write_lay_pin(logger, stream, glay, gpurp, gpurp_l, pin, make_pin_obj, resolution, scale);
            }
        }
    }

    logger.info("Export layout labels.");
    for (auto iter = cv.begin_label(); iter != cv.end_label(); ++iter) {
        write_lay_label(logger, stream, *iter, resolution, scale);
    }

    logger.info("Export layout boundaries.");
    auto pr_prop_str = std::string("oaBoundary:pr");
    for (auto iter = cv.begin_boundary(); iter != cv.end_boundary(); ++iter) {
        auto btype = iter->get_type();
        auto gkey = lookup.get_gds_layer(btype);
        if (!gkey) {
            logger.warn("Cannot find boundary type {} in object map.  Skipping boundary.", btype);
        } else {
            auto[glay, gpurp] = *gkey;
            if (btype == boundary_type::PR) {
                // write OA boundary property string
                write_polygon(logger, stream, glay, gpurp, *iter, scale, &pr_prop_str);
            } else {
                write_polygon(logger, stream, glay, gpurp, *iter, scale);
            }
        }
    }

    logger.info("Export layer blockages.");
    for (auto iter = cv.begin_lay_block(); iter != cv.end_lay_block(); ++iter) {
        for (const auto &blockage : iter->second) {
            auto btype = blockage.get_type();
            auto lay_id = blockage.get_layer();
            auto gkey = lookup.get_gds_layer(lay_id, btype);
            if (!gkey) {
                logger.warn(
                    "Cannot find layer blockage type {} on layer {} in object map.  Skipping.",
                    static_cast<enum_t>(btype), lay_id);
            } else {
                auto[glay, gpurp] = *gkey;
                write_polygon(logger, stream, glay, gpurp, blockage, scale);
            }
        }
    }

    write_struct_end(logger, stream);

    logger.info("Finish GDS export.");
}

} // namespace gdsii
} // namespace cbag
