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

#include <fstream>
#include <tuple>

#include <cbag/logging/logging.h>

#include <yaml-cpp/yaml.h>

#include <cbag/util/sorted_map.h>

#include <cbag/layout/cellview.h>
#include <cbag/schematic/cellview.h>
#include <cbag/schematic/instance.h>
#include <cbag/util/io.h>

#include <cbag/oa/database.h>
#include <cbag/oa/read.h>
#include <cbag/oa/read_lib.h>
#include <cbag/oa/util.h>
#include <cbag/oa/write.h>
#include <cbag/oa/write_lib.h>

namespace cbagoa {

using oa_lay_purp_t = std::pair<oa::oaLayerNum, oa::oaPurposeNum>;
using oa_via_lay_purp_t = std::tuple<oa_lay_purp_t, oa_lay_purp_t, oa_lay_purp_t>;

LibDefObserver::LibDefObserver(oa::oaUInt4 priority)
    : oa::oaObserver<oa::oaLibDefList>(priority, true) {}

oa::oaBoolean LibDefObserver::onLoadWarnings(oa::oaLibDefList *obj, const oa::oaString &msg,
                                             oa::oaLibDefListWarningTypeEnum type) {

    err_msg = "OA Error: " + std::string(msg);
    return true;
}

database::database(std::string lib_def_fname)
    : lib_def_file_(std::move(lib_def_fname)), logger(cbag::get_cbag_logger()) {
    try {
        oaDesignInit(oacAPIMajorRevNumber, oacAPIMinorRevNumber, oacDataModelRevNumber);

        logger->info("Creating new database with file: {}", lib_def_file_);
        oa::oaLibDefList::openLibs(lib_def_file_.c_str());

        if (!lib_def_obs_.err_msg.empty()) {
            throw std::runtime_error(lib_def_obs_.err_msg);
        }

    } catch (...) {
        handle_oa_exceptions(*logger);
    }
}

database::~database() {
    try {
        logger->info("Closing all OA libraries from definition file: {}", lib_def_file_);
        oa::oaIter<oa::oaLib> lib_iter(oa::oaLib::getOpenLibs());
        oa::oaLib *lib_ptr;
        while ((lib_ptr = lib_iter.getNext()) != nullptr) {
            oa::oaString tmp_str;
            lib_ptr->getName(ns, tmp_str);
            lib_ptr->close();
        }
    } catch (...) {
        handle_oa_exceptions(*logger);
    }
}

void database::add_yaml_path(const std::string &lib_name, std::string yaml_path) {
    yaml_path_map.insert_or_assign(lib_name, std::move(yaml_path));
}

void database::add_primitive_lib(std::string lib_name) {
    primitive_libs.insert(std::move(lib_name));
}

bool database::is_primitive_lib(const std::string &lib_name) const {
    return primitive_libs.find(lib_name) != primitive_libs.end();
}

std::vector<std::string> database::get_cells_in_lib(const std::string &lib_name) const {
    std::vector<std::string> ans;
    cbagoa::get_cells(ns_native, *logger, lib_name, std::back_inserter(ans));
    return ans;
}

std::string database::get_lib_path(const std::string &lib_name) const {
    std::string ans;
    try {
        oa::oaScalarName lib_name_oa = oa::oaScalarName(ns_native, lib_name.c_str());
        oa::oaLib *lib_ptr = oa::oaLib::find(lib_name_oa);
        if (lib_ptr == nullptr) {
            throw std::invalid_argument(fmt::format("Cannot find library {}", lib_name));
        }
        oa::oaString tmp_str;
        lib_ptr->getFullPath(tmp_str);
        return {tmp_str};
    } catch (...) {
        handle_oa_exceptions(*logger);
    }
    // should never get here
    return "";
}

void database::create_lib(const std::string &lib_name, const std::string &lib_path,
                          const std::string &tech_lib) const {
    try {
        logger->info("Creating OA library {}", lib_name);

        // open library
        oa::oaScalarName lib_name_oa = oa::oaScalarName(ns_native, lib_name.c_str());
        oa::oaLib *lib_ptr = oa::oaLib::find(lib_name_oa);
        if (lib_ptr == nullptr) {
            // append library name to lib_path
            auto new_lib_path = cbag::util::join(lib_path, lib_name);
            cbag::util::make_parent_dirs(new_lib_path);

            // create new library
            logger->info("Creating library {} at path {}, with tech lib {}", lib_name,
                         new_lib_path.c_str(), tech_lib);

            oa::oaScalarName oa_tech_lib(ns_native, tech_lib.c_str());
            lib_ptr = oa::oaLib::create(lib_name_oa, new_lib_path.c_str());
            oa::oaTech::attach(lib_ptr, oa_tech_lib);

            // NOTE: I cannot get open access to modify the library file, so
            // we just do it by hand.
            std::ofstream outfile;
            outfile.open(lib_def_file_, std::ios_base::app);
            outfile << "DEFINE " << lib_name << " " << new_lib_path.c_str() << std::endl;
            outfile.close();

            // Create cdsinfo.tag file
            new_lib_path /= "cdsinfo.tag";
            outfile.open(new_lib_path.c_str(), std::ios_base::out);
            outfile << "CDSLIBRARY" << std::endl;
            outfile << "NAMESPACE LibraryUnix" << std::endl;
            outfile.close();
        } else {
            logger->info("Library already exists, do nothing.");
        }
    } catch (...) {
        handle_oa_exceptions(*logger);
    }
}

cbag::sch::cellview database::read_sch_cellview(const std::string &lib_name,
                                                const std::string &cell_name,
                                                const std::string &view_name) const {
    try {
        return cbagoa::read_sch_cellview(ns_native, ns, *logger, lib_name, cell_name, view_name,
                                         primitive_libs);
    } catch (...) {
        handle_oa_exceptions(*logger);
        throw;
    }
}

std::vector<cell_key_t> database::read_sch_recursive(const std::string &lib_name,
                                                     const std::string &cell_name,
                                                     const std::string &view_name) const {
    std::vector<cell_key_t> ans;
    cbagoa::read_sch_recursive(ns_native, ns, *logger, lib_name, cell_name, view_name,
                               yaml_path_map, primitive_libs, std::back_inserter(ans));
    return ans;
}

std::vector<cell_key_t> database::read_library(const std::string &lib_name,
                                               const std::string &view_name) const {
    std::vector<cell_key_t> ans;
    cbagoa::read_library(ns_native, ns, *logger, lib_name, view_name, yaml_path_map, primitive_libs,
                         std::back_inserter(ans));
    return ans;
}

void database::write_sch_cellview(const std::string &lib_name, const std::string &cell_name,
                                  const std::string &view_name, bool is_sch,
                                  const cbag::sch::cellview &cv,
                                  const str_map_t *rename_map) const {
    try {
        cbagoa::write_sch_cellview(ns_native, ns, *logger, lib_name, cell_name, view_name, is_sch,
                                   cv, rename_map);
    } catch (...) {
        handle_oa_exceptions(*logger);
    }
}

void database::write_lay_cellview(const std::string &lib_name, const std::string &cell_name,
                                  const std::string &view_name, const cbag::layout::cellview &cv,
                                  oa::oaTech *tech, const str_map_t *rename_map) const {
    try {
        cbagoa::write_lay_cellview(ns_native, ns, *logger, lib_name, cell_name, view_name, cv, tech,
                                   rename_map);
    } catch (...) {
        handle_oa_exceptions(*logger);
    }
}

void database::implement_sch_list(const std::string &lib_name, const std::string &sch_view,
                                  const std::string &sym_view,
                                  const std::vector<sch_cv_info> &cv_list) const {
    cbagoa::implement_sch_list<std::vector<sch_cv_info>>(ns_native, ns, *logger, lib_name, sch_view,
                                                         sym_view, cv_list);
}

void database::implement_lay_list(const std::string &lib_name, const std::string &view,
                                  const std::vector<lay_cv_info> &cv_list) const {
    cbagoa::implement_lay_list<std::vector<lay_cv_info>>(ns_native, ns, *logger, lib_name, view,
                                                         cv_list);
} // namespace cbagoa

void database::write_tech_info_file(const std::string &fname, const std::string &tech_lib,
                                    const std::string &pin_purpose, const std::string &label_purpose) const {
    oa::oaTech *tech_ptr = read_tech(ns_native, tech_lib);

    // read layer/purpose/via mappings
    cbag::util::sorted_map<oa::oaLayerNum, std::string> lay_map;
    cbag::util::sorted_map<oa::oaPurposeNum, std::string> pur_map;
    cbag::util::sorted_map<oa_via_lay_purp_t, std::string> via_map;

    oa::oaString tmp;
    oa::oaIter<oa::oaLayer> lay_iter(tech_ptr->getLayers());
    oa::oaLayer *lay;
    while ((lay = lay_iter.getNext()) != nullptr) {
        lay->getName(tmp);
        lay_map.emplace(lay->getNumber(), std::string(tmp));
    }

    oa::oaIter<oa::oaPurpose> pur_iter(tech_ptr->getPurposes());
    oa::oaPurpose *pur;
    while ((pur = pur_iter.getNext()) != nullptr) {
        pur->getName(tmp);
        pur_map.emplace(pur->getNumber(), std::string(tmp));
    }

    oa::oaIter<oa::oaViaDef> via_iter(tech_ptr->getViaDefs());
    oa::oaStdViaDef *via;
    auto def_purp = oa::oaPurpose::get(tech_ptr, oa::oacDrawingPurposeType)->getNumber();
    while ((via = reinterpret_cast<oa::oaStdViaDef *>(via_iter.getNext())) != nullptr) {
        via->getName(tmp);
        oa::oaViaParam via_params;
        via->getParams(via_params);
        via_map.emplace(std::make_tuple(std::make_pair(via->getLayer1Num(), def_purp),
                                        std::make_pair(via_params.getCutLayer(), def_purp),
                                        std::make_pair(via->getLayer2Num(), def_purp)),
                        std::string(tmp));
    }

    // emit to YAML
    YAML::Emitter out;
    out.SetSeqFormat(YAML::Flow);
    out << YAML::BeginMap;

    out << YAML::Key << "options" << YAML::Value;
    out << YAML::BeginMap;
    out << YAML::Key << "default_purpose" << YAML::Value << "drawing";
    out << YAML::Key << "pin_purpose" << YAML::Value << pin_purpose;
    out << YAML::Key << "label_purpose" << YAML::Value << label_purpose;
    out << YAML::Key << "make_pin_obj" << YAML::Value << true;
    out << YAML::EndMap;

    out << YAML::Key << "layer" << YAML::Value;
    out << YAML::BeginMap;
    for (auto const & [ lay_id, lay_name ] : lay_map) {
        out << YAML::Key << lay_name << YAML::Value << lay_id;
    }
    out << YAML::EndMap;

    out << YAML::Key << "purpose" << YAML::Value;
    out << YAML::BeginMap;
    for (auto const & [ purp_id, purp_name ] : pur_map) {
        out << YAML::Key << purp_name << YAML::Value << purp_id;
    }
    out << YAML::EndMap;

    out << YAML::Key << "via_layers" << YAML::Value;
    out << YAML::BeginMap;
    for (auto const & [ v_lay_purp_tuple, via_id ] : via_map) {
        auto & [ lay1_key, cut_key, lay2_key ] = v_lay_purp_tuple;
        out << YAML::Key << via_id << YAML::Value;
        out << YAML::Block << YAML::BeginSeq;
        out << YAML::Flow << YAML::BeginSeq << lay1_key.first << lay1_key.second << YAML::EndSeq
            << YAML::Block;
        out << YAML::Flow << YAML::BeginSeq << cut_key.first << cut_key.second << YAML::EndSeq
            << YAML::Block;
        out << YAML::Flow << YAML::BeginSeq << lay2_key.first << lay2_key.second << YAML::EndSeq
            << YAML::Block;
        out << YAML::EndSeq << YAML::Flow;
    }
    out << YAML::EndMap;

    out << YAML::EndMap;

    // write to file
    std::ofstream out_file = cbag::util::open_file_write(fname);
    out_file << out.c_str();
    out_file.close();
}

} // namespace cbagoa
