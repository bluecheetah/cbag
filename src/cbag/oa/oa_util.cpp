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

#include <cbag/logging/spdlog.h>

#include <oa/oaDesignDB.h>

#include <cbag/oa/oa_util.h>

namespace cbagoa {

oa::oaTech *read_tech(const oa::oaNativeNS &ns, const std::string &lib_name) {
    // open technology file
    oa::oaScalarName lib_name_oa = oa::oaScalarName(ns, lib_name.c_str());
    oa::oaTech *tech_ptr = oa::oaTech::find(lib_name_oa);
    if (tech_ptr == nullptr) {
        // opened tech not found, attempt to open
        if (!oa::oaTech::exists(lib_name_oa)) {
            throw std::runtime_error(fmt::format("Cannot find technology library {}, or it has "
                                                 "no valid technology library attached",
                                                 lib_name));
        } else {
            tech_ptr = oa::oaTech::open(lib_name_oa, 'r');
            if (tech_ptr == nullptr) {
                throw std::runtime_error(
                    fmt::format("Cannot open technology library: {}", lib_name));
            }
        }
    }
    return tech_ptr;
}

oa::oaDesign *open_design(const oa::oaNativeNS &ns, spdlog::logger &logger,
                          const std::string &lib_name, const std::string &cell_name,
                          const std::string &view_name, char mode,
                          oa::oaReservedViewTypeEnum view_enum) {
    oa::oaScalarName lib_oa(ns, lib_name.c_str());
    oa::oaScalarName cell_oa(ns, cell_name.c_str());
    oa::oaScalarName view_oa(ns, view_name.c_str());

    logger.info("Opening design {}__{}({}) with mode {}", lib_name, cell_name, view_name, mode);
    oa::oaDesign *dsn_ptr = nullptr;
    if (mode == 'r') {
        dsn_ptr = oa::oaDesign::open(lib_oa, cell_oa, view_oa, mode);
    } else {
        oa::oaViewType *view_type = oa::oaViewType::get(view_enum);
        dsn_ptr = oa::oaDesign::open(lib_oa, cell_oa, view_oa, view_type, mode);
    }
    if (dsn_ptr == nullptr) {
        throw std::invalid_argument(fmt::format("Cannot open cell: {}__{}({}) with mode {}",
                                                lib_name, cell_name, view_name, mode));
    }
    return dsn_ptr;
}

void handle_oa_exceptions(spdlog::logger &logger) {
    logger.error("throwing exception");
    try {
        throw;
    } catch (const oa::oaException &ex) {
        auto msg = fmt::format("OA Exception ID: {}, Msg: {}", ex.getMsgId(), ex.getMsg());
        logger.error(msg);
        throw std::runtime_error(msg);
    } catch (const oa::IException &ex) {
        auto msg = fmt::format("OA IException ID: {}, Msg: {}", ex.getMsgId(), ex.getMsg());
        logger.error(msg);
        throw std::runtime_error(msg);
    }
}

// write all symbol views to file
// get library read access
oa::oaLib *open_library_read(const oa::oaNativeNS &ns, const std::string &lib_name) {
    oa::oaLib *lib_ptr = oa::oaLib::find(oa::oaScalarName(ns, lib_name.c_str()));
    if (lib_ptr == nullptr) {
        throw std::invalid_argument(fmt::format("Cannot find library {}", lib_name));
    }
    if (!lib_ptr->getAccess(oa::oacReadLibAccess, LIB_ACCESS_TIMEOUT)) {
        throw std::runtime_error(fmt::format("Cannot obtain read access to library: {}", lib_name));
    }
    return lib_ptr;
}

} // namespace cbagoa
