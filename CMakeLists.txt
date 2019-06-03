# SPDX-License-Identifier: BSD-3-Clause AND Apache-2.0
# Copyright 2018 Regents of the University of California
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# * Redistributions of source code must retain the above copyright notice, this
#   list of conditions and the following disclaimer.
#
# * Redistributions in binary form must reproduce the above copyright notice,
#   this list of conditions and the following disclaimer in the documentation
#   and/or other materials provided with the distribution.
#
# * Neither the name of the copyright holder nor the names of its
#   contributors may be used to endorse or promote products derived from
#   this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

# Copyright 2019 Blue Cheetah Analog Design Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

cmake_minimum_required(VERSION 3.13)
project(cbag)

# Boost Spirit X3 requires C++14
# std::variant requires C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# control compiler warnings
add_compile_options(-fexceptions)
add_compile_options(-fmax-errors=2)
add_compile_options(-g)
add_compile_options(-Wall)
add_compile_options(-Wno-delete-non-virtual-dtor)
# add_compile_options(-Wno-logical-op-parentheses)
# add_compile_options(-Wno-new-returns-null)
# set optimzation level for release
set(CMAKE_CXX_FLAGS_RELEASE "-O3")
# generate compilation commands file for emacs
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
# prefer pthreads
set(THREADS_PREFER_PTHREAD_FLAG ON)

# make sure linker raise errors if shared library has undefined symbols
# this makes it a lot easier to debug
set(CMAKE_SHARED_LINKER_FLAGS "-Wl,--no-undefined")
# add rpaths to the final install executable
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
# check for link time optimization
include(CheckIPOSupported)

check_ipo_supported(RESULT ipo_result)
if(ipo_result AND CMAKE_BUILD_TYPE MATCHES Release)
  message("Enabling interprocedural optimization")
  set(CMAKE_INTERPROCEDURAL_OPTIMIZATION TRUE)
else()
  message("Interprocedural optimization disabled")
endif()

# Must use GNUInstallDirs to install libraries into correct
# locations on all platforms.
include(GNUInstallDirs)

# Include Boost as an imported target
set(Boost_USE_STATIC_LIBS ON)
find_package(Boost REQUIRED)

message("Boost include directory: " ${Boost_INCLUDE_DIRS})
message("Boost library directory: " ${Boost_LIBRARY_DIRS})
message("Boost libraries: " ${Boost_LIBRARIES})

# Include fmt library
find_package(fmt REQUIRED CONFIG)

# Include spdlog
set(SPDLOG_FMT_EXTERNAL ON CACHE BOOL "Use external fmt library instead of bundled")
set(SPDLOG_FMT_HEADER_ONLY ON CACHE BOOL "Use header-only variant of external fmt library")
find_package(spdlog REQUIRED CONFIG)

# Include threads for spdlog
# find_package(Threads REQUIRED)

# Include yaml-cpp
find_package(yaml-cpp REQUIRED CONFIG)

# Include cbag_polygon
add_subdirectory(cbag_polygon)

# set variable storing binary/library location
message("install prefix: " ${CMAKE_INSTALL_PREFIX})
set(CBAG_BINARY_DIR ${CMAKE_INSTALL_PREFIX}/cbag/bin)
set(CBAG_LIBRARY_DIR ${CMAKE_INSTALL_PREFIX}/cbag/lib)
message("install rpath: " ${CMAKE_INSTALL_RPATH})

set(SRC_FILES_LIB_CBAG
  ${CMAKE_CURRENT_SOURCE_DIR}/src/cbag/common/transformation_util.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/cbag/gdsii/math.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/cbag/gdsii/parse_map.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/cbag/gdsii/read.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/cbag/gdsii/read_util.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/cbag/gdsii/write.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/cbag/gdsii/write_util.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/cbag/layout/blockage.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/cbag/layout/boundary.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/cbag/layout/cellview.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/cbag/layout/cellview_poly.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/cbag/layout/cellview_util.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/cbag/layout/grid_object.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/cbag/layout/instance.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/cbag/layout/label.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/cbag/layout/len_info.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/cbag/layout/lp_lookup.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/cbag/layout/path_util.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/cbag/layout/pin.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/cbag/layout/routing_grid.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/cbag/layout/routing_grid_util.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/cbag/layout/tech.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/cbag/layout/tech_util.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/cbag/layout/track_coloring.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/cbag/layout/track_info.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/cbag/layout/track_info_util.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/cbag/layout/vector45.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/cbag/layout/via.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/cbag/layout/via_info.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/cbag/layout/via_lookup.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/cbag/layout/via_param.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/cbag/layout/via_param_util.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/cbag/layout/via_util.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/cbag/layout/via_wrapper.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/cbag/layout/wire_width.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/cbag/logging/logging.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/cbag/netlist/cdl.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/cbag/netlist/core.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/cbag/netlist/lstream.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/cbag/netlist/netlist.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/cbag/netlist/spectre.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/cbag/netlist/verilog.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/cbag/schematic/arc.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/cbag/schematic/cellview.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/cbag/schematic/cellview_info.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/cbag/schematic/donut.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/cbag/schematic/ellipse.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/cbag/schematic/eval_text.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/cbag/schematic/instance.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/cbag/schematic/line.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/cbag/schematic/path.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/cbag/schematic/pin_figure.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/cbag/schematic/pin_object.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/cbag/schematic/polygon.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/cbag/schematic/rectangle.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/cbag/schematic/shape_base.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/cbag/schematic/term_attr.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/cbag/schematic/text_base.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/cbag/schematic/text_t.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/cbag/spirit/ast.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/cbag/spirit/name.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/cbag/spirit/name_unit.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/cbag/spirit/range.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/cbag/util/io.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/cbag/util/name_convert.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/cbag/util/string.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/cbag/yaml/cellviews.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/cbag/yaml/datatypes.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/cbag/yaml/figures.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/cbag/yaml/len_info.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/cbag/yaml/shapes.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/cbag/yaml/via_info.cpp
  )

set(SRC_FILES_LIB_CBAG_OA
  ${CMAKE_CURRENT_SOURCE_DIR}/src/cbag/oa/oa_database.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/cbag/oa/oa_read.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/cbag/oa/oa_read_lib.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/cbag/oa/oa_util.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/cbag/oa/oa_write.cpp
  )


if(DEFINED ENV{OA_LINK_DIR})
message("OA include directory: " $ENV{OA_INCLUDE_DIR})
message("OA link directory: " $ENV{OA_LINK_DIR})

add_library(cbag ${SRC_FILES_LIB_CBAG} ${SRC_FILES_LIB_CBAG_OA})
target_link_directories(cbag
  PUBLIC
  $ENV{OA_LINK_DIR}
  )
target_include_directories(cbag
  PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${Boost_INCLUDE_DIRS}
  $ENV{OA_INCLUDE_DIR}
  )
target_link_libraries(cbag
  PUBLIC
  cbag_polygon
  fmt::fmt-header-only
  oaBase
  oaCommon
  oaPlugIn
  PRIVATE
  stdc++fs
  ${Boost_LIBRARIES}
  yaml-cpp
  oaDM
  oaDMFileSysBase
  oaTech
  oaDesign
  ${CMAKE_DL_LIBS}
  )

else()
message("WARNING: building without OpenAccess support.")

# setup global include directories and link libraries
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include
  ${Boost_INCLUDE_DIRS}
  )

add_library(cbag ${SRC_FILES_LIB_CBAG})
target_include_directories(cbag
  PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${Boost_INCLUDE_DIRS}
  )
target_link_libraries(cbag
  PUBLIC
  cbag_polygon
  fmt::fmt-header-only
  PRIVATE
  stdc++fs
  ${Boost_LIBRARIES}
  yaml-cpp
  )

endif()

# add utility binaries
add_subdirectory(util EXCLUDE_FROM_ALL)

# add tests binaries
add_subdirectory(tests EXCLUDE_FROM_ALL)
