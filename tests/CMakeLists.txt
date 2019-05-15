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

find_package(Catch2 REQUIRED CONFIG)

add_executable(cbag_test
  ${CMAKE_CURRENT_SOURCE_DIR}/main.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/cbag/common/transformation.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/cbag/gdsii/gds_lookup.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/cbag/gdsii/io.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/cbag/gdsii/math.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/cbag/layout/cellview.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/cbag/layout/path.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/cbag/layout/grid.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/cbag/layout/tech.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/cbag/layout/track_coloring.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/cbag/layout/track_info.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/cbag/layout/via.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/cbag/netlist/output.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/cbag/schematic/cellview.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/cbag/spirit/name.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/cbag/util/interval.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/cbag/util/io.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/cbag/util/sorted_map.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/cbag/util/sorted_vector.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/cbag/util/string.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/cbag/oa/oa_io.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/yaml-cpp/tuple.cpp
  )

target_include_directories(cbag_test
  PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  )

target_link_libraries(cbag_test
  PUBLIC
  cbag
  fmt::fmt-header-only
  yaml-cpp
  Catch2::Catch2
  )

install(TARGETS
  cbag_test
  RUNTIME DESTINATION ${CBAG_BINARY_DIR}
  LIBRARY DESTINATION ${CBAG_LIBRARY_DIR}
  )
