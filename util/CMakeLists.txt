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

add_executable(write_tech_info
  ${CMAKE_CURRENT_SOURCE_DIR}/write_tech_info.cpp
  )

# shared library dependencies
target_link_libraries(write_tech_info
  PUBLIC
  cbag
  )

add_executable(write_cv_yaml
  ${CMAKE_CURRENT_SOURCE_DIR}/write_cv_yaml.cpp
  )

# shared library dependencies
target_link_libraries(write_cv_yaml
  PUBLIC
  cbag
  )

add_executable(print_cv_props
  ${CMAKE_CURRENT_SOURCE_DIR}/print_cv_props.cpp
  )

# shared library dependencies
target_link_libraries(print_cv_props
  PUBLIC
  cbag
  )

add_executable(print_gds
  ${CMAKE_CURRENT_SOURCE_DIR}/print_gds.cpp
  )

# shared library dependencies
target_link_libraries(print_gds
  PUBLIC
  cbag
  )

add_executable(write_gds_rects
  ${CMAKE_CURRENT_SOURCE_DIR}/write_gds_rects.cpp
  )

# shared library dependencies
target_link_libraries(write_gds_rects
  PUBLIC
  cbag
  )


install(TARGETS
  write_tech_info write_cv_yaml
  RUNTIME DESTINATION ${CBAG_BINARY_DIR}
  LIBRARY DESTINATION ${CBAG_LIBRARY_DIR}
  )
