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

#include <catch2/catch.hpp>

#include <filesystem>

#include <cbag/util/io.h>

namespace fs = std::filesystem;

TEST_CASE("can open file for read when there's no write permission", "[io]") {
    constexpr auto default_perms = fs::perms::owner_read | fs::perms::owner_write |
                                   fs::perms::group_read | fs::perms::others_read;
    auto fname = std::string("tests/data/test_io/io.txt");

    // set permission to no write
    auto fpath = fs::path(fname);
    fs::permissions(fpath, fs::perms::owner_read);
    auto stream = cbag::util::open_file_read(fname);
    auto result = stream.fail();
    stream.close();
    fs::permissions(fpath, default_perms);

    REQUIRE(result == false);
}

TEST_CASE("throw exception when reading file with no read permission", "[io]") {
    constexpr auto default_perms = fs::perms::owner_read | fs::perms::owner_write |
                                   fs::perms::group_read | fs::perms::others_read;
    auto fname = std::string("tests/data/test_io/io.txt");

    // set permission to no write
    auto fpath = fs::path(fname);
    fs::permissions(fpath, fs::perms::owner_write);
    auto throws_exception = false;
    try {
        auto stream = cbag::util::open_file_read(fname);
        stream.close();
    } catch (...) {
        throws_exception = true;
    }
    fs::permissions(fpath, default_perms);

    REQUIRE(throws_exception == true);
}

TEST_CASE("throw exception when writing file with no write permission", "[io]") {
    constexpr auto default_perms = fs::perms::owner_read | fs::perms::owner_write |
                                   fs::perms::group_read | fs::perms::others_read;
    auto fname = std::string("tests/data/test_io/io.txt");

    // set permission to no write
    auto fpath = fs::path(fname);
    fs::permissions(fpath, fs::perms::owner_read);
    auto throws_exception = false;
    try {
        auto stream = cbag::util::open_file_write(fname);
        stream.close();
    } catch (...) {
        throws_exception = true;
    }
    fs::permissions(fpath, default_perms);

    REQUIRE(throws_exception == true);
}
