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

#ifndef TESTS_UTIL_IO_H
#define TESTS_UTIL_IO_H

#include <filesystem>
#include <fstream>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include <catch2/catch.hpp>

#include <yaml-cpp/yaml.h>

inline std::string read_file(const std::string &fname) {
    std::ifstream in(fname);
    std::stringstream stream;
    stream << in.rdbuf();
    return stream.str();
}

template <typename T> class yaml_generator : public Catch::Generators::IGenerator<T> {
  private:
    std::vector<T> data_;
    std::size_t idx_;

  public:
    yaml_generator(const std::string &fname) : idx_(0) {
        auto node = YAML::LoadFile(fname);
        data_ = node.as<std::vector<T>>();
    }

    const T &get() const override { return data_[idx_]; }
    bool next() override { return (++idx_) < data_.size(); }
};

template <typename T>
Catch::Generators::GeneratorWrapper<T> read_test_vector(const std::string &fname) {
    return Catch::Generators::GeneratorWrapper<T>(
        std::unique_ptr<Catch::Generators::IGenerator<T>>(new yaml_generator<T>(fname)));
}

class dir_generator : public Catch::Generators::IGenerator<std::string> {
  private:
    std::vector<std::string> data_;
    std::size_t idx_;

  public:
    dir_generator(const std::string &root_dir) : idx_(0) {
        auto dir_path = std::filesystem::path(root_dir);
        if (std::filesystem::is_directory(dir_path)) {
            std::filesystem::directory_iterator stop;
            for (std::filesystem::directory_iterator itr(dir_path); itr != stop; ++itr) {
                if (std::filesystem::is_directory(itr->status())) {
                    data_.push_back(itr->path().filename());
                }
            }
        }
    }

    const std::string &get() const override { return data_[idx_]; }
    bool next() override { return (++idx_) < data_.size(); }
};

inline Catch::Generators::GeneratorWrapper<std::string>
read_directories(const std::string &root_dir) {
    return Catch::Generators::GeneratorWrapper<std::string>(
        std::unique_ptr<Catch::Generators::IGenerator<std::string>>(new dir_generator(root_dir)));
}

#endif
