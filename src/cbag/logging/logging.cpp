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

#include <vector>

#include <cbag/logging/logging.h>
#include <cbag/util/io.h>

#include "spdlog/details/signal_handler.h"

namespace cbag {

std::unordered_map<std::string, std::shared_ptr<file_logger::file_sink_t>> file_logger::sink_map_ =
    {};

file_logger::file_logger(const std::string &name, const std::string &log_file, int stdout_level) {
    if (!cbag::util::is_file(log_file)) {
        auto ostream = cbag::util::open_file_write(log_file, false);
        ostream.close();
    }
    auto log_path = cbag::util::get_canonical_path(log_file);
    log_basename_ = log_path.string();

    auto test = spdlog::get(name);
    if (test) {
        ptr_ = test;
    } else {
        auto iter = sink_map_.find(log_basename_);

        std::shared_ptr<file_sink_t> file_sink = nullptr;
        if (iter == sink_map_.end()) {
            file_sink = std::make_shared<file_sink_t>(log_basename_, max_log_size_, num_log_file_);
            sink_map_.emplace(log_basename_, file_sink);
        } else {
            file_sink = iter->second;
        }
        auto stdout_sink = std::make_shared<stdout_sink_t>();
        stdout_sink->set_level(static_cast<spdlog::level::level_enum>(stdout_level));

        auto sinks = std::vector<spdlog::sink_ptr>();
        sinks.reserve(2);
        sinks.push_back(std::move(file_sink));
        sinks.push_back(std::move(stdout_sink));
        ptr_ = std::make_shared<spdlog::logger>(name, sinks.begin(), sinks.end());

        spdlog::register_logger(ptr_);
    }
}

int file_logger::level() const { return static_cast<int>(ptr_->level()); }
int file_logger::flush_level() const { return static_cast<int>(ptr_->flush_level()); }
const std::string &file_logger::log_basename() const noexcept { return log_basename_; }

void file_logger::trace(const std::string &msg) { ptr_->trace(msg); }
void file_logger::debug(const std::string &msg) { ptr_->debug(msg); }
void file_logger::info(const std::string &msg) { ptr_->info(msg); }
void file_logger::warn(const std::string &msg) { ptr_->warn(msg); }
void file_logger::error(const std::string &msg) { ptr_->error(msg); }
void file_logger::critical(const std::string &msg) { ptr_->critical(msg); }
void file_logger::log(int level, const std::string &msg) {
    ptr_->log(static_cast<spdlog::level::level_enum>(level), msg);
}
void file_logger::flush() { ptr_->flush(); }
void file_logger::set_level(int level) {
    ptr_->set_level(static_cast<spdlog::level::level_enum>(level));
}
void file_logger::flush_on(int level) {
    ptr_->flush_on(static_cast<spdlog::level::level_enum>(level));
}

void init_logging() {
    if (!spdlog::get("bag")) {
        spdlog::installCrashHandler();

        auto code = static_cast<int>(spdlog::level::warn);
        auto bag_logger = file_logger("bag", "bag.log", code);
        auto cbag_logger = file_logger("cbag", "bag.log", code);

        spdlog::flush_on(spdlog::level::warn);
    }
}

std::shared_ptr<spdlog::logger> get_bag_logger() {
    init_logging();
    return spdlog::get("bag");
}

std::shared_ptr<spdlog::logger> get_cbag_logger() {
    init_logging();
    return spdlog::get("cbag");
}

} // namespace cbag
