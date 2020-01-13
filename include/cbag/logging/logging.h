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

#ifndef CBAG_LOGGING_LOGGING_H
#define CBAG_LOGGING_LOGGING_H

#include <memory>
#include <string>
#include <unordered_map>

#include <cbag/common/typedefs.h>
#include <cbag/logging/spdlog.h>

#include <spdlog/sinks/rotating_file_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>

namespace cbag {

class file_logger {
  public:
    using file_sink_t = spdlog::sinks::rotating_file_sink_mt;
    using stdout_sink_t = spdlog::sinks::stdout_color_sink_mt;

  private:
    std::shared_ptr<spdlog::logger> ptr_;
    std::string log_basename_;

    static constexpr cnt_t max_log_size_ = 1024 * 1024 * 10;
    static constexpr cnt_t num_log_file_ = 3;
    static std::unordered_map<std::string, std::shared_ptr<file_sink_t>> sink_map_;

  public:
    file_logger(const std::string &name, const std::string &log_file, int stdout_level);

    int level() const;
    int flush_level() const;
    const std::string &log_basename() const noexcept;

    void trace(const std::string &msg);
    void debug(const std::string &msg);
    void info(const std::string &msg);
    void warn(const std::string &msg);
    void error(const std::string &msg);
    void critical(const std::string &msg);
    void log(int level, const std::string &msg);
    void flush();
    void set_level(int level);
    void flush_on(int level);
};

void init_logging();

std::shared_ptr<spdlog::logger> get_bag_logger();

std::shared_ptr<spdlog::logger> get_cbag_logger();

} // namespace cbag

#endif
