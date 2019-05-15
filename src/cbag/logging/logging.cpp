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

#include <cbag/common/typedefs.h>
#include <cbag/logging/logging.h>

#include "spdlog/details/signal_handler.h"
#include <spdlog/sinks/rotating_file_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>

namespace cbag {

constexpr cnt_t max_log_size = 1024 * 1024 * 10;
constexpr cnt_t num_log_file = 3;

void init_logging() {
    if (spdlog::get("bag") == nullptr) {
        spdlog::installCrashHandler();

        std::vector<spdlog::sink_ptr> sinks;
        sinks.push_back(std::make_shared<spdlog::sinks::rotating_file_sink_st>(
            "bag.log", max_log_size, num_log_file));
        auto stdout_sink = std::make_shared<spdlog::sinks::stdout_color_sink_st>();
        stdout_sink->set_level(spdlog::level::warn);
        sinks.push_back(std::move(stdout_sink));

        auto logger = std::make_shared<spdlog::logger>("bag", sinks.begin(), sinks.end());
        spdlog::register_logger(logger);

        auto cbag_logger = logger->clone("cbag");
        spdlog::register_logger(cbag_logger);

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
