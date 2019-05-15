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

#include <algorithm>

#include <fmt/core.h>

#include <yaml-cpp/yaml.h>

#include "yaml-cpp/unordered_map.h"

#include <cbag/layout/lp_lookup.h>

namespace cbag {
namespace layout {

lp_lookup::lp_lookup() = default;

lp_lookup::lp_lookup(const YAML::Node &parent)
    : lay_map(parent["layer"].as<lay_map_t>()), purp_map(parent["purpose"].as<purp_map_t>()) {

    auto def_purp = parent["default_purpose"].as<std::string>();
    auto pin_purp = parent["pin_purpose"].as<std::string>();

    auto purp_iter = purp_map.find(def_purp);
    if (purp_iter == purp_map.end()) {
        throw std::out_of_range(fmt::format("Cannot find default purpose: {}", def_purp));
    } else {
        default_purpose = purp_iter->second;
    }
    purp_iter = purp_map.find(pin_purp);
    if (purp_iter == purp_map.end()) {
        throw std::out_of_range(fmt::format("Cannot find pin purpose: {}", pin_purp));
    } else {
        pin_purpose = purp_iter->second;
    }
}

purp_t lp_lookup::get_default_purpose() const { return default_purpose; }

purp_t lp_lookup::get_pin_purpose() const { return pin_purpose; }

const std::string &lp_lookup::get_layer_name(lay_t lay_id) const {
    auto iter = std::find_if(
        lay_map.begin(), lay_map.end(),
        [&lay_id](const std::pair<std::string, purp_t> &v) { return lay_id == v.second; });
    return iter->first;
}

const std::string &lp_lookup::get_purpose_name(purp_t purp_id) const {
    auto iter = std::find_if(
        purp_map.begin(), purp_map.end(),
        [&purp_id](const std::pair<std::string, purp_t> &v) { return purp_id == v.second; });
    return iter->first;
}

std::optional<lay_t> lp_lookup::get_layer_id(const std::string &layer) const {
    auto iter = lay_map.find(layer);
    if (iter == lay_map.end())
        return {};
    return iter->second;
}

std::optional<purp_t> lp_lookup::get_purpose_id(const std::string &purpose) const {
    if (purpose.empty()) {
        return default_purpose;
    }
    auto iter = purp_map.find(purpose);
    if (iter == purp_map.end())
        return {};
    return iter->second;
}

} // namespace layout
} // namespace cbag
