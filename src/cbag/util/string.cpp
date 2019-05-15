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

#include <cstdlib>
#include <regex>

#include <cbag/util/string.h>

namespace cbag {
namespace util {

token_iterator::token_iterator(const std::string &text, const char *separators) {
    boost::char_separator<char> sep(separators);
    tokens = std::make_unique<tok_container_t>(text, sep);
    cur = tokens->begin();
    end = tokens->end();
}

bool token_iterator::has_next() const { return cur != end; }

std::string token_iterator::get_next() {
    auto ans = *cur;
    ++cur;
    return ans;
}

// replaces all strings inside ${} by the corresponding environment variable
// (the brackets must be present).
std::string expand_env(const std::string &text) {
    // get regex object
    // matches ${...}
    auto env_regex = std::regex("\\$\\{([^}]+)\\}");
    auto iter = std::sregex_iterator(text.begin(), text.end(), env_regex);
    auto stop = std::sregex_iterator();
    auto ans = std::string();
    ans.reserve(text.size());
    auto last_pos = static_cast<std::size_t>(0);
    auto bgn = text.begin();
    for (; iter != stop; ++iter) {
        auto match = *iter;
        auto s = std::getenv(match.str(1).c_str());
        auto pos = match.position(0);
        ans.append(bgn + last_pos, bgn + pos);
        if (s)
            ans.append(s);
        last_pos = pos + match.length(0);
    }
    ans.append(bgn + last_pos, text.end());
    return ans;
}

} // namespace util
} // namespace cbag
