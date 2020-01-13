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

#include <cbag/netlist/lstream.h>

namespace cbag {
namespace netlist {

lstream::back_inserter::back_inserter(lstream *stream) : stream_(stream) {}

lstream::back_inserter &lstream::back_inserter::operator*() { return *this; }
lstream::back_inserter &lstream::back_inserter::operator++() { return *this; }
lstream::back_inserter &lstream::back_inserter::operator=(std::string name) {
    (*stream_) << std::move(name);
    return *this;
}

lstream::lstream() {}

bool lstream::empty() const { return tokens.empty(); }

lstream::back_inserter lstream::get_back_inserter() { return back_inserter(this); }

lstream &lstream::append_last(const char *seq) {
    tokens.back().append(seq);
    return *this;
}

lstream &lstream::append_last(const std::string &seq) {
    tokens.back().append(seq);
    return *this;
}

lstream &lstream::operator<<(const std::string &token) {
    tokens.push_back(token);
    return *this;
}

lstream &lstream::operator<<(std::string &&token) {
    tokens.push_back(std::move(token));
    return *this;
}

lstream &lstream::operator<<(const char *token) {
    tokens.emplace_back(token);
    return *this;
}

lstream &lstream::operator<<(const std::vector<std::string> &tokens_) {
    tokens.insert(tokens.end(), tokens_.begin(), tokens_.end());
    return *this;
}

lstream &lstream::operator<<(std::vector<std::string> &&tokens_) {
    tokens.insert(tokens.end(), std::make_move_iterator(tokens_.begin()),
                  std::make_move_iterator(tokens_.end()));
    return *this;
}

struct lstream::helper {
    static void to_file_helper(const lstream &line, nstream_output &stream, bool newline,
                               int tab_size, bool break_before, size_t ncol, const char *cnt_str) {
        auto iter = line.tokens.begin();
        auto stop = line.tokens.end();
        if (iter == stop) {
            return;
        }
        stream << *iter;
        size_t cur_col = iter->size();
        ++iter;
        for (; iter != stop; ++iter) {
            size_t n = iter->size();
            size_t cur_len = (break_before) ? n + 3 : n + 1;
            if (cur_col + cur_len <= ncol) {
                stream << ' ' << *iter;
                cur_col += n + 1;
            } else {
                // line break
                if (cnt_str == nullptr) {
                    // no line break character
                    stream << '\n';
                    for (int cnt = 0; cnt < tab_size; ++cnt) {
                        stream << ' ';
                    }
                    stream << *iter;
                    cur_col = n + tab_size;
                } else if (break_before) {
                    stream << ' ' << cnt_str << '\n';
                    for (int cnt = 0; cnt < tab_size; ++cnt) {
                        stream << ' ';
                    }
                    stream << *iter;
                    cur_col = n + tab_size;
                } else {
                    stream << '\n' << cnt_str << ' ' << *iter;
                    cur_col = n + 2;
                }
            }
        }
        if (newline)
            stream << '\n';
    }
};

nstream_output &lstream::to_file(nstream_output &stream, spirit::namespace_cdba,
                                 bool newline) const {
    helper::to_file_helper(*this, stream, newline, 0, false, 80, "+");
    return stream;
}
nstream_output &lstream::to_file(nstream_output &stream, spirit::namespace_cdl_cmd,
                                 bool newline) const {
    helper::to_file_helper(*this, stream, newline, 0, false, 80, "*+");
    return stream;
}
nstream_output &lstream::to_file(nstream_output &stream, spirit::namespace_verilog,
                                 bool newline) const {
    helper::to_file_helper(*this, stream, newline, 4, true, 120, nullptr);
    return stream;
}
nstream_output &lstream::to_file(nstream_output &stream, spirit::namespace_spectre,
                                 bool newline) const {
    helper::to_file_helper(*this, stream, newline, 0, false, 80, "+");
    return stream;
}

} // namespace netlist
} // namespace cbag
