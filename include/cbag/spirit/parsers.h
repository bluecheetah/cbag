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

#ifndef CBAG_SPIRIT_PARSERS_H
#define CBAG_SPIRIT_PARSERS_H

#include <boost/spirit/home/x3.hpp>

#include <cbag/spirit/config.h>

namespace x3 = boost::spirit::x3;

namespace cbag {

template <typename A, typename R> void parse(const std::string &source, R const &rule, A &ast) {
    if (source.empty())
        throw std::invalid_argument("Cannot parse empty string.");

    std::stringstream out;

    auto iter_start = source.begin();
    auto const iter_end = source.end();

    // Our error handler
    spirit::parser::error_handler_type error_handler(iter_start, iter_end, out);
    const std::reference_wrapper<spirit::parser::error_handler_type> err_ref =
        std::ref(error_handler);
    // Our spirit
    auto const parser =
        // we pass our error handler to the spirit so we can access
        // it later on in our on_error and on_sucess handlers
        x3::with<spirit::parser::error_handler_tag>(err_ref)[rule];

    // Go forth and parse!
    bool success = x3::parse(iter_start, iter_end, parser, ast);
    bool not_finish = (iter_start != iter_end);
    if (!success || not_finish) {
        if (iter_start != iter_end) {
            std::string marker(source.size(), '-');
            marker[iter_start - source.begin()] = '^';
            out << "Cannot parse following string completely:\n";
            out << source << '\n'
                << marker << '\n'
                << "Only up to index " << (iter_start - source.begin()) << '\n';
        }
        throw std::invalid_argument(out.str());
    }
}

} // namespace cbag

#endif // CBAG_SPIRIT_PARSERS_H
