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

#ifndef CBAG_SPIRIT_RANGE_DEF_H
#define CBAG_SPIRIT_RANGE_DEF_H

#include <boost/spirit/home/x3.hpp>
#include <boost/spirit/home/x3/support/utility/annotate_on_success.hpp>

#include <cbag/spirit/ast.h>
#include <cbag/spirit/ast_adapted.h>
#include <cbag/spirit/error_handler.h>
#include <cbag/spirit/range.h>

namespace x3 = boost::spirit::x3;

namespace cbag {
namespace spirit {
namespace parser {

// after parsing start, set default values of stop and step
auto init_range = [](auto &ctx) {
    x3::_val(ctx).stop = x3::_attr(ctx);
    x3::_val(ctx).step = 1;
};

// make sure value is not 0
auto check_zero = [](auto &ctx) { x3::_pass(ctx) = (x3::_attr(ctx) != 0); };

range_type const range = "range";

/** Grammer for range
 *
 *  range can be one of:
 *  <a>
 *  <a:b>
 *  <a:b:c>
 *
 *  if b is missing, then b = a by default.  If c is missing, then c = 1.
 *  c cannot be 0, all of a, b, and c must be non-negative.
 *  expectation operators are used in places where context is uniquely determined
 *  so we get good error messages.
 */
auto const range_def = range_type{} = '<' > (x3::uint32[init_range]) >>
                                      -(':' > x3::uint32 >> -(':' > x3::uint32[check_zero])) > '>';

BOOST_SPIRIT_DEFINE(range);

struct range_class : x3::annotate_on_success, error_handler_base {};
} // namespace parser
} // namespace spirit
} // namespace cbag
#endif // CBAG_SPIRIT_RANGE_DEF_H
