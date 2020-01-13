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

#ifndef CBAG_SPIRIT_NAME_DEF_H
#define CBAG_SPIRIT_NAME_DEF_H

#include <boost/spirit/home/x3.hpp>
#include <boost/spirit/home/x3/support/utility/annotate_on_success.hpp>

#include <cbag/spirit/ast.h>
#include <cbag/spirit/ast_adapted.h>
#include <cbag/spirit/error_handler.h>
#include <cbag/spirit/name.h>
#include <cbag/spirit/name_rep.h>
#include <cbag/spirit/name_unit_def.h>

namespace x3 = boost::spirit::x3;

namespace cbag {
namespace spirit {
namespace parser {

name_rep_type const name_rep = "name_rep";

name_type const name = "name";

/** Grammar for multiplier tag
 *
 *  The multiplier tag has the form:
 *  <*a>
 *
 *  a must be positive.
 *  expectation operator is used because once the tag prefix is matched,
 *  the context is uniquely determined.
 */
auto const mult_tag = "<*" > x3::uint32[check_zero] > '>';

/** Grammar for a group of names
 *
 *  A group of names is a name_unit or a name surrounded by parentheses.
 *  expectation operator is used to get good error messages.  sequence
 *  operator is used after name_unit to allow for back-tracking, as
 *  we cannot distinguish between name and name_unit until we hit
 *  a comma or the close parenthesis.
 */
auto const grp_name = '(' > ((name_unit >> ')') | (name > ')'));

/** Grammar for a repeated name
 *
 *  name_rep can be one of:
 *  foo
 *  <*a>foo
 *  <*a>(foo)
 *  <*a>(foo,bar)
 *
 *  sequence operator is used in the first OR block to allow for
 *  back-tracking, as we cannot distinguish between a group of names
 *  or a name unit until we see the character right after the multiplier
 *  tag.
 */
auto const name_rep_def = name_rep_type{} = (mult_tag >> grp_name) | (-mult_tag > name_unit);

/** Grammar for name.
 *
 *  a name is simply a comma-separated list of name_reps.
 */
auto const name_def = name_rep % ',';

BOOST_SPIRIT_DEFINE(name_rep);
BOOST_SPIRIT_DEFINE(name);

struct name_rep_class : x3::annotate_on_success, error_handler_base {};
struct name_class : x3::annotate_on_success, error_handler_base {};
} // namespace parser
} // namespace spirit
} // namespace cbag
#endif // CBAG_SPIRIT_NAME_DEF_H
