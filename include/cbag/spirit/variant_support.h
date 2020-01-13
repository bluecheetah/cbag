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

#ifndef CBAG_SPIRIT_VARIANT_SUPPORT_H
#define CBAG_SPIRIT_VARIANT_SUPPORT_H

// hack from https://github.com/boostorg/spirit/issues/270 to get
// X3 to work with std::variant
#include <variant>

#include <boost/mpl/vector.hpp>
#include <boost/spirit/home/x3/support/traits/is_variant.hpp>
#include <boost/spirit/home/x3/support/traits/tuple_traits.hpp>
#include <boost/spirit/home/x3/support/traits/variant_find_substitute.hpp>
#include <boost/spirit/home/x3/support/traits/variant_has_substitute.hpp>

// Based on: boost/spirit/home/x3/support/traits/variant_find_substitute.hpp
namespace boost::spirit::x3::traits {
template <typename... Ts> struct is_variant<std::variant<Ts...>> : mpl::true_ {};

template <typename... Ts, typename Attribute>
struct variant_find_substitute<std::variant<Ts...>, Attribute> {
    typedef std::variant<Ts...> variant_type;
    typedef mpl::vector<Ts...> types;
    typedef typename mpl::end<types>::type end;

    typedef typename mpl::find_if<types, is_same<mpl::_1, Attribute>>::type iter_1;

    typedef typename mpl::eval_if<is_same<iter_1, end>,
                                  mpl::find_if<types, traits::is_substitute<mpl::_1, Attribute>>,
                                  mpl::identity<iter_1>>::type iter;

    typedef
        typename mpl::eval_if<is_same<iter, end>, mpl::identity<Attribute>, mpl::deref<iter>>::type
            type;
};

template <typename... Ts, typename Attribute>
struct variant_has_substitute_impl<std::variant<Ts...>, Attribute> {
    // Find a type from the variant that can be a substitute for Attribute.
    // return true_ if one is found, else false_

    typedef mpl::vector<Ts...> types;

    typedef typename mpl::end<types>::type end;

    typedef typename mpl::find_if<types, is_same<mpl::_1, Attribute>>::type iter_1;

    typedef typename mpl::eval_if<is_same<iter_1, end>,
                                  mpl::find_if<types, traits::is_substitute<mpl::_1, Attribute>>,
                                  mpl::identity<iter_1>>::type iter;

    typedef mpl::not_<is_same<iter, end>> type;
};
} // namespace boost::spirit::x3::traits

#endif
