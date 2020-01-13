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

#ifndef CBAG_YAML_SHAPES_H
#define CBAG_YAML_SHAPES_H

#include <boost/fusion/include/adapt_struct.hpp>

#include <yaml-cpp/yaml.h>

#include <cbag/schematic/shape_t_def.h>

#include <cbag/yaml/fusion.h>
#include <cbag/yaml/int_array.h>
#include <cbag/yaml/point_data.h>

BOOST_FUSION_ADAPT_STRUCT(cbag::sch::rectangle, layer, purpose, net, bbox)

BOOST_FUSION_ADAPT_STRUCT(cbag::sch::polygon, layer, purpose, net, points)

BOOST_FUSION_ADAPT_STRUCT(cbag::sch::arc, layer, purpose, net, ang_start, ang_stop, bbox)

BOOST_FUSION_ADAPT_STRUCT(cbag::sch::donut, layer, purpose, net, center, radius, hole_radius)

BOOST_FUSION_ADAPT_STRUCT(cbag::sch::ellipse, layer, purpose, net, bbox)

BOOST_FUSION_ADAPT_STRUCT(cbag::sch::line, layer, purpose, net, points)

BOOST_FUSION_ADAPT_STRUCT(cbag::sch::path, layer, purpose, net, width, points, style, begin_ext,
                          end_ext)

BOOST_FUSION_ADAPT_STRUCT(cbag::sch::text_t, layer, purpose, net, origin, alignment, orient, font,
                          height, overbar, visible, drafting, text)

BOOST_FUSION_ADAPT_STRUCT(cbag::sch::eval_text, layer, purpose, net, origin, alignment, orient,
                          font, height, overbar, visible, drafting, text, evaluator)

BOOST_FUSION_ADAPT_STRUCT(cbag::sch::term_attr, layer, purpose, net, origin, alignment, orient,
                          font, height, overbar, visible, drafting, attr_type, format)

namespace YAML {

template <> struct convert<cbag::sch::eval_text> {
    using value_type = cbag::sch::eval_text;

    static Node encode(const value_type &rhs);

    static bool decode(const Node &node, value_type &rhs);
};

template <> struct convert<cbag::sch::shape_t> {
    using value_type = cbag::sch::shape_t;

    static Node encode(const value_type &rhs);

    static bool decode(const Node &node, value_type &rhs);
};

} // namespace YAML

#endif // CBAG_YAML_SHAPES_H
