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

#ifndef CBAG_LAYOUT_CELLVIEW_POLY_H
#define CBAG_LAYOUT_CELLVIEW_POLY_H

#include <cbag/layout/blockage.h>
#include <cbag/layout/boundary.h>
#include <cbag/layout/cellview.h>
#include <cbag/layout/cv_obj_ref.h>
#include <cbag/layout/polygons.h>
#include <cbag/layout/pt_list.h>
#include <cbag/layout/tech_util.h>

namespace cbag {
namespace layout {

class track_id;

shape_ref<box_t> add_rect(const std::shared_ptr<cellview> &cv_ptr, const std::string &layer,
                          const std::string &purpose, box_t bbox, bool commit);

template <typename T>
shape_ref<T> add_polygon(const std::shared_ptr<cellview> &cv_ptr, const std::string &layer,
                         const std::string &purpose, T &&poly, bool commit) {
    auto key = layer_t_at(*(cv_ptr->get_tech()), layer, purpose);
    return {cv_ptr, std::move(key), std::forward<T>(poly), commit};
}

template <typename T, typename = IsPtList<T>>
shape_ref<poly_90_t> add_poly90(const std::shared_ptr<cellview> &cv_ptr, const std::string &layer,
                                const std::string &purpose, const T &data, bool commit) {
    poly_90_t poly;
    poly.set_points(traits::pt_list<T>::begin(data), traits::pt_list<T>::end(data));
    return add_polygon(cv_ptr, layer, purpose, std::move(poly), commit);
}

template <typename T, typename = IsPtList<T>>
shape_ref<poly_45_t> add_poly45(const std::shared_ptr<cellview> &cv_ptr, const std::string &layer,
                                const std::string &purpose, const T &data, bool commit) {
    poly_45_t poly;
    poly.set(traits::pt_list<T>::begin(data), traits::pt_list<T>::end(data));
    return add_polygon(cv_ptr, layer, purpose, std::move(poly), commit);
}

template <typename T, typename = IsPtList<T>>
shape_ref<poly_t> add_poly(const std::shared_ptr<cellview> &cv_ptr, const std::string &layer,
                           const std::string &purpose, const T &data, bool commit) {
    poly_t poly;
    poly.set(traits::pt_list<T>::begin(data), traits::pt_list<T>::end(data));
    return add_polygon(cv_ptr, layer, purpose, std::move(poly), commit);
}

template <typename T, typename = IsPtList<T>>
cv_obj_ref<blockage> add_blockage(const std::shared_ptr<cellview> &cv_ptr, const std::string &layer,
                                  blockage_type blk_type, const T &data, bool commit) {
    auto lay_id_opt = cv_ptr->get_tech()->get_layer_id(layer);
    auto lay_id = (lay_id_opt) ? *lay_id_opt : 0;
    blockage obj(blk_type, lay_id);
    obj.set(traits::pt_list<T>::begin(data), traits::pt_list<T>::end(data));
    return {cv_ptr, std::move(obj), commit};
}

template <typename T, typename = IsPtList<T>>
cv_obj_ref<boundary> add_boundary(const std::shared_ptr<cellview> &cv_ptr, boundary_type bnd_type,
                                  const T &data, bool commit) {
    boundary obj(bnd_type);
    obj.set(traits::pt_list<T>::begin(data), traits::pt_list<T>::end(data));
    return {cv_ptr, std::move(obj), commit};
}

} // namespace layout
} // namespace cbag

#endif
