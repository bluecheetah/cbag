// SPDX-License-Identifier: BSD-3-Clause AND Apache-2.0
/*
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

#ifndef CBAG_LAYOUT_SERIALIZE_H
#define CBAG_LAYOUT_SERIALIZE_H

#include <string>

#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>

#include <cbag/common/box_t.h>

namespace boost {
namespace serialization {

template <class Archive>
void serialize(Archive &ar, cbag::polygon::interval_data<cbag::coord_t> &obj,
               const unsigned int version) {
    ar &(obj[false]);
    ar &(obj[true]);
}

template <class Archive> void serialize(Archive &ar, cbag::box_t &obj, const unsigned int version) {
    ar &(obj[false]);
    ar &(obj[true]);
}

} // namespace serialization
} // namespace boost

#endif
