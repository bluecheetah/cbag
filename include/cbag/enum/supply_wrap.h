// SPDX-License-Identifier: Apache-2.0
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

#ifndef CBAG_ENUM_SUPPLY_WRAP_H
#define CBAG_ENUM_SUPPLY_WRAP_H

#include <cbag/common/typedefs.h>

namespace cbag {

enum class supply_wrap : enum_t {
    NONE = 0,
    TOP = 1,
    ALL = 2,
};

} // namespace cbag

#endif
