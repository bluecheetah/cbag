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

#ifndef CBAG_COMMON_LP_LOOKUP_H
#define CBAG_COMMON_LP_LOOKUP_H

#include <unordered_map>

#include <cbag/common/typedefs.h>

namespace cbag {
namespace layout {

using lay_map_t = std::unordered_map<std::string, lay_t>;
using purp_map_t = std::unordered_map<std::string, purp_t>;

class lp_lookup {
  private:
    lay_map_t lay_map_;
    purp_map_t purp_map_;
    purp_t default_purpose_;
    purp_t pin_purpose_;
    purp_t label_purpose_;

  public:
    // default constructor needed to make tech default constructable
    lp_lookup();

    lp_lookup(lay_map_t &&lay_map, purp_map_t &&purp_map, const std::string &def_purp,
              const std::string &pin_purp, const std::string &label_purp);

    purp_t get_default_purpose() const;

    purp_t get_pin_purpose() const;

    purp_t get_label_purpose() const;

    const std::string &get_layer_name(lay_t lay_id) const;

    const std::string &get_purpose_name(purp_t purp_id) const;

    std::optional<lay_t> get_layer_id(const std::string &layer) const;

    std::optional<purp_t> get_purpose_id(const std::string &purpose) const;
};

} // namespace layout
} // namespace cbag

#endif
