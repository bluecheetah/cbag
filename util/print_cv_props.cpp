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

#include <iostream>

#include <cbag/oa/database.h>
#include <cbag/oa/read.h>
#include <cbag/oa/util.h>

void print_cv_props_top(const char *lib_name, const char *cell_name,
                        const char *view_name = "symbol", const char *lib_file = "cds.lib") {

    cbagoa::database db(lib_file);

    try {
        cbagoa::print_cv_props(db.ns_native, db.ns, *(db.logger), lib_name, cell_name, view_name);
    } catch (...) {
        cbagoa::handle_oa_exceptions(*(db.logger));
    }
}

int main(int argc, char *argv[]) {
    switch (argc) {
    case 3:
        print_cv_props_top(argv[1], argv[2]);
        break;
    case 4:
        print_cv_props_top(argv[1], argv[2], argv[3]);
        break;
    case 5:
        print_cv_props_top(argv[1], argv[2], argv[3], argv[4]);
        break;
    default:
        std::cout << "Usage: print_cv_props_top <lib_name> <cell_name> [<view_name> "
                     "[<cds_lib_fname>]]"
                  << std::endl;
    }

    return 0;
}
