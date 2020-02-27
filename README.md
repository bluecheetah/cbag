# CBAG (C++ BAG AMS Generator)

A C++ VLSI circuit schematic and layout database library written primarily to work
with [BAG](https://github.com/bluecheetah/bag).

This is a fork of [CBAG] (https://github.com/ucb-art/cbag), which as of May 13th, 2019,
has been taken offline temporarily.

## Licensing

This library is licensed under the Apache-2.0 license.  However, some source files are licensed
under both Apache-2.0 and BSD-3-Clause license, meaning that the user must comply with the
terms and conditions of both licenses.  See [here](LICENSE.BSD-3-Clause) for full text of the
BSD license, and [here](LICENSE.Apache-2.0) for full text of the Apache license.  See individual
files to check if they fall under Apache-2.0, or both Apache-2.0 and BSD-3-Clause.

## Dependencies

* spdlog
  `git clone git@github.com:gabime/spdlog.git`

*  fmt
  `git clone git@github.com:fmtlib/fmt.git`

* yaml-cpp
  `git clone git@github.com:jbeder/yaml-cpp.git`

* catch2
  `git clone git@github.com:catchorg/Catch2.git`

*  libboost-serialization
  (on Debian/Ubuntu) `sudo apt install libboost-serialization-dev`

## Restrictions

### Machine Architecture

1. This extension will only work on architectures that performs arithmetic right shift.  In other
   words, right shift on negative numbers will fill in 1s on the most significant bit.

### Schematic

1. All schematic pins must be either a scalar net or a bus net, and if it is a bus net,
   the step size must be exactly 1.  For example, `foo`, `bar<3:0>`, and `baz<0:2>` are valid
   pin names, but `a,b`, `c<3:0:2>`, `<*3>d` are illegal.

2. If a schematic contains a bus pin, then there can be no other net inside the schematic with the
   same base name.  For example, if a cellview has a pin called `foo<3:1>`, you cannot have an
   internal net named `foo<4>`, or `foo<0>`.
