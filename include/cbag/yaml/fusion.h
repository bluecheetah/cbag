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

/*
NOTE:
The code is entirely copied from:
http://jrruethe.github.io/blog/2015/08/17/yaml-de-slash-serialization-with-boost-fusion
*/

#ifndef CBAG_YAML_FUSION_H
#define CBAG_YAML_FUSION_H

#include <string>

#include <boost/mpl/range_c.hpp>
#include <boost/mpl/size.hpp>

// boost::fusion
#include <boost/fusion/include/adapted.hpp>

// boost::fusion::for_each
#include <boost/fusion/include/for_each.hpp>

// boost::fusion::zip
#include <boost/fusion/include/zip.hpp>

// boost::units::detail::demangle
#include <boost/units/detail/utility.hpp>

#include <yaml-cpp/yaml.h>

namespace cbagyaml {

template <typename S> struct sequence {
    // Point to the first element
    typedef boost::mpl::int_<0> begin;

    // Point to the element after the last element in the sequence
    typedef typename boost::fusion::result_of::size<S>::type end;

    // Point to the first element
    typedef boost::mpl::int_<0> first;

    // Point to the second element (for pairs)
    typedef boost::mpl::int_<1> second;

    // Point to the last element in the sequence
    typedef typename boost::mpl::prior<end>::type last;

    // Number of elements in the sequence
    typedef typename boost::fusion::result_of::size<S>::type size;

    // Get a range representing the size of the structure
    typedef boost::mpl::range_c<unsigned int, 0, boost::mpl::size<S>::value> indices;
};

template <typename S, typename N> struct element_at {
    // Type of the element at this index
    typedef typename boost::fusion::result_of::value_at<S, N>::type type;

    // Previous element
    typedef typename boost::mpl::prior<N>::type previous;

    // Next element
    typedef typename boost::mpl::next<N>::type next;

    // Member name of the element at this index
    static inline std::string name() {
        return boost::fusion::extension::struct_member_name<S, N::value>::call();
    }

    // Type name of the element at this index
    static inline std::string type_name() {
        return boost::units::detail::demangle(typeid(type).name());
    }

    // Access the element
    static inline typename boost::fusion::result_of::at<S const, N>::type get(S const &s) {
        return boost::fusion::at<N>(s);
    }
};

template <typename T> struct type {
    // Return the string name of the type
    static inline std::string name() { return boost::units::detail::demangle(typeid(T).name()); }
};

template <typename T> struct inserter {
    typedef T Type;

    inserter(YAML::Node &subroot) : mSubroot(subroot) {}

    template <typename Zip> void operator()(Zip const &zip) const {
        typedef typename boost::remove_const<typename boost::remove_reference<
            typename boost::fusion::result_of::at_c<Zip, 0>::type>::type>::type Index;

        // Get the field name as a string using reflection
        std::string field_name = element_at<Type, Index>::name();

        // Get the field type
        typedef BOOST_TYPEOF(boost::fusion::at_c<1>(zip)) FieldType;

        // Alias the member
        FieldType const &member = boost::fusion::at_c<1>(zip);

        // Store this field in the yaml node
        mSubroot[field_name] = member;
    }

  protected:
    YAML::Node &mSubroot;
};

template <typename T> struct extractor {
    typedef T Type;

    extractor(YAML::Node &subroot) : mSubroot(subroot), mItem(0) {}

    template <typename Zip> void operator()(Zip const &zip) const {
        typedef typename boost::remove_const<typename boost::remove_reference<
            typename boost::fusion::result_of::at_c<Zip, 0>::type>::type>::type Index;

        // Get the field name as a string using reflection
        std::string field_name = element_at<Type, Index>::name();

        // Get the field native type
        typedef BOOST_TYPEOF(boost::fusion::at_c<1>(zip)) FieldType;

        // Alias the member
        // We need to const cast this because "boost::fusion::for_each"
        // requires that zip be const, however we want to modify it.
        FieldType const &const_member = boost::fusion::at_c<1>(zip);
        FieldType &member = const_cast<FieldType &>(const_member);

        // We need to const cast this because "boost::fusion::for_each"
        // requires that operator() be const, however we want to modify
        // the object. This item number is used for error reporting.
        int const &const_item = mItem;
        int &item = const_cast<int &>(const_item);

        // Try to load the value from the file
        try {
            // Extract this field from the yaml node
            member = mSubroot[field_name].template as<FieldType>();

            // This item number helps us find issues when loading incomplete
            // yaml files
            ++item;
        }
        // Catch any exceptions
        catch (YAML::Exception const &e) {
            std::string type_name = type<FieldType>::name();

            throw std::invalid_argument("Error loading item.");
        }
    }

  protected:
    YAML::Node &mSubroot;
    int mItem;
};

} // namespace cbagyaml

namespace YAML {
template <typename T> struct convert {
    // This function will only be available if the template parameter is a boost
    // fusion sequence
    static Node
    encode(T const &rhs,
           typename boost::enable_if<typename boost::fusion::traits::is_sequence<T>::type>::type * =
               0) {
        // For each item in T
        // Call inserter recursively
        // Every sequence is made up of primitives at some level

        // Get a range representing the size of the structure
        typedef typename cbagyaml::sequence<T>::indices indices;

        // Make a root node to insert into
        YAML::Node root;

        // Create an inserter for the root node
        cbagyaml::inserter<T> inserter(root);

        // Insert each member of the structure
        boost::fusion::for_each(boost::fusion::zip(indices(), rhs), inserter);

        return root;
    }

    // This function will only be available if the template parameter is a boost
    // fusion sequence
    static bool
    decode(Node const &node, T &rhs,
           typename boost::enable_if<typename boost::fusion::traits::is_sequence<T>::type>::type * =
               0) {
        // For each item in T
        // Call extractor recursively
        // Every sequence is made up of primitives at some level

        // Get a range representing the size of the structure
        typedef typename cbagyaml::sequence<T>::indices indices;

        // Create an extractor for the root node
        // Yaml-cpp requires node to be const&, but the extractor makes
        // non-const calls to it.
        Node &writable_node = const_cast<Node &>(node);
        cbagyaml::extractor<T> extractor(writable_node);

        // Extract each member of the structure
        try {
            // An exception is thrown if any item in the loop cannot be read
            boost::fusion::for_each(boost::fusion::zip(indices(), rhs), extractor);
        }
        // Catch all exceptions and prevent them from propagating
        catch (...) {
            return false;
        }
        // If we made it here, all fields were read correctly
        return true;
    }
};
} // namespace YAML
#endif // CBAG_YAML_FUSION_H
