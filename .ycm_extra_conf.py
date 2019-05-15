#!/bin/env python
# SPDX-License-Identifier: BSD-3-Clause AND Apache-2.0
# Copyright 2018 Regents of the University of California
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# * Redistributions of source code must retain the above copyright notice, this
#   list of conditions and the following disclaimer.
#
# * Redistributions in binary form must reproduce the above copyright notice,
#   this list of conditions and the following disclaimer in the documentation
#   and/or other materials provided with the distribution.
#
# * Neither the name of the copyright holder nor the names of its
#   contributors may be used to endorse or promote products derived from
#   this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

# Copyright 2019 Blue Cheetah Analog Design Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""The ycmd configuration script.

This file is used by ycmd to find compilation flags for a file.  It is based on Nils Deppe's file in
2017 and modified on top of that.
"""

import os

import ycm_core

default_flags = ['-x', 'c++', '-Wall', '-Wextra', '-Werror', '-std=c++17']

cpp_source_extensions = {'.cpp', '.cxx', '.cc', '.c', '.m', '.mm'}

header_file_extensions = {'.h', '.H', '.hxx', '.hpp', '.hh'}

completion_database = {}


def FlagsForFile(filename, **_):
    """Returns the flags needed to compile the file.

    Algorithm is as follows:

    1. If no compilation database contains the given file, use default flags.

    2. If flags are found in the compilation database, return them.  Otherwise, reload the database
    and try again.

    3. If flags are still not found, this is probably a source file not added to the build system
    yet, in that case return None so ycmd will try again later.

    Note 1: This function is only called by ycmd if no flags have been loaded for the file. Once
    flags are loaded it is not called again. Or at least that appears to be the behavior.
    
    Note 2: If precompiled headers are not working then the libclang used by ycmd is different than
    the system libclang. The solution seems to be setting ycmd to use the system libclang.

    """
    # print_log('getting flags for file: {}'.format(filename))
    fdir, fname = os.path.split(filename)
    # check existing database in cache
    ddir, db = next((ddb for ddb in completion_database.items() if ddb[0] in fdir),
                    (None, None))

    if ddir is None:
        # database not in cache, try to find it.
        # print_log('Try to find database')
        ddir, db = FindCompilationDatabaseAndDir(fdir, fname)
        if ddir is None:
            # no database found, just return default flags.
            # print_log('database not found.  Return defaults')
            return GetDefaultFlags(fdir)
        # found database, update cache
        # print_log('Adding database to cache')
        completion_database[ddir] = db
    # print_log('Database at: {}'.format(ddir))  

    # find compilation info for this file
    # print_log('Try to get compilation flag from database')
    compilation_info = GetCompilationInfoForFile(fdir, fname, ddir, db, False)

    if compilation_info is None or len(compilation_info.compiler_flags_) == 0:
        # cannot find compilation info in database, try updating cache
        # print_log('Flags not found, refreshing database')
        completion_database[ddir] = db = ycm_core.CompilationDatabase(ddir)
        # try again after updating cache
        # print_log('Try to get compilation flags again')
        compilation_info = GetCompilationInfoForFile(fdir, fname, ddir, db, True)

    if compilation_info is None or len(compilation_info.compiler_flags_) == 0:
        # print_log('Still no flags, return None')
        # We cannot find compilation info even after refresh.  Since we return default flags for
        # headers, this must be a source file that is not added to build system yet.  Return None to
        # tell YCMD that it should try again later.  try loading the flags again.
        return None

    # return compilation flags
    # print_log('Flags found: {}'.format(compilation_info.compiler_flags_))
    return {'flags': list(compilation_info.compiler_flags_),
            'include_paths_relative_to_dir':
            compilation_info.compiler_working_dir_}


def FindCompilationDatabaseAndDir(fdir, fname):
    """Find compilation database for the given directory.

    Returns the directory the ycm compilation database was found in, and the database. Returns None
    if no compilation database was found.  This assumes that any compilation database found in the
    hierarchy is the one we want. That is, it assumes projects are not nested.
    """
    while fname:
        # print_log('Checking {} for database'.format(fdir))
        if os.path.isfile(os.path.join(fdir, "compile_commands.json")):
            # print_log('Found database at {}'.format(fdir))
            return fdir, ycm_core.CompilationDatabase(fdir)
        fdir, fname = os.path.split(fdir)
    return None, None


def GetDefaultFlags(fdir):
    """Returns the default compilation flags.
    """
    return {'flags': list(default_flags),
            'include_paths_relative_to_dir': fdir}


def GetCompilationInfoForFile(fdir, fname, ddir, db, use_default_for_header=False):
    """The compilation information from the database.
    
    The compilation_commands.json file generated by CMake does not have entries for header files. So
    we do our best by asking the db for flags for a corresponding source file, if any. If one
    exists, the flags for that file should be good enough.
    """
    basename, ext = os.path.splitext(fname)
    # print_log('Find comp info for basename: {}, extension: {}'.format(basename, ext))
    if ext in header_file_extensions:
        comp_info = FindHeaderCompilationInfo(fdir, basename, ddir, db)
        if comp_info is None:
            # source file not found
            return GetDefaultFlags(fdir) if use_default_for_header else None
        else:
            return comp_info
    else:
        # print_log('Is not header, file is: {}'.format(os.path.join(fdir, fname)))
        return db.GetCompilationInfoForFile(os.path.join(fdir, fname))
    

def FindHeaderCompilationInfo(fdir, basename, ddir, db):
    """Try to find compilation info for a header file.

    Given the basename of a header file, find the compilation info using the following algorithm:

    1. Let HDIR be the header file directory, compute SDIR, which is HDIR with the last occurrence
       of "include" replaced by "src".

    2. Find a source file with the same basename in SDIR.  If it exists, return its compilation
       flags if it has them, otherwise return None.

    3. Otherwise, starting from SDIR, find any source file with compilation flags and return it.
       The search starts from SDIR and ends at the database directory, where we recursively go up,
       and also search for subdirectories we haven't search before.

    4. If all else fails, return None.
    """
    # print_log('Finding compilation flag for header {} {}'.format(fdir, basename))
    # get source directory
    parts = fdir.rsplit('include', 1)
    sdir = 'src'.join(parts)
    # print_log('SDIR = {}'.format(sdir))

    # search for source file with matching name
    for sext in cpp_source_extensions:
        cur_fname = os.path.join(sdir, basename + sext)
        if os.path.isfile(cur_fname):
            comp_info = db.GetCompilationInfoForFile(cur_fname)
            # print_log('found source file {}'.format(cur_fname))
            return comp_info if comp_info.compiler_flags_ else None

    # search for any source file
    # print_log('recursive search')
    return RecursiveCompilationInfoSearch(sdir, ddir, '', db)


def RecursiveCompilationInfoSearch(cur_dir, stop_dir, exclude_dir, db):
    """Search for any compilation info recursively.

    Find any source file with compilation info in this directory and all subdirectories, excluding
    the given directory name.  If None found, repeat this process in the parent directory, but
    exclude this directory from search (to avoid duplicate search).  This process ends when we reach
    stop_dir.
    """
    # search in this directory and all subdirectories
    test = RecursiveCompilationInfoSearchDown(cur_dir, exclude_dir, db)
    if test is not None:
        return test

    # stop condition
    if cur_dir == stop_dir:
        return None

    # move up
    parent_dir, cur_dir_name = os.path.split(cur_dir)
    return RecursiveCompilationInfoSearch(parent_dir, stop_dir, cur_dir_name, db)


def RecursiveCompilationInfoSearchDown(cur_dir, exclude_dir, db):
    """Search for any compilation info recursively downwards.

    Find any source file with compilation info in current directory.  Recurse in all subdirectories
    if none found.  Skip exclude_dir if it is a subdirectory of the current directory.
    """
    if not os.path.isdir(cur_dir):
        return None

    _, sub_dirs, files = next(os.walk(cur_dir))

    # check files in current directory
    for fname in files:
        fbase, fext = os.path.splitext(fname)
        if fext in cpp_source_extensions:
            comp_info = db.GetCompilationInfoForFile(os.path.join(cur_dir, fname))
            if comp_info.compiler_flags_:
                # print_log("Getting flags for a header file, "
                #           "returning compilation flags for file: ".format(filename))
                return comp_info

    # recurse in all subdirectories
    for next_dir in sub_dirs:
        if next_dir != exclude_dir:
            test = RecursiveCompilationInfoSearchDown(os.path.join(cur_dir, next_dir), '', db)
            if test is not None:
                return test

    return None


def print_log(msg):
    with open('/tmp/ycm-extra-conf.log', 'a') as f:
        f.write(msg + '\n')
