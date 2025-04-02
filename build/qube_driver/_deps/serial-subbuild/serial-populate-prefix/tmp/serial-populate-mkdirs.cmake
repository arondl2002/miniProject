# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "/home/user/miniProject/build/qube_driver/_deps/serial-src"
  "/home/user/miniProject/build/qube_driver/_deps/serial-build"
  "/home/user/miniProject/build/qube_driver/_deps/serial-subbuild/serial-populate-prefix"
  "/home/user/miniProject/build/qube_driver/_deps/serial-subbuild/serial-populate-prefix/tmp"
  "/home/user/miniProject/build/qube_driver/_deps/serial-subbuild/serial-populate-prefix/src/serial-populate-stamp"
  "/home/user/miniProject/build/qube_driver/_deps/serial-subbuild/serial-populate-prefix/src"
  "/home/user/miniProject/build/qube_driver/_deps/serial-subbuild/serial-populate-prefix/src/serial-populate-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/home/user/miniProject/build/qube_driver/_deps/serial-subbuild/serial-populate-prefix/src/serial-populate-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "/home/user/miniProject/build/qube_driver/_deps/serial-subbuild/serial-populate-prefix/src/serial-populate-stamp${cfgdir}") # cfgdir has leading slash
endif()
