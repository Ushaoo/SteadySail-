# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "C:/Espressif/frameworks/esp-idf-v4.4.8/components/bootloader/subproject"
  "D:/SteadySail--1/version_2/build/bootloader"
  "D:/SteadySail--1/version_2/build/bootloader-prefix"
  "D:/SteadySail--1/version_2/build/bootloader-prefix/tmp"
  "D:/SteadySail--1/version_2/build/bootloader-prefix/src/bootloader-stamp"
  "D:/SteadySail--1/version_2/build/bootloader-prefix/src"
  "D:/SteadySail--1/version_2/build/bootloader-prefix/src/bootloader-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "D:/SteadySail--1/version_2/build/bootloader-prefix/src/bootloader-stamp/${subDir}")
endforeach()
