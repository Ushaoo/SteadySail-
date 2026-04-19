# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "/Users/adam/esp/esp-idf-v4.4.3/components/bootloader/subproject"
  "/Users/adam/Desktop/这是一个总文件夹/上学/HKUST/FYP/ESP/my_project/build/bootloader"
  "/Users/adam/Desktop/这是一个总文件夹/上学/HKUST/FYP/ESP/my_project/build/bootloader-prefix"
  "/Users/adam/Desktop/这是一个总文件夹/上学/HKUST/FYP/ESP/my_project/build/bootloader-prefix/tmp"
  "/Users/adam/Desktop/这是一个总文件夹/上学/HKUST/FYP/ESP/my_project/build/bootloader-prefix/src/bootloader-stamp"
  "/Users/adam/Desktop/这是一个总文件夹/上学/HKUST/FYP/ESP/my_project/build/bootloader-prefix/src"
  "/Users/adam/Desktop/这是一个总文件夹/上学/HKUST/FYP/ESP/my_project/build/bootloader-prefix/src/bootloader-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/Users/adam/Desktop/这是一个总文件夹/上学/HKUST/FYP/ESP/my_project/build/bootloader-prefix/src/bootloader-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "/Users/adam/Desktop/这是一个总文件夹/上学/HKUST/FYP/ESP/my_project/build/bootloader-prefix/src/bootloader-stamp${cfgdir}") # cfgdir has leading slash
endif()
