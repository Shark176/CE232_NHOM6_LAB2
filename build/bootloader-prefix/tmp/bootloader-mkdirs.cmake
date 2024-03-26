# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "E:/DATA/Espressif/frameworks/esp-idf-v5.2/components/bootloader/subproject"
  "D:/Phan_Thanh_Tuan/UIT-Online/HK6/TH_TKHTNKD/LAB2_OLED/build/bootloader"
  "D:/Phan_Thanh_Tuan/UIT-Online/HK6/TH_TKHTNKD/LAB2_OLED/build/bootloader-prefix"
  "D:/Phan_Thanh_Tuan/UIT-Online/HK6/TH_TKHTNKD/LAB2_OLED/build/bootloader-prefix/tmp"
  "D:/Phan_Thanh_Tuan/UIT-Online/HK6/TH_TKHTNKD/LAB2_OLED/build/bootloader-prefix/src/bootloader-stamp"
  "D:/Phan_Thanh_Tuan/UIT-Online/HK6/TH_TKHTNKD/LAB2_OLED/build/bootloader-prefix/src"
  "D:/Phan_Thanh_Tuan/UIT-Online/HK6/TH_TKHTNKD/LAB2_OLED/build/bootloader-prefix/src/bootloader-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "D:/Phan_Thanh_Tuan/UIT-Online/HK6/TH_TKHTNKD/LAB2_OLED/build/bootloader-prefix/src/bootloader-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "D:/Phan_Thanh_Tuan/UIT-Online/HK6/TH_TKHTNKD/LAB2_OLED/build/bootloader-prefix/src/bootloader-stamp${cfgdir}") # cfgdir has leading slash
endif()
