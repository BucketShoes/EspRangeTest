# EspRangeTest firmware.
#
# One codebase, one build per target - C3 and S3 have no 802.15.4 radio, so channel
# availability is compile-time gated (see probe_channels() in main/app_main.c) rather than
# assumed. Build for a specific board with:
#
#   idf.py set-target esp32c6 && idf.py build flash monitor
#
cmake_minimum_required(VERSION 3.16)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(esprangetest)
