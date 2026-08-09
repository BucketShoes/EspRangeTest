# Host build of rt_core plus its tests. No ESP-IDF, no hardware, no external test
# framework - just a C compiler.
#
#   cmake -S . -B build-host && cmake --build build-host && ctest --test-dir build-host
cmake_minimum_required(VERSION 3.16)
project(rt_core_host_tests C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

add_compile_options(-Wall -Wextra -Werror -Wconversion -Wshadow -O1 -g)

add_library(rt_core STATIC
    ../src/rt_chan.c
    ../src/rt_payload.c
    ../src/rt_obs.c
    ../src/rt_mode.c
    ../src/rt_report.c
    ../../rt_button/src/rt_button_sm.c
)
target_include_directories(rt_core PUBLIC ../include ../../rt_button/include)

enable_testing()

foreach(t rt_payload rt_obs rt_mode rt_report rt_button)
    add_executable(test_${t} test_${t}.c)
    target_link_libraries(test_${t} PRIVATE rt_core)
    add_test(NAME ${t} COMMAND test_${t})
endforeach()
