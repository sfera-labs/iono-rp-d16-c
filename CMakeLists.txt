cmake_minimum_required(VERSION 3.12)

# Pull in SDK (must be before project)
include(pico_sdk_import.cmake)

project(iono_rp_d16)

add_subdirectory(lib/iono_rp_d16)
add_subdirectory(example)
