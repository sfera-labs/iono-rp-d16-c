add_library(iono_rp_d16 INTERFACE)
target_sources(iono_rp_d16 INTERFACE 
    ${CMAKE_CURRENT_LIST_DIR}/iono_rp_d16.c
    )
target_link_libraries(iono_rp_d16 INTERFACE hardware_spi)
target_include_directories(iono_rp_d16 INTERFACE ${CMAKE_CURRENT_LIST_DIR}/include)
