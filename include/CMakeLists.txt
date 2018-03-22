set(_ipath ${PROJECT_SOURCE_DIR}/include/)

file(GLOB ALL_HEADERS ${_ipath}/*.h ${_ipath}/*.hpp)

add_custom_target(all_spi_include SOURCES ${ALL_HEADERS})
