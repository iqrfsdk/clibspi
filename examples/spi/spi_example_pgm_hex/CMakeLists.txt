set(PROJECT_NAME spi_example_pgm_hex)

set(SPI_EXAMPLE_PGM_HEX_SRC_FILES
        spi_example_pgm_hex.c
        )

include_directories(${CMAKE_SOURCE_DIR}/include)

add_executable(${PROJECT_NAME} ${SPI_EXAMPLE_PGM_HEX_SRC_FILES})
target_link_libraries(${PROJECT_NAME}
        sysfs_gpio
        spi_iqrf)

install(TARGETS ${PROJECT_NAME} DESTINATION ${CMAKE_INSTALL_PREFIX}/sbin)
