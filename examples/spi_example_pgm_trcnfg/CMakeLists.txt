set(PROJECT_NAME spi_example_pgm_trcnfg)

set(SPI_EXAMPLE_PGM_TRCNFG_SRC_FILES
        spi_example_pgm_trcnfg.c
        )

include_directories(${CMAKE_SOURCE_DIR}/include)

add_executable(${PROJECT_NAME} ${SPI_EXAMPLE_PGM_TRCNFG_SRC_FILES})
target_link_libraries(${PROJECT_NAME}
        spi_iqrf)

install(TARGETS ${PROJECT_NAME} DESTINATION ${CMAKE_INSTALL_PREFIX}/sbin)
