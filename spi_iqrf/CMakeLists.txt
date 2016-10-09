set(SPIDEV_SRC_FILES
	spi_iqrf.c
)

include_directories(${CMAKE_SOURCE_DIR}/include)

set(SPIDEV_H_FILES
	${CMAKE_SOURCE_DIR}/include/spi_iqrf.h
)

# Group the files in IDE.
source_group("include" FILES ${SPIDEV_H_FILES})

add_library(spi_iqrf
        STATIC
		${SPIDEV_SRC_FILES}
		${SPIDEV_H_FILES}
)

target_link_libraries(spi_iqrf sysfs_gpio)

#install(TARGETS spi_iqrf DESTINATION ${CMAKE_INSTALL_PREFIX}/lib)
