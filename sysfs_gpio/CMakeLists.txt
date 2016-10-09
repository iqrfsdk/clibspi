set(SYSFS_GPIO_SRC_FILES
	sysfs_gpio.c
)

include_directories(${CMAKE_SOURCE_DIR}/include)

set(SYSFS_GPIO_H_FILES
	${CMAKE_SOURCE_DIR}/include/sysfs_gpio.h
	${CMAKE_SOURCE_DIR}/include/machines_def.h
)

# Group the files in IDE.
source_group("include" FILES ${SYSFS_GPIO_H_FILES})

add_library(sysfs_gpio
        STATIC
		${SYSFS_GPIO_SRC_FILES}
		${SYSFS_GPIO_H_FILES}
)

#install(TARGETS sysfs_gpio DESTINATION ${CMAKE_INSTALL_PREFIX}/lib)
