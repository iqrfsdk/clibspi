project(spi_iqrf)

set(_SRC_FILES
  spi_iqrf.c
)

add_library(${PROJECT_NAME} STATIC ${_SRC_FILES})

target_link_libraries(${PROJECT_NAME} sysfs_gpio)

