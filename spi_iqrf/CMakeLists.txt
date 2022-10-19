project(spi_iqrf)

set(_SRC_FILES iqrf_gpio.c spi_iqrf.c)

add_library(${PROJECT_NAME} STATIC ${_SRC_FILES})
