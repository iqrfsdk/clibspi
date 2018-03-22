project(sysfs_gpio)

set(_SRC_FILES
  sysfs_gpio.c
)

add_library(${PROJECT_NAME} STATIC ${_SRC_FILES})

