project(clibspi)

cmake_minimum_required(VERSION 3.18)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_EXTENSIONS ON)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_EXTENSIONS ON)

set(CMAKE_POSITION_INDEPENDENT_CODE ON)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_LIST_DIR}/cmake" CACHE STRING "Modules for CMake" FORCE)

set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

include(${PROJECT_SOURCE_DIR}/cmake/ccache.cmake)
include(${PROJECT_SOURCE_DIR}/cmake/hardening.cmake)

find_package(JNI)

message(STATUS "CMAKE_CXX_STANDARD:      ${CMAKE_CXX_STANDARD}")
message(STATUS "CMAKE_CXX_FLAGS:         ${CMAKE_CXX_FLAGS}")
message(STATUS "CMAKE_CXX_FLAGS_RELEASE: ${CMAKE_CXX_FLAGS_RELEASE}")
message(STATUS "CMAKE_CXX_FLAGS_DEBUG:   ${CMAKE_CXX_FLAGS_DEBUG}")
message(STATUS "CMAKE_C_FLAGS:           ${CMAKE_C_FLAGS}")
message(STATUS "CMAKE_C_FLAGS_RELEASE:   ${CMAKE_C_FLAGS_RELEASE}")
message(STATUS "CMAKE_C_FLAGS_DEBUG:     ${CMAKE_C_FLAGS_DEBUG}")

#set(MACHINE RPI)

# if(MACHINE)
	# add_definitions(-D${MACHINE})
# endif(MACHINE)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

add_subdirectory(include)
add_subdirectory(spi_iqrf)

if (JNI_FOUND)
  add_subdirectory(spi_javastub)
else()
  message("Java not found => spi_javastub cannot be configured")
endif()

get_directory_property(hasParent PARENT_DIRECTORY)
if(hasParent)
    set(clibspi_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include PARENT_SCOPE)
    set(clibspi_FOUND TRUE PARENT_SCOPE)
endif()

add_subdirectory(examples)

# Configure config file.
# This file specifies actions performed and variables exported when using find_package on this project.
# The find_package requires properly set ${PROJECT_NAME}_DIR variable to a location where the
# configured config file persists. This variable is usually specified from the outside of CMake.
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/${PROJECT_NAME}Config.cmake.in ${PROJECT_BINARY_DIR}/${PROJECT_NAME}Config.cmake @ONLY)
