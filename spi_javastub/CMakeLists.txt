set(_name spi_javastub)

find_package(Java REQUIRED)
find_package(JNI REQUIRED)

if (JNI_FOUND)
	message (STATUS "JNI_INCLUDE_DIRS=${JNI_INCLUDE_DIRS}")
	message (STATUS "JNI_LIBRARIES=${JNI_LIBRARIES}")
	include_directories(${JNI_INCLUDE_DIRS})
endif()

set(spi_javastub_SRC_FILES
	spi_javastub.cpp
)

set(spi_javastub_INC_FILES
	com_microrisc_spi_SimpleSPI_Master.h
)

# Group the files in IDE.
source_group("include" FILES ${spi_javastub_INC_FILES})

include_directories(${CMAKE_CURRENT_SOURCE_DIR})
include_directories(${CMAKE_SOURCE_DIR}/include)

add_library(
	${_name}
	SHARED
	${spi_javastub_SRC_FILES} 
	${spi_javastub_INC_FILES}
)

target_link_libraries(${_name} spi_iqrf)

#install(TARGETS ${project} DESTINATION ${CMAKE_INSTALL_PREFIX}/lib)
