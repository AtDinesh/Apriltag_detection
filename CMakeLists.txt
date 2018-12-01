project(Apriltag_detection C CXX)

cmake_minimum_required ( VERSION 2.8 )

find_package( OpenCV 2.4 REQUIRED )

if(OPENCV_FOUND)
  include_directories(${OPENCV_INCLUDE_DIRS})
  #add_subdirectory(opencv)
endif(OPENCV_FOUND)

find_path(APRILTAG_INCLUDE_DIR NAMES apriltag.h PATH_SUFFIXES "apriltag" ${APRILTAG_INCLUDE_PATH})
find_library(APRILTAG_FOUND apriltag)

if(APRILTAG_FOUND)
  include_directories(${APRILTAG_FOUND})
  #add_subdirectory(opencv)
endif(APRILTAG_FOUND)

if(NOT APRILTAG_FOUND)
  message(FATAL_ERROR "apriltag not found")
else(APRILTAG_FOUND)
  message(STATUS "apriltag found: ${APRILTAG_FOUND}")
endif()

if(NOT APRILTAG_INCLUDE_DIR)
  message(FATAL_ERROR "apriltag_include_dir not found")
else(APRILTAG_INCLUDE_DIR)
  message(STATUS "apriltag found: ${APRILTAG_INCLUDE_DIR}")
endif()

#SET(Apriltag_detection_SOURCES <files>)
#SET(Apriltag_detection_HEADERS <files>)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${EXTRA_FLAGS}")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${EXTRA_C_FLAGS}")

set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR})
set(LIBRARY_OUTPUT_PATH ${CMAKE_BINARY_DIR}/lib)

#add_subdirectory(src)
#include_directories(src)

#INCLUDE()
#ADD_DEFINITIONS()

#include_directories(include ${OpenCV_INCLUDE_DIRS} ${apriltag_INCLUDE_DIRS})

add_executable(apriltag_basis_test src/apriltag_basis_test.cpp)

target_link_libraries(apriltag_basis_test ${APRILTAG_FOUND} )