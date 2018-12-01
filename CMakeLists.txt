project(Apriltag_detection C CXX)

cmake_minimum_required ( VERSION 2.8 )

find_package( OpenCV 2.4 REQUIRED )

if(OPENCV_FOUND)
  include_directories(${OPENCV_INCLUDE_DIRS})
  #add_subdirectory(opencv)
endif(OPENCV_FOUND)

#it is not possible to find apriltag library using find_package yet.
#we need to use find_path and find_library.
#the install folder of the library should be in the CMAKE_PREFIX_PATH
#the pkgconfig in which the apriltag.pc file is should be in the PKG_CONFIG_PATH

find_path(APRILTAG_INCLUDE_DIR NAMES apriltag.h PATH_SUFFIXES "apriltag" ${APRILTAG_INCLUDE_PATH})
find_library(APRILTAG_LIBRARY NAMES apriltag PATH_SUFFIXES "${CMAKE_LIBRARY_ARCHITECTURE}" "apriltag" ${APRILTAG_LIBRARY_PATH})

if(NOT APRILTAG_LIBRARY)
  message(FATAL_ERROR "apriltag not found")
else(APRILTAG_LIBRARY)
  message(STATUS "apriltag found: ${APRILTAG_LIBRARY}")
endif()

if(NOT APRILTAG_INCLUDE_DIR)
  message(FATAL_ERROR "apriltag_include_dir not found")
else(APRILTAG_INCLUDE_DIR)
  message(STATUS "apriltag found: ${APRILTAG_INCLUDE_DIR}")
endif()

INCLUDE_DIRECTORIES(${APRILTAG_INCLUDE_DIR})

#SET(Apriltag_detection_SOURCES <files>)
#SET(Apriltag_detection_HEADERS <files>)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${EXTRA_FLAGS}")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${EXTRA_C_FLAGS}")

set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR})
#set(LIBRARY_OUTPUT_PATH ${CMAKE_BINARY_DIR}/lib)

#add_subdirectory(examples)

#include_directories(include ${OpenCV_INCLUDE_DIRS} ${apriltag_INCLUDE_DIRS})

add_executable(apriltag_basis_test examples/apriltag_basis_test.cpp)
target_link_libraries(apriltag_basis_test ${APRILTAG_LIBRARY} )