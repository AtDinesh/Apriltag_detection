project(Apriltag_detection C CXX)
set (CMAKE_CXX_STANDARD 11)

cmake_minimum_required ( VERSION 2.8 )

find_package( OpenCV REQUIRED )
include(FindPkgConfig)
pkg_search_module(OPENCV opencv>=2.3 opencv-2.3.1 opencv-3.0.0)

if(OPENCV_FOUND)
  message(STATUS "opencv found: ${OpenCV_INCLUDE_DIRS}")
  include_directories(${OPENCV_INCLUDE_DIRS})
else(NOT OPENCV_FOUND)
  message(FATAL_ERROR "opencv_include_dirs not found")
endif(OPENCV_FOUND)

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})


#It is not possible to find apriltag library using find_package yet.
#We need to use find_path and find_library.
#The install folder of the library should be in the CMAKE_PREFIX_PATH.
#The pkgconfig folder in which the apriltag.pc file is should be in the PKG_CONFIG_PATH.

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

find_package (Threads REQUIRED)

INCLUDE_DIRECTORIES(${APRILTAG_INCLUDE_DIR})

link_libraries(apriltag m)
#SET(Apriltag_detection_SOURCES <files>)
#SET(Apriltag_detection_HEADERS <files>)

set(CMAKE_REQUIRED_LIBRARIES ${CMAKE_REQUIRED_LIBRARIES} -lm)

set(EXTRA_FLAGS "-Wall ${CMAKE_THREAD_LIBS_INIT}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${EXTRA_FLAGS} ${CMAKE_REQUIRED_LIBRARIES}")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${EXTRA_C_FLAGS} ${CMAKE_REQUIRED_LIBRARIES}")

set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR})
#set(LIBRARY_OUTPUT_PATH ${CMAKE_BINARY_DIR}/lib)

add_executable(apriltag_basis_test examples/apriltag_basis_test.cpp)
target_link_libraries(apriltag_basis_test ${APRILTAG_LIBRARY} ${CMAKE_THREAD_LIBS_INIT} ${OPENCV_LDFLAGS} m)

add_executable(pose_estimation_benchmark examples/pose_estimation_benchmark.cpp)
target_link_libraries(pose_estimation_benchmark ${APRILTAG_LIBRARY} ${CMAKE_THREAD_LIBS_INIT} ${OPENCV_LDFLAGS} m)