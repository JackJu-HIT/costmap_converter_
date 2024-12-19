cmake_minimum_required(VERSION 2.8.3)
project(costmap_2d)
set(CMAKE_CXX_FLAGS "${CMAKE_C_FLAGS} -O2  ")
find_package(
  Boost REQUIRED COMPONENTS system
        )
find_package(PCL 1.7 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
find_package(OpenCV REQUIRED)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -pthread")
include_directories(
    include
)
find_package(Boost REQUIRED system)
include_directories(${Boost_INCLUDE_DIRS}
                     ${OpenCV_INCLUDE_DIRS})

include_directories(matplotlib-cpp)

add_library(costmap_converter   src/costmap_to_polygons.cpp 
                                src/costmap_to_polygons_concave.cpp
                                src/costmap_to_lines_convex_hull.cpp
                                src/costmap_to_lines_ransac.cpp
                                src/costmap_converter_manager.cpp
                              
)


add_executable(main main.cpp)

target_link_libraries(
      main
     costmap_converter
     ${Boost_SYSTEM_LIBRARY}
     ${OpenCV_LIBS}
     python2.7
     ${PYTHON_LIBRARIES}
     ${PCL_LIBRARIES} 
)