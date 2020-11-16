cmake_minimum_required(VERSION 2.8.3)

project(lidar_demos)

# Point Cloud Library (PCL)
find_package(PCL 1.09 REQUIRED COMPONENTS)
include_directories(include ${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# Computational Geometry Algorithms Library (CGAL)
set(CMAKE_BUILD_TYPE "Release")
find_package(CGAL QUIET COMPONENTS)
if(NOT CGAL_FOUND)
  message(STATUS "This project requires CGAL and will not be compiled.")
  return()
endif()

# Boost and its components
find_package( Boost REQUIRED )
if ( NOT Boost_FOUND )
  message(STATUS "This project requires the Boost library, and will not be compiled.")
  return()
endif()

# PROJ 
find_package(PROJ)
include_directories(${PROJ_INCLUDE_DIRS})


add_executable(lidar_demo src/lidar_demo.cpp)
target_link_libraries(lidar_demo ${PCL_LIBRARIES} gdal ${CGAL_LIBRARIES} ${PROJ_LIBRARIES} point_cloud_rasterizer las_classifier las_surfacing)



install(TARGETS lidar_demo
        DESTINATION lib)
install(DIRECTORY include/${PROJECT_NAME}
        DESTINATION include
        FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp")