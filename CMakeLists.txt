cmake_minimum_required(VERSION 3.5 FATAL_ERROR)

project(read_pcd)

find_package(PCL 1.11 REQUIRED)

# set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_CXX_STANDARD 17)

include_directories(${PCL_INCLUDE_DIRS}
                    "${CMAKE_CURRENT_SOURCE_DIR}/include"
                    )

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

set(SOURCES src/main.cpp
            src/NormalityDetection.cpp
            )

add_executable(plane_normality ${SOURCES}) 
target_link_libraries (plane_normality ${PCL_LIBRARIES})