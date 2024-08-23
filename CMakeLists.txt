cmake_minimum_required(VERSION 3.28)
project(mesh_intersection)

set(CMAKE_CXX_STANDARD 20)
set(CGAL_DO_NOT_WARN_ABOUT_CMAKE_BUILD_TYPE TRUE)

find_package(CGAL REQUIRED)

add_executable(mesh_intersection main.cpp)

target_link_libraries(${PROJECT_NAME} CGAL::CGAL)