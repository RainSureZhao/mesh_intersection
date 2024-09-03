cmake_minimum_required(VERSION 3.28)
project(mesh_intersection)

set(CMAKE_CXX_STANDARD 20)
set(CGAL_DO_NOT_WARN_ABOUT_CMAKE_BUILD_TYPE TRUE)

set(CMAKE_AUTOMOC ON)
SET(CMAKE_AUTOUIC ON)
SET(CGAL_DO_NOT_WARN_ABOUT_CMAKE_BUILD_TYPE TRUE)

# 添加编译器标志
if(MSVC)
    add_compile_options(/bigobj)
else()
    add_compile_options(-Wa,-mbig-obj)
endif()

find_package(CGAL REQUIRED OPTIONAL_COMPONENTS Qt5)

find_package(CGAL REQUIRED)

file(GLOB EXAMPLES "include/examples/*.h")

add_executable(mesh_intersection main.cpp ${EXAMPLES})

target_link_libraries(${PROJECT_NAME} CGAL::CGAL)