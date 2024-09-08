cmake_minimum_required(VERSION 3.28)
project(mesh_intersection)

set(CMAKE_CXX_STANDARD 20)
set(CGAL_DO_NOT_WARN_ABOUT_CMAKE_BUILD_TYPE TRUE)

set(CMAKE_AUTOMOC ON)
SET(CMAKE_AUTOUIC ON)

# 添加编译器标志
if(MSVC)
    add_compile_options(/bigobj)
    set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>")
    set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /MT")
    set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} /MTd")

elseif(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    # 添加针对 Clang 编译器的编译选项
    # add_compile_options(-Wa,-mbig-obj)
else()
    # 对于其他编译器，也可以在这里添加相应的编译选项
    add_compile_options(-Wa,-mbig-obj)
endif()

find_package(CGAL REQUIRED OPTIONAL_COMPONENTS Qt5)


file(GLOB EXAMPLES "include/examples/*.h")

add_executable(mesh_intersection main.cpp ${EXAMPLES})

target_include_directories(mesh_intersection PRIVATE include)
target_link_libraries(${PROJECT_NAME} CGAL::CGAL)