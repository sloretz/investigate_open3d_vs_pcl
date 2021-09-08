cmake_minimum_required(VERSION 3.10)

project(o3d_cpp_py)

find_package(Open3D REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Development)

Python3_add_library(o3d_cpp_py MODULE use_o3d.cpp)
target_link_libraries(o3d_cpp_py PRIVATE
  Python3::Module
  Open3D::Open3D
)
