cmake_minimum_required(VERSION 3.10)

project(o3d_cpp_py)

find_package(Python3 REQUIRED COMPONENTS Development Interpreter)

# Open3D
find_package(Open3D REQUIRED)

Python3_add_library(o3d_cpp_py MODULE use_o3d.cpp)
target_link_libraries(o3d_cpp_py PRIVATE
  Python3::Module
  Open3D::Open3D
)

# davidcaron/pclpy
ExternalProject_Add(pclpy
  URL https://github.com/davidcaron/pclpy/archive/refs/tags/0.12.0.tar.gz
  URL_HASH SHA256=720b54bb5ac282a5994192caf10b9799ef4c30dc2f9d4cd98568c85daf20a230
