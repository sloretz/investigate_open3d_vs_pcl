cmake_minimum_required(VERSION 3.10)

project(o3d_cpp_py)

find_package(benchmark REQUIRED)

find_package(Open3D REQUIRED)
find_package(PCL REQUIRED)

add_executable(voxel_downsample voxel_downsample.cpp)
target_link_libraries(voxel_downsample PRIVATE
  Open3D::Open3D
  benchmark::benchmark
  ${PCL_LIBRARIES}
)
target_include_directories(voxel_downsample PRIVATE
  ${PCL_INCLUDE_DIRS}
)
