#include <memory>

#include <pybind11/pybind11.h>
#include <Open3D/Geometry/PointCloud.h>


std::shared_ptr<open3d::geometry::PointCloud> voxel_down_sample(const open3d::geometry::PointCloud & pc_in) {
    return pc_in.VoxelDownSample(0.05);
}

PYBIND11_MODULE(o3d_cpp_py, m) {
    m.doc() = "Investigate Open3D C++ and Python interop";
    m.def("voxel_down_sample", &voxel_down_sample, "Downsample a point cloud");
}
