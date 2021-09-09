#include <cassert>
#include <memory>

#include <benchmark/benchmark.h>

#include <Open3D/Geometry/PointCloud.h>
#include <Open3D/IO/ClassIO/PointCloudIO.h>
#include <Open3D/Registration/Registration.h>

static
void BM_open3d(benchmark::State& state)
{
  open3d::geometry::PointCloud pcd;
  assert(open3d::io::ReadPointCloud("bunny.pcd", pcd));

  auto pcd_rotated = open3d::geometry::PointCloud(pcd.points_);
  Eigen::Matrix3d rot;
  rot << 0.36, 0.48, -0.8,
        -0.8, 0.6, 0,
        0.48, 0.64, 0.6;
  pcd_rotated.Rotate(rot, true);

  std::shared_ptr<open3d::geometry::PointCloud> pcd_final;

  const double max_correspondence_distance = 10;

  for (auto _ : state) {
    open3d::registration::RegistrationICP(pcd, pcd_rotated, max_correspondence_distance);
  }
}

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>

static
void BM_pcl(benchmark::State& state)
{
  auto pcd = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  auto pcd_rotated = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

  pcl::PCDReader reader;
  reader.read("bunny.pcd", *pcd);

  // rotate bunny point cloud
  Eigen::Matrix4d rot;
  rot << 0.36, 0.48, -0.8, 0,
        -0.8, 0.6, 0, 0,
        0.48, 0.64, 0.6, 0,
        0, 0, 0, 1;
  pcl::transformPointCloud(*pcd, *pcd_rotated, rot);

  pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
  icp.setInputSource(pcd);
  icp.setInputTarget(pcd_rotated);
  icp.setMaxCorrespondenceDistance(10);
  icp.setMaximumIterations(30); // Open3D Defaults to 30 iterations max
  
  pcl::PointCloud<pcl::PointXYZ> pcd_final;

  for (auto _ : state) {
    icp.align(pcd_final);
    assert(icp.hasConverged());
  }
}

BENCHMARK(BM_open3d);
BENCHMARK(BM_pcl);
BENCHMARK_MAIN();

