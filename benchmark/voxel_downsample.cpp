#include <cassert>
#include <memory>

#include <benchmark/benchmark.h>

#include <Open3D/Geometry/PointCloud.h>
#include <Open3D/IO/ClassIO/PointCloudIO.h>

static
void BM_open3d(benchmark::State& state)
{
  open3d::geometry::PointCloud pcd;
  std::shared_ptr<open3d::geometry::PointCloud> downsampled_pcd;
  assert(open3d::io::ReadPointCloud("bunny.pcd", pcd));

  for (auto _ : state) {
    downsampled_pcd = pcd.VoxelDownSample(0.05);
  }
}

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

static
void BM_pcl(benchmark::State& state)
{
  auto pcd = std::make_shared<pcl::PCLPointCloud2>();
  auto downsampled_pcd = std::make_shared<pcl::PCLPointCloud2>();

  pcl::PCDReader reader;
  reader.read("bunny.pcd", *pcd);

  pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
  sor.setInputCloud(pcd);
  sor.setLeafSize(0.05f, 0.05f, 0.05f);

  for (auto _ : state) {
    sor.filter(*downsampled_pcd);
  }
}

BENCHMARK(BM_open3d);
BENCHMARK(BM_pcl);
BENCHMARK_MAIN();
