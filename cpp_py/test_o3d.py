import open3d as o3d
import o3d_cpp_py

pcd = o3d.io.read_point_cloud("example.pcd")
down_pcd = o3d_cpp_py.voxel_down_sample(pcd)
down_pcd.paint_uniform_color([1, 0.706, 0])

o3d.visualization.draw_geometries([pcd, down_pcd])
