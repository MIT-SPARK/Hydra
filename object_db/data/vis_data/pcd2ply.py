import open3d as o3d

recons_path = "./recons.pcd"
pc = o3d.io.read_point_cloud(recons_path)
o3d.io.write_point_cloud("./recons.ply", pc)

kp_path = "./keypoints.pcd"
pc = o3d.io.read_point_cloud(kp_path)
o3d.io.write_point_cloud("./keypoints.ply", pc)
