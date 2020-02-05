import copy
import open3d as o3d
import numpy as np

GT_COLOR = [0, 0.651, 0.929]
RECONS_COLOR = [1,0.1,1]

def create_spheres(pc, color=[0,0,1], radius=0.05):
    """
    Create a list of spheres from a open3d point cloud
    Numpy array needs to be N-by-3
    """
    data = np.asarray(pc.points)
    vis_list = []
    for row in range(data.shape[0]):
        c_pt = data[row, :]
        mesh_sphere = o3d.geometry.TriangleMesh.create_sphere(radius=radius)
        mesh_sphere.compute_vertex_normals()
        mesh_sphere.paint_uniform_color(color)
        mesh_sphere.translate(c_pt)
        vis_list.append(mesh_sphere)
    return vis_list

def load_gt_cad(gt_cad_path):
    mesh = o3d.io.read_triangle_mesh(gt_cad_path)
    mesh.compute_triangle_normals()
    mesh.paint_uniform_color(GT_COLOR)
    return mesh

def load_recons(recons_path):
    pc = o3d.io.read_point_cloud(recons_path)
    return pc

def load_harris_keypoints(kp_path):
    pc = o3d.io.read_point_cloud(kp_path)
    return pc

def custom_draw_geometry_with_rotation(vis_list):

    def rotate_view(vis):
        ctr = vis.get_view_control()
        ctr.rotate(2.0, 0.0)
        return False

    o3d.visualization.draw_geometries_with_animation_callback(vis_list,
                                                              rotate_view)

def visualize(R, t, gt_mesh, recons, keypoints):
    """
    Visualization function
    """
    trans_mat = np.zeros((4,4))
    trans_mat[:3,:3] = R
    trans_mat[:3,3] = t
    trans_mat[3,3] = 1

    inv_trans_mat = np.linalg.inv(trans_mat)

    transformed_gt_mesh = copy.deepcopy(gt_mesh)
    #transformed_gt_mesh.transform(trans_mat)

    translation = np.array([-10,0,0])
    keypoints.transform(inv_trans_mat)
    recons.transform(inv_trans_mat)

    transformed_gt_mesh.translate(translation)
    keypoints.translate(translation)
    recons.translate(translation)

    # create keypoints
    kp_spheres = create_spheres(keypoints)

    recons.paint_uniform_color(RECONS_COLOR)
    
    # visualization
    vis_list = [transformed_gt_mesh]
    vis_list.extend(kp_spheres)
    o3d.visualization.draw_geometries(vis_list)

    vis_list.append(recons)
    o3d.visualization.draw_geometries(vis_list)


if __name__ == "__main__":
    print("Visualization script for figure 1")

    # What I need:
    # 1. GT transformed CAD model
    # 2. Reconstructed point clouds
    # 3. Harris keypoints
    gt_cad_path = "./Couch_1seat.obj"
    recons_path = "./recons.ply"
    kp_path = "./keypoints.pcd"

    R1 = np.array([ 
   [0.222422, 0.0135435, 0.974856],
   [0.974926, 0.00405288, -0.222494],
   [-0.00696433, 0.9999, -0.0123025]])

    t1 = np.array([2.98689,18.8833,-1.02605])

    gt_mesh = load_gt_cad(gt_cad_path)
    recons = load_recons(recons_path)
    kps = load_harris_keypoints(kp_path)
    kps.points = o3d.utility.Vector3dVector(np.asarray(kps.points)[:12,:])
    visualize(R1, t1, gt_mesh, recons, kps)
