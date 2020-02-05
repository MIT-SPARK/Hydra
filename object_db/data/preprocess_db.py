import os
import csv
import argparse

import open3d as o3d

def gen_sampled_point_cloud(obj_file_path, dest_path, num_points=10000):
    """
    Load obj file, sample and save

    obj_file_path: path of the object file to be loaded
    dest_path: path to store the sampled point cloud as ply file
    """
    mesh = o3d.io.read_triangle_mesh(obj_file_path)
    pc = mesh.sample_points_poisson_disk(num_points)
    o3d.io.write_point_cloud(dest_path, pc)
    return pc

def get_all_model_obj_paths(db_path):
    """
    Return a list of all obj files in the database
    """
    sub_dirs = [os.path.join(db_path, name) for name in os.listdir(db_path)
            if os.path.isdir(os.path.join(db_path, name))]

    obj_files = [os.path.join(fpath, "model.obj") for fpath in sub_dirs]

    return obj_files, sub_dirs 

def main(db_path):
    """
    Main function
    """
    obj_paths, sub_dirs = get_all_model_obj_paths(db_path)
    for obj_f, sub_dir in zip(obj_paths, sub_dirs):
        dest_path = os.path.join(sub_dir, "model.ply")
        gen_sampled_point_cloud(obj_f, dest_path)

if __name__ == "__main__":
    print("Preprocess CAD db for KimeraX.")

    parser = argparse.ArgumentParser(description='Preprocess CAD db for KimeraX.')
    parser.add_argument('--db_path', type=str, dest='db_path',default="./simulation_db/",help='path to the databse')

    args = parser.parse_args()
    db_path = args.db_path
    main(db_path)

