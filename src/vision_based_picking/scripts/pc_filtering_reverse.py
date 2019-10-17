#!/usr/bin/env python

import numpy as np
import open3d as o3d
import copy

def filter_pc(cl, voxel_size, sor_params=None, ror_params=None):
    # sor_params = [nb_neighbors, std_ratio]
    # ror_params = [nb_neighbors, radius]
    if ror_params != None:
        cl, ind = cl.remove_radius_outlier(ror_params[0], ror_params[1])
    if sor_params != None:
        cl, ind = cl.remove_statistical_outlier(sor_params[0], sor_params[1])
    cl = cl.voxel_down_sample(voxel_size)

    print("Showing filtered point cloud")
    o3d.visualization.draw_geometries([cl])

if __name__ == "__main__":

    # pc_location = "PointCloud2.pcd"
    # pc_location = "PointClouds/Turquoise Tall Bowl/Whole_Scene_Camera1.pcd"
    # pc_location = "PointClouds/Turquoise Small Bowl/Whole_Scene_Camera2.pcd"
    pc_location = "PointClouds/Blue Bowl/Whole_Scene_Camera2.pcd"
    # pc_location = "PointClouds/Blue Plate/Whole_Scene_Camera2.pcd"
    # pc_location = "PointClouds/Blue Glass/Whole_Scene_Camera1.pcd"
    # pc_location = "PointClouds/Turquoise Utensils/Whole_Scene_Camera1.pcd"
    # pc_location = "PointClouds/Blue Utensils/Whole_Scene_Camera2.pcd"

    print("Load a point cloud")
    pcd = o3d.io.read_point_cloud(pc_location)
    o3d.visualization.draw_geometries([pcd])

    filter_pc(pcd, 0.005, [20, 1.0], [20, 0.01])
