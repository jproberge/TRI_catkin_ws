from segment_bowl import *
import open3d as o3d
import numpy as np
import copy


def draw_registration_result(source, target, transformation):
    source_temp = copy.deepcopy(source)
    target_temp = copy.deepcopy(target)
    source_temp.paint_uniform_color([1, 0.706, 0])
    target_temp.paint_uniform_color([0, 0.651, 0.929])
    source_temp.transform(transformation)
    o3d.visualization.draw_geometries([source_temp, target_temp])

if __name__ == "__main__":
    T_O_C = np.load('T_O_C.npy')

    source_pcd = get_bowl_pcd("data/Whole_Scene_Camera1_Bowl.pcd", "data/Whole_Scene_Camera2_Bowl.pcd")
    target_pcd = o3d.io.read_point_cloud("data/bowl_model.pcd")

    # find their centroid distances
    source_pcd_pts = np.asarray(source_pcd.points)
    source_pcd_centroid = np.mean(source_pcd_pts, axis=0)
    target_pcd_pts = np.asarray(target_pcd.points)
    target_pcd_centroid = np.mean(target_pcd_pts, axis=0)
    r_s_t = target_pcd_centroid - source_pcd_centroid 

    # perform icp
    threshold = 0.2
    T_init = np.eye(4)
    T_init[:3,3] = r_s_t

    draw_registration_result(source_pcd, target_pcd, T_init)

    print("Initial alignment")
    evaluation = o3d.registration.evaluate_registration(source_pcd, target_pcd,
                                                        threshold, T_init)
    print(evaluation)


    print("Apply point-to-plane ICP")
    reg_p2l = o3d.registration.registration_icp(
        source_pcd, target_pcd, threshold, T_init,
        o3d.registration.TransformationEstimationPointToPoint())
    print(reg_p2l)
    print("Transformation is:")
    print(reg_p2l.transformation)
    print("")
    draw_registration_result(source_pcd, target_pcd, reg_p2l.transformation)

    T_C_A = reg_p2l.transformation

    # we want to find now T_O_A
    T_O_A = T_O_C.dot(T_C_A) 

    print("Object pose in robot frame is:\n%s" % T_O_A)
