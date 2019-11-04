from segment_bowl import *
import open3d as o3d
import numpy as np
import copy
import time

import rospy
import tf
import geometry_msgs.msg
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header

from scipy.spatial.transform import Rotation as R


def draw_registration_result(source, target, transformation):
    source_temp = copy.deepcopy(source)
    target_temp = copy.deepcopy(target)
    source_temp.paint_uniform_color([1, 0.706, 0])
    target_temp.paint_uniform_color([0, 0.651, 0.929])
    source_temp.transform(transformation)
    o3d.visualization.draw_geometries([source_temp, target_temp])

def pc_msg_gen(pc_pts, frame_id):
    
    header = Header()
    header.frame_id = "cam1"

    msg = PointCloud2() 
    msg.fields = [
        PointField('x', 0, PointField.FLOAT32, 1),
        PointField('y', 4, PointField.FLOAT32, 1),
        PointField('z', 8, PointField.FLOAT32, 1)]
    msg.header = header

    msg.height = 1
    msg.width = len(pc_pts)
    msg.is_bigendian = False
    msg.point_step = 12
    msg.row_step = 12*pc_pts.shape[0]
    msg.is_dense = int(np.isfinite(pc_pts).all())
    msg.data = np.asarray(pc_pts, np.float32).tostring()

    return msg


if __name__ == "__main__":
    #T_O_C = np.load('T_O_C.npy')

    rospy.init_node("find_grasp", anonymous=True, disable_signals=True)
    pub = rospy.Publisher('bowl_pcd', PointCloud2, queue_size=10)

    T_O_C = np.genfromtxt('../../../CalibrationMatrices/O_T_C1.csv', delimiter=',')[:,:4]
    T_O_C = np.vstack((T_O_C, np.array([0, 0, 0, 1])))
    T_O_C2 = np.genfromtxt('../../../CalibrationMatrices/O_T_C2.csv', delimiter=',')[:,:4]
    T_O_C2 = np.vstack((T_O_C2, np.array([0, 0, 0, 1])))

    source_pcd = get_bowl_pcd("data/Whole_Scene_Camera1_Bowl.pcd", "data/Whole_Scene_Camera2_Bowl.pcd")
    target_pcd = o3d.io.read_point_cloud("data/bowl_model.pcd")

    # find their centroid distances
    source_pcd_pts = np.asarray(source_pcd.points)
    source_pcd_centroid = np.mean(source_pcd_pts, axis=0)
    target_pcd_pts = np.asarray(target_pcd.points)
    target_pcd_centroid = np.mean(target_pcd_pts, axis=0)
    r_s_t = target_pcd_centroid - source_pcd_centroid 

    # perform icp
    threshold = 0.05
    T_init = np.eye(4)
    T_init[:3,3] = r_s_t

    #draw_registration_result(source_pcd, target_pcd, T_init)

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
    #draw_registration_result(source_pcd, target_pcd, reg_p2l.transformation)

    T_C_A = reg_p2l.transformation

    # we want to find now T_O_A
    T_O_A = T_O_C.dot(T_C_A) 


    # transform model points by T_O_A
    target_pcd_pts_homogeneous = np.hstack((target_pcd_pts, np.ones((target_pcd_pts.shape[0], 1))))
    target_pcd_pts_O = ((T_O_A.dot(target_pcd_pts_homogeneous.T)).T)[:,:3]

    msg = PointCloud2() 
    msg.fields = [
        PointField('x', 0, PointField.FLOAT32, 1),
        PointField('y', 4, PointField.FLOAT32, 1),
        PointField('z', 8, PointField.FLOAT32, 1)]


    # transform model points by T_O_A
    source_pcd_pts_homogeneous = np.hstack((source_pcd_pts, np.ones((source_pcd_pts.shape[0], 1))))
    source_pcd_pts_O = ((T_O_C.dot(source_pcd_pts_homogeneous.T)).T)[:,:3]


    br = tf.TransformBroadcaster()

    while (not rospy.is_shutdown()):
        time.sleep(1)

        pub.publish(pc_msg_gen(source_pcd_pts_O, "base_link"))

        r = R.from_dcm(T_O_C[:3,:3])
        print("R_O_C: %s" % T_O_C[:3,:3])
        print("q_O_C: %s" % r.as_quat())
        br.sendTransform(T_O_C[:3,3],
                          r.as_quat(),
                          rospy.Time.now(),
                          "cam1",
                          "base_link")

        r = R.from_dcm(T_O_C2[:3,:3])
        br.sendTransform(T_O_C2[:3,3],
                          r.as_quat(),
                          rospy.Time.now(),
                          "cam2",
                          "base_link")

        r = R.from_dcm(T_O_A[:3,:3])
        br.sendTransform(T_O_A[:3,3],
                          r.as_quat(),
                          rospy.Time.now(),
                          "obj",
                          "base_link")


    print("Object pose in robot frame is:\n%s" % T_O_A)
