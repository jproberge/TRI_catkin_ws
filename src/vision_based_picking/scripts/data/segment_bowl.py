import numpy as np
import open3d as o3d
from sklearn.cluster import KMeans

BOWL_COLOR = np.array([0.025, 0.16, 0.75])


# We assume the following:
# 1. Only the bowl is in the scene.
# 2. We know the bowl's color and its somewhat uniform

def get_bowl_pcd(filepath):
    pcd = o3d.io.read_point_cloud(filepath)

    pcd_filtered = filter_pc(pcd, 0.003, [15, 0.01], [30, 0.015])

    # Filter for the object points
    # get the point cloud colors as numpy array
    pcd_colors = np.asarray(pcd_filtered.colors)
    # track point IDs
    pcd_ids = np.arange(pcd_colors.shape[0])
    # color wanted is (0.025, 0.16, 0.75) which is the tiel bowl so use color distance. 
    # WARNIG this is not illumination or shadow robust.
    pcd_color_dist = np.linalg.norm(np.subtract(pcd_colors, BOWL_COLOR), axis=1)
    pcd_color_dist_sorted = np.argsort(pcd_color_dist)
    # get mask of colors that are close enough
    pcd_color_near_mask = pcd_color_dist < 0.38
    # get the id of the masked points
    pcd_color_near_id = np.argwhere(pcd_color_near_mask)

    # filter the original point cloud to get points near color chosen
    pcd_points_filtered = np.asarray(pcd_filtered.points)[pcd_color_near_mask]
    pcd_color_filtered = np.asarray(pcd_filtered.colors)[pcd_color_near_mask]

    # get the centroid of the points
    pcd_centroid = np.mean(pcd_points_filtered, axis=0)
    print(pcd_centroid)

    pcd_region = pcd_spatial_crop(pcd_filtered, pcd_centroid, 0.3)

    n_cluster = 6
    centers, labels = kmeans_color(pcd_region, n_cluster)
    color = np.random.rand(n_cluster,3)
    kmeans_pcd = o3d.geometry.PointCloud()
    kmeans_pcd.colors = o3d.utility.Vector3dVector(color[labels])
    kmeans_pcd.points = pcd_region.points

    # find the cluster closest to the color we want (id the bowl)
    color_centers = centers[:,3:]
    pcd_color_dist = np.linalg.norm(np.subtract(color_centers, BOWL_COLOR), axis=1)
    # sort distances in ascending order
    pcd_color_dist_sorted = np.argsort(pcd_color_dist)
    closest_cluster = pcd_color_dist_sorted[0]
    in_closest_cluster = (labels == closest_cluster)
    final_pcd = mask_pcd(pcd_region, in_closest_cluster)

    # create new point cloud
    o3d.io.write_point_cloud("data/segmented_bowl.pcd", final_pcd)
    #o3d.visualization.draw_geometries([final_pcd])

    return final_pcd



def mask_pcd(pcd, mask):
    """
    Helper function that takes in a points cloud and filters points 
    based on a mask
    Input:  
            pcd: open3d PointCloud object
            mask: boolean array size of number of points
    Output:
            new_pcd: open3d PointCloud object filtered
    """
    pcd_pts_arr = np.asarray(pcd.points)
    pcd_colors_arr = np.asarray(pcd.colors)
    new_pcd = o3d.geometry.PointCloud()
    new_pcd.colors = o3d.utility.Vector3dVector(pcd_colors_arr[mask])
    new_pcd.points = o3d.utility.Vector3dVector(pcd_pts_arr[mask])
    return new_pcd


def pcd_spatial_crop(pcd, centroid, box_width):
    """
    Simple function that takes in a points cloud and filters points 
    that are within a specified distance from the camera in the z axis (optical axis)
    Input:  
            pcd: open3d PointCloud object
            centroid: 3d vector of center of object
            box_width: width of the clipping
    Output:
            new_pcd: open3d PointCloud object filtered
    """
    pcd_pts_arr = np.asarray(pcd.points)
    pcd_colors_arr = np.asarray(pcd.colors)
    dist_mask = np.bitwise_and(pcd_pts_arr[:,2] > (centroid[2] - box_width/2.0), \
                              pcd_pts_arr[:,2] < (centroid[2] + box_width/2.0))
    dist_mask = np.bitwise_and(dist_mask, pcd_pts_arr[:,1] > (centroid[1] - box_width/2.0))
    dist_mask = np.bitwise_and(dist_mask, pcd_pts_arr[:,1] < (centroid[1] + box_width/2.0))
    dist_mask = np.bitwise_and(dist_mask, pcd_pts_arr[:,0] > (centroid[0] - box_width/2.0))
    dist_mask = np.bitwise_and(dist_mask, pcd_pts_arr[:,0] < (centroid[0] + box_width/2.0))
    return mask_pcd(pcd, dist_mask)
    
def kmeans_color(pcd, n_clusters):
    """
    Function takes in a points cloud and finds the kmeans of these points based on color
    and position. Positions are multiplied by a factor to penalize physical distance more.
    Input:  
            pcd: open3d PointCloud object
            n_clusters: how many clusters to get from kmeans
    Output:
            cluster centers: n centroids returned by kmeans
            labels: label of each data point by kmeans
    """
    pcd_pts = np.asarray(pcd.points)
    pcd_colors = np.asarray(pcd.colors)
    kmeans = KMeans(n_clusters=n_clusters)
    kmeans.fit(np.hstack([pcd_pts*5.0,pcd_colors]))
    return kmeans.cluster_centers_, kmeans.labels_


def filter_pc(cl, voxel_size, sor_params=None, ror_params=None):
    # sor_params = [nb_neighbors, std_ratio]
    # ror_params = [nb_neighbors, radius]
    cl = cl.voxel_down_sample(voxel_size)
    if sor_params != None:
        cl, ind = cl.remove_statistical_outlier(sor_params[0], sor_params[1])
    if ror_params != None:
        cl, ind = cl.remove_radius_outlier(ror_params[0], ror_params[1])
    return cl


if __name__ == "__main__":
    get_bowl_pcd("Whole_Scene_Camera1_Bowl.pcd")
