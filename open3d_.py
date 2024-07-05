

#Read PC data
import open3d as o3d
import os, glob
import numpy as np
import open3d.core as o3c
import matplotlib.pyplot as plt
filename = os.getcwd()+"\\data\\road.pcd"
road = o3d.t.io.read_point_cloud(filename)
#road = o3d.io.read_point_cloud(filename)


downroad = road.voxel_down_sample(voxel_size=0.5)
plane_model, inliers = downroad.segment_plane(distance_threshold=10, ransac_n=1000, num_iterations=1000)
inlier_cloud = downroad.select_by_index(inliers)
o3d.visualization.draw_geometries([inlier_cloud])

#Filter point cloud
points = np.asarray(inlier_cloud.points)
filtered = np.where(points[:,2] > np.mean(points[:,2]))
filtered_points = points[filtered] 
filtered_pcd = o3d.geometry.PointCloud()
filtered_pcd.points = o3d.utility.Vector3dVector(filtered_points)
o3d.visualization.draw_geometries([filtered_pcd])

#Remove outliers
print("Statistical oulier removal")
def display_inlier_outlier(cloud, ind):
    inlier_cloud = cloud.select_by_index(ind)
    outlier_cloud = cloud.select_by_index(ind, invert=True)
    print("Showing inliers (gray): ")
    inlier_cloud.paint_uniform_color([0.8, 0.8, 0.8])
    o3d.visualization.draw_geometries([inlier_cloud])
									  


cl, ind = filtered_pcd.remove_statistical_outlier(nb_neighbors=2000, std_ratio=20)
ind_cloud = filtered_pcd.select_by_index(ind)
o3d.visualization.draw_geometries([ind_cloud])
o3d.io.write_point_cloud(os.getcwd()+"\\data\\road_outlier_removed.pcd", ind_cloud)


o3d.io.write_point_cloud("/Users/ying/Documents/MATLAB/inlier.pcd", inlier_cloud)

#Get inliers coordinates
inlier_coord = np.asarray(inlier_cloud.points)
outlier_cloud = downroad.select_by_index(inliers, invert=True)
o3d.visualization.draw_geometries([outlier_cloud.to_legacy()])


#boundary detection - radius (neighbour search radius), max_nn (default 20, max number of neighbour to search), angle_threshold (default 90) to decide if a point is on the boudnary.
ind_cloud = o3d.t.io.read_point_cloud(os.getcwd()+"\\data\\road_outlier_removed.pcd")
ind_cloud.estimate_normals()
boundarys, mask = ind_cloud.compute_boundary_points(20, 400)
#print(f"Detect {boundarys.point.positions.shape[0]} bnoundary points from {inlier_cloud.point.positions.shape[0]} points.")

boundarys_ = boundarys.paint_uniform_color([1, 0.706, 0])
#inlier_cloud_ = ind_cloud.paint_uniform_color ([0, 0.3, 0])
o3d.visualization.draw_geometries([boundarys_.to_legacy()])


#dbscan 
import pandas as pd
file = pd.read_csv (os.getcwd()+"\\data\\road_edge_clean.csv", header = None)
xyz = file.to_numpy()
pcd = o3d.geometry.PointCloud()
pcd.points = o3d.utility.Vector3dVector(xyz)
o3d.io.write_point_cloud(os.getcwd()+"\\data\\road_edge_clean.pcd", pcd)


boundarys_ = o3d.t.io.read_point_cloud(os.getcwd()+"\\data\\road_edge_clean.pcd")

def tune_param (pcd, eps, min_points):
    with o3d.utility.VerbosityContextManager(o3d.utility.VerbosityLevel.Debug) as cm:
        labels = pcd.cluster_dbscan(eps, min_points, print_progress=True)
    max_label = labels.max().item()
    print(f"point cloud has {max_label + 1} clusters")
    colors = plt.get_cmap("tab20")(labels.numpy() / (max_label if max_label > 0 else 1))
    colors = o3c.Tensor(colors[:, :3], o3c.float32)
    colors[labels < 0] = 0
    pcd.point.colors = colors
    return pcd, labels

bound, label = tune_param (boundarys_, 20, 10)
o3d.visualization.draw_geometries([bound.to_legacy()])

def ransac (pcd_data, distance_threshold, ransac_n):
    plane_model, inliers = pcd_data.segment_plane(distance_threshold=distance_threshold, ransac_n=ransac_n, num_iterations=1000)
    inlier_cloud = pcd_data.select_by_index(inliers)
    return inlier_cloud

inlier_cloud = ransac (boundarys_, 6, 100)
o3d.visualization.draw_geometries([inlier_cloud.to_legacy()])