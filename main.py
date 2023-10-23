import os
import re
import time
import numpy as np
import open3d as o3d
from matplotlib import pyplot as plt


def extract_number_from_filename(filename):
    match = re.search(r'\d+', filename)
    if match:
        return int(match.group())
    else:
        return -1


def visualize_pointCloud(pointCloud):
    o3d.visualization.draw_geometries([pointCloud],
                                      zoom=0.1,
                                      front=[0, 0, 1],
                                      lookat=[0, 0, 0],
                                      up=[0, 1, 0]
                                      )


def downsample_pointCloud(pointCloud, v_size):
    new_cloud = pointCloud.voxel_down_sample(voxel_size=v_size)
    return new_cloud


def outlier_removal(pointCloud, neighbors, st_distance):
    new_cloud, _ = pointCloud.remove_statistical_outlier(nb_neighbors=neighbors, std_ratio=st_distance)
    return new_cloud


def plane_segmentation(pointCloud, dist_threshold, n_points, interactions):
    plane_model, inliers = pointCloud.segment_plane(distance_threshold=dist_threshold,
                                                    ransac_n=n_points,
                                                    num_iterations=interactions)

    return plane_model, inliers


def main():
    filelist = []
    for file in os.listdir('dataset/PointClouds'):
        f = os.path.join("dataset", "PointClouds", file)
        filelist.append(f)

    filelist.sort(key=extract_number_from_filename)

    objectpointcloudlist = []
    pointcloudlist = []
    for file in filelist:
        pointcloud = o3d.io.read_point_cloud(file)
        pointcloud = outlier_removal(pointcloud, 10, 1)
        pointcloud = downsample_pointCloud(pointcloud, 0.15)
        pointcloudlist.append(pointcloud)
        plane_model, inliers = plane_segmentation(pointcloud, 0.009, 3, 25000)
        objects = pointcloud.select_by_index(inliers, invert=True)
        objectpointcloudlist.append(objects)

    # vis = o3d.visualization.Visualizer()
    # vis.create_window()
    #
    # for point_cloud in objectpointcloudlist:
    #     vis.clear_geometries()
    #     vis.add_geometry(point_cloud)
    #     vis.update_renderer()
    #     vis.poll_events()

    # Close the Visualizer window when done
    # vis.destroy_window()


# pointcloud = o3d.io.read_point_cloud("dataset/PointClouds/400.pcd")
# pointcloud = outlier_removal(pointcloud, 10, 1.0)
# pointcloud = downsample_pointCloud(pointcloud, 0.15)
#
# plane_model, inliers = plane_segmentation(pointcloud, 0.009, 3, 25000)
#
# objects = pointcloud.select_by_index(inliers, invert=True)
#
# # o3d.visualization.draw_geometries([objects])
#
#
# labels = np.array(
#     objects.cluster_dbscan(eps=1.5, min_points=15, print_progress=True))
#
# max_label = labels.max()
# print(f"point cloud has {max_label + 1} clusters")
# colors = plt.get_cmap("tab20")(labels / (max_label if max_label > 0 else 1))
# colors[labels < 0] = 0
# objects.colors = o3d.utility.Vector3dVector(colors[:, :3])
#
# o3d.visualization.draw_geometries([objects],
#                                   zoom=0.1,
#                                   front=[0, 0, 1],
#                                   lookat=[0, 0, 0],
#                                   up=[0, 1, 0]
#                                   )


#

#


if __name__ == "__main__":
    main()
