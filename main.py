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


def visualize_pointCloud(pointClouds):
    o3d.visualization.draw_geometries(pointClouds,
                                      zoom=0.1,
                                      front=[0, 0, 1],
                                      lookat=[0, 0, 0],
                                      up=[0, 1, 0]
                                      )


def downsample_pointCloud(pointCloud, v_size):
    new_cloud = pointCloud.voxel_down_sample(voxel_size=v_size)
    return new_cloud


def outlier_removal(pointCloud, neighbors, st_distance):
    """
    neighbors, which specifies how many neighbors are taken into account in order to calculate the average distance
    for a given point.
    st_distance, which allows setting the threshold level based on the standard deviation of the average distances across
    the point cloud. The lower this number the more aggressive the filter will be.
    """
    new_cloud, _ = pointCloud.remove_statistical_outlier(nb_neighbors=neighbors, std_ratio=st_distance)
    return new_cloud


def plane_segmentation(pointCloud, dist_threshold, n_points, interactions):
    plane_model, inliers = pointCloud.segment_plane(distance_threshold=dist_threshold,
                                                    ransac_n=n_points,
                                                    num_iterations=interactions)

    return plane_model, inliers


def get_box_volume(box):
    a = box.extent
    a = a * 2
    print(np.prod(a))


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

    cluster_info_list = []

    for obj in objectpointcloudlist:
        labels = np.array(obj.cluster_dbscan(eps=1.20, min_points=6))
        cluster_points_dict = {label: [] for label in np.unique(labels)}
        for point, label in zip(np.asarray(obj.points), labels):
            cluster_points_dict[label].append(point)
        cluster_info_list.append(cluster_points_dict)

    working_dict = cluster_info_list[399]

    for key in working_dict:
        points = working_dict[key]
        if 500 > len(points) > 8:
            vector_p = o3d.utility.Vector3dVector(points)
            box = o3d.geometry.OrientedBoundingBox.create_from_points(vector_p)
            get_box_volume(box)
            point_cloud = o3d.geometry.PointCloud()
            point_cloud.points = vector_p
            visualize_pointCloud([point_cloud])




###########################################################
###########################################################
###########################################################
###########################################################
###########################################################
###########################################################

    # get corners of these boxes
    # Get volume
    # Compare by Volume



    # visualize_pointCloud([objectpointcloudlist[0]])

    # pcd = objectpointcloudlist[399]
    # with o3d.utility.VerbosityContextManager(
    #         o3d.utility.VerbosityLevel.Debug) as cm:
    #     labels = np.array(
    #         pcd.cluster_dbscan(eps=1.20, min_points=6, print_progress=True))
    #
    # max_label = labels.max()
    # print(f"point cloud has {max_label + 1} clusters")
    # colors = plt.get_cmap("tab20")(labels / (max_label if max_label > 0 else 1))
    # colors[labels < 0] = 0
    # pcd.colors = o3d.utility.Vector3dVector(colors[:, :3])
    # o3d.visualization.draw_geometries([pcd],
    #                                   zoom=0.455,
    #                                   front=[-0.4999, -0.1659, -0.8499],
    #                                   lookat=[2.1813, 2.0619, 2.0999],
    #                                   up=[0.1204, -0.9852, 0.1215])






















    # for key in cluster_info_list[350].keys():
    #     print(len(cluster_info_list[350][key]))
    #
    #
    # cluster_points = cluster_info_list[350][2]
    # vector_p = o3d.utility.Vector3dVector(cluster_points)
    # box = o3d.geometry.OrientedBoundingBox.create_from_points(vector_p)
    # print(np.asarray(box.get_box_points()))
    #
    # pc = o3d.geometry.PointCloud()
    # print(vector_p)
    # pc.points = o3d.utility.Vector3dVector(box.get_box_points())
    # pc1 = o3d.geometry.PointCloud()
    # pc1.points = vector_p
    # visualize_pointCloud([pc, pc1])

    # object = cluster_info_list[350]['object']
    # labels = cluster_info_list[350]['labels']
    #
    # unique_labels, counts = np.unique(labels, return_counts=True)
    # max_label = unique_labels.max()

    # print(unique_labels)
    # print(counts)

    # points_array = np.asarray(object.points)

    # for label, count in zip(unique_labels, counts):
    #     if label >= 0:
    #         print(f"Cluster {label} in object has {count} points")

    # max_label = labels.max()
    # # print(f"point cloud has {max_label + 1} clusters")
    # colors = plt.get_cmap("tab20")(labels / (max_label if max_label > 0 else 1))
    # colors[labels < 0] = 0
    # object.colors = o3d.utility.Vector3dVector(colors[:, :3])

    # voxel_grid = o3d.geometry.VoxelGrid.create_from_point_cloud(pointcloudlist[0],
    #                                                             voxel_size=1)
    # # o3d.visualization.draw_geometries([voxel_grid])
    #
    # voxels = np.asarray(voxel_grid.get_voxels())
    #
    # for v in voxels:
    #     print(v)

    # vis = o3d.visualization.Visualizer()
    # vis.create_window()
    #
    # for point_cloud in objectpointcloudlist:
    #     vis.clear_geometries()
    #     vis.add_geometry(point_cloud)
    #     # vis.update_renderer()
    #     vis.poll_events()
    #
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

#

#

#


if __name__ == "__main__":
    main()
