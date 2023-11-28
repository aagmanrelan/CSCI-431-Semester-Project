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


def background_subtract(point_cloud_lists):
    visualizer = o3d.visualization.Visualizer()

    visualizer.create_window('PointClouds')
    first = True

    for idx in range(len(point_cloud_lists)):

        # Initial Frame
        prev_frame = point_cloud_lists[25]

        # Mean Frame 
        # prev_frame = np.sum(point_cloud_lists[idx - 10: idx])/10

        temp = point_cloud_lists[idx]

        dist = np.asarray(temp.compute_point_cloud_distance(prev_frame))
        background = np.where(dist < 0.05)[0]

        if first:
            current_frame = temp.select_by_index(background, invert=True)
            visualizer.add_geometry(current_frame)
            point_cloud_lists.append(current_frame)
            first = False
        else:
            current_frame.points = temp.select_by_index(background, invert=True).points
            point_cloud_lists.append(current_frame)
            visualizer.update_geometry(current_frame)
            visualizer.poll_events()
            visualizer.update_renderer()

    return

    print(len(point_cloud_lists))

    # count = 0

    # for point_cloud in point_cloud_lists:
    # visualizer.clear_geometries()
    #     if count == 0:
    #         print('j')
    #         visualizer.add_geometry(point_cloud)
    #         count = 1
    #     else:
    #         print('i')
    #         visualizer.update_geometry(point_cloud)
    #         visualizer.poll_events()
    #         visualizer.update_renderer()

    # visualizer.destroy_window()


def main():
    filelist = []
    point_cloud_lists = []

    for file in os.listdir('dataset/PointClouds'):
        f = os.path.join("dataset", "PointClouds", file)
        filelist.append(f)

    filelist.sort(key=extract_number_from_filename)

    for file in filelist:
        pointcloud = o3d.io.read_point_cloud(file)
        # Let's exclude outlier removal as it is introducing noise. 
        # pointcloud = outlier_removal(pointcloud, 10, 1)

        pointcloud = downsample_pointCloud(pointcloud, 0.10)
        point_cloud_lists.append(pointcloud)

    background_subtract(point_cloud_lists)


def extract_number_from_filename(filename):
    match = re.search(r'\d+', filename)
    if match:
        return int(match.group())
    else:
        return -1


if __name__ == '__main__':
    main()

############### ROUGH WORK #############

'''
def background_subtract():

    point_clouds_path = './dataset/PointClouds/'

    visualizer = o3d.visualization.Visualizer()

    visualizer.create_window('PointClouds')

    files = os.listdir(point_clouds_path)

    files.sort(key=extract_number_from_filename)

    first = True

    point_cloud_lists = []


    for idx in range(50, 100):
        prev_frame = o3d.io.read_point_cloud(point_clouds_path + files[idx - 1])
        temp = o3d.io.read_point_cloud(point_clouds_path + files[idx])
        dist = np.asarray(temp.compute_point_cloud_distance(prev_frame))
        background = np.where(dist < 0.01)[0]
        
        if first:
            current_frame = temp.select_by_index(background, invert=True)
            visualizer.add_geometry(current_frame)
            point_cloud_lists.append(current_frame)
            first = False
        else:
            current_frame.points = temp.select_by_index(background, invert=True).points
            point_cloud_lists.append(current_frame)
            visualizer.update_geometry(current_frame)
            visualizer.poll_events()
            visualizer.update_renderer()



    print(len(point_cloud_lists))

    count = 0

'''
