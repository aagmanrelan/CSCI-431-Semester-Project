import math
import os
import re
import time
import numpy as np
import open3d as o3d


def extract_number_from_filename(filename):
    match = re.search(r'\d+', filename)
    if match:
        return int(match.group())
    else:
        return -1


def downsample_pointCloud(pointCloud, v_size):
    new_cloud = pointCloud.voxel_down_sample(voxel_size=v_size)
    return new_cloud


def plane_segmentation_outlier_cloud(point_cloud):
    plane_model, inliers = point_cloud.segment_plane(distance_threshold=0.01,
                                                     ransac_n=3,
                                                     num_iterations=1000)

    outlier_cloud = point_cloud.select_by_index(inliers, invert=True)

    return outlier_cloud


def background_subtract(point_cloud_lists):
    # visualizer = o3d.visualization.Visualizer()

    # visualizer.create_window('PointClouds')
    first = True

    point_cloud_sequence = []

    for idx in range(len(point_cloud_lists)):

        prev_frame = plane_segmentation_outlier_cloud(point_cloud_lists[0])

        # Current point cloud in
        temp = point_cloud_lists[idx]
        temp = plane_segmentation_outlier_cloud(temp)

        dist = np.asarray(temp.compute_point_cloud_distance(prev_frame))

        background = np.where(dist < 0.05)[0]

        if first:
            current_frame = temp.select_by_index(background, invert=True)
            # visualizer.add_geometry(current_frame)
            point_cloud_sequence.append(current_frame)
            first = False
        else:

            # current_frame.points = temp.select_by_index(background, invert=True).points
            current_frame = temp.select_by_index(background, invert=True)
            point_cloud_sequence.append(current_frame)
            # visualizer.update_geometry(current_frame)
            # visualizer.poll_events()
            # visualizer.update_renderer()

    return point_cloud_sequence


def get_vehicle_clusters(point_cloud):  # 1.85, 3
    labels = np.array(point_cloud.cluster_dbscan(eps=1.81, min_points=2))
    cluster_points_dict = {label: [] for label in np.unique(labels)}
    for point, label in zip(np.asarray(point_cloud.points), labels):
        cluster_points_dict[label].append(point)
    return cluster_points_dict


def calculate_centroid(point_cloud_cluster):
    return np.mean(np.asarray(point_cloud_cluster), axis=0)


def calculate_axis_aligned_bounding_box(point_cloud_cluster):
    cluster_points_vector = o3d.utility.Vector3dVector(point_cloud_cluster)
    bounding_box = o3d.geometry.AxisAlignedBoundingBox.create_from_points(cluster_points_vector)
    return bounding_box


def calculate_box_center(bounding_box):
    return bounding_box.get_center()


'''
vehicle_id : {
    data : {
        Bounding_box,
        prev_centroid: tuple, 
        mov_seq : list  
    }
}
'''


def calculate_motion_vector(prev_position, current_position):
    return np.array(current_position) - np.array(prev_position)


def write_results_to_file(frame_number, vehicle_track):
    f = open(f"perception_results/frame_{frame_number}.csv", "w")
    f.write("vehicle_id,position_x,position_y,position_z,mvec_x,mvec_y,mvec_z,bbox_x_min,bbox_x_max,bbox_y_min,"
            "bbox_y_max,bbox_z_min,bbox_z_max\n")
    for key in vehicle_track.keys():
        data = vehicle_track[key]
        f.write(str(key) + ",")
        center_x = data['prev_centroid'][0]
        center_y = data['prev_centroid'][1]
        center_z = data['prev_centroid'][2]
        mvx = data['motion_vectors'][0]
        mvy = data['motion_vectors'][1]
        mvz = data['motion_vectors'][2]
        bounding_box = data['Bounding_Box']
        BBox_X_Min = bounding_box.get_min_bound()[0]
        BBox_Y_Min = bounding_box.get_min_bound()[1]
        BBox_Z_Min = bounding_box.get_min_bound()[2]

        BBox_X_Max = bounding_box.get_max_bound()[0]
        BBox_Y_Max = bounding_box.get_max_bound()[1]
        BBox_Z_Max = bounding_box.get_max_bound()[2]

        f.write(str(center_x) + "," + str(center_y) + "," + str(center_z) + "," + str(mvx) + "," +
                str(mvy) + "," + str(
            mvz) + "," + str(BBox_X_Min) + "," + str(BBox_X_Max) + "," + str(BBox_Y_Min) + "," + str(
            BBox_Y_Max) + "," + str(BBox_Z_Min) + "," +
                str(BBox_Z_Max))
        f.write("\n")
    f.close()


# This is where the point clouds need to be visualized
def process_point_clouds(point_cloud_sequence):
    vehicle_tracks = {}  # Dictionary to keep track of vehicles across frames

    cluster_colors = {}

    visualizer = o3d.visualization.Visualizer()

    visualizer.create_window('PointClouds')

    for i, point_cloud in enumerate(point_cloud_sequence):
        vehicle_clusters = get_vehicle_clusters(point_cloud)
        geometries = []
        boxes = []
        for cluster, vehicle in vehicle_clusters.items():
            if cluster != -1 and cluster < 6:
                # vehicle_id = map_cluster_to_vehicle(vehicle, vehicle_tracks)  # Implement this function
                if cluster not in cluster_colors:
                    cluster_colors[cluster] = np.random.rand(3)

                color = cluster_colors[cluster]

                cluster_points = o3d.geometry.PointCloud()
                cluster_points.points = o3d.utility.Vector3dVector(vehicle)
                cluster_points.paint_uniform_color(color)
                geometries.append(cluster_points)

                vehicle_id = cluster
                bounding_box = calculate_axis_aligned_bounding_box(vehicle)
                boxes.append(bounding_box)
                centroid = calculate_box_center(bounding_box)

                if vehicle_id in vehicle_tracks.keys():
                    prev_centroid = vehicle_tracks[vehicle_id]['prev_centroid']
                    motion_vector = calculate_motion_vector(prev_centroid, centroid)
                    vehicle_tracks[vehicle_id]['motion_vectors'] = motion_vector
                    vehicle_tracks[vehicle_id]['Bounding_Box'] = bounding_box
                else:
                    vehicle_tracks[vehicle_id] = {'Bounding_Box': bounding_box, 'prev_centroid': centroid,
                                                  'motion_vectors': np.zeros(3)}

                vehicle_tracks[vehicle_id]['prev_centroid'] = centroid

        visualizer.clear_geometries()

        for g in geometries:
            visualizer.add_geometry(g)

        for b in boxes:
            lines = o3d.geometry.LineSet.create_from_axis_aligned_bounding_box(b)
            lines.paint_uniform_color([0, 0, 0])  # Set color to black
            visualizer.add_geometry(lines)

        visualizer.poll_events()
        visualizer.update_renderer()
        # time.sleep(0.01)

        write_results_to_file(i, vehicle_tracks)

    return vehicle_tracks


def main():
    filelist = []
    point_cloud_lists = []

    for file in os.listdir('dataset/PointClouds'):
        f = os.path.join("dataset", "PointClouds", file)
        filelist.append(f)

    filelist.sort(key=extract_number_from_filename)

    for file in filelist:
        pointcloud = o3d.io.read_point_cloud(file)

        pointcloud = downsample_pointCloud(pointcloud, 0.15)
        point_cloud_lists.append(pointcloud)

    point_cloud_list = background_subtract(point_cloud_lists)
    process_point_clouds(point_cloud_list)


def visualize_pointCloud(pointClouds):
    o3d.visualization.draw_geometries(pointClouds,
                                      zoom=0.1,
                                      front=[0, 0, 1],
                                      lookat=[0, 0, 0],
                                      up=[0, 1, 0]
                                      )


def extract_number_from_filename(filename):
    match = re.search(r'\d+', filename)
    if match:
        return int(match.group())
    else:
        return -1


if __name__ == '__main__':
    main()
