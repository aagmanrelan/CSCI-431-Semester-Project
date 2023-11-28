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

    
def background_subtract(point_cloud_lists):

    # visualizer = o3d.visualization.Visualizer()

    # visualizer.create_window('PointClouds')
    first = True

    point_cloud_sequence = []

    for idx in range(len(point_cloud_lists)):

        # Initial Frame
        prev_frame = point_cloud_lists[25]

        # Mean Frame 
        # prev_frame = np.sum(point_cloud_lists[idx - 10: idx])/10

        # Current point cloud in 
        temp = point_cloud_lists[idx]

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

    # This ideally should contain only the points belonging to all cars

        # if idx ==499:
        #     break
    
    # pcd = point_cloud_sequence[499]

    # with o3d.utility.VerbosityContextManager(
    #                     o3d.utility.VerbosityLevel.Debug) as cm:
    #                 labels = np.array(
    #                     pcd.cluster_dbscan(eps=0.9, min_points=4, print_progress=True))

    # max_label = labels.max()
    # print(f"point cloud has {max_label + 1} clusters")
    # colors = plt.get_cmap("tab20")(labels / (max_label if max_label > 0 else 1))
    # colors[labels < 0] = 0
    # pcd.colors = o3d.utility.Vector3dVector(colors[:, :3])
    # o3d.visualization.draw_geometries([pcd],
    #                                             zoom=0.455,
    #                                             front=[-0.4999, -0.1659, -0.8499],
    #                                             lookat=[2.1813, 2.0619, 2.0999],
    #                                             up=[0.1204, -0.9852, 0.1215])
    return point_cloud_sequence

def get_vehicle_clusters(point_cloud):
        # eps =1.25
        #min_points =6

        labels = np.array(point_cloud.cluster_dbscan(eps=0.9, min_points=4))
        cluster_points_dict = {label: [] for label in np.unique(labels)}
        for point, label in zip(np.asarray(point_cloud.points), labels):
            cluster_points_dict[label].append(point)
        
        return cluster_points_dict

def calculate_centroid(point_cloud):
    return np.mean(np.asarray(point_cloud), axis=0)


'''
vehicle_id : {
    data : {
        prev_centroid: tuple, 
        mov_seq : list  
    }
}
'''
def identify_vehicle(current_vehicle, vehicle_tracks, distance_threshold=1.0):
    current_centroid = calculate_centroid(current_vehicle)

    for vehicle_id, data in vehicle_tracks.items():
        prev_centroid = data['prev_centroid']
        distance = np.linalg.norm(np.array(prev_centroid) - np.array(current_centroid))
        if distance < distance_threshold:
            return vehicle_id

    # If no match found, assign a new ID
    return max(vehicle_tracks.keys(), default=0) + 1

def calculate_motion_vector(prev_position, current_position):
    return np.array(current_position) - np.array(prev_position)

def process_point_clouds(point_cloud_sequence):
    vehicle_tracks = {}  # Dictionary to keep track of vehicles across frames
    count = 100

    for i, point_cloud in enumerate(point_cloud_sequence):
        # Assume get_vehicle_clusters returns a list of vehicle clusters
        vehicle_clusters = get_vehicle_clusters(point_cloud)

        # if count == 100:
        #     for key in vehicle_clusters.keys():
        #         print(str(key), len(vehicle_clusters[key]))
        #     break
        if i < 101:
            for cluster, vehicle in vehicle_clusters.items():
                if cluster != -1:
                    vehicle_id = identify_vehicle(vehicle, vehicle_tracks)  # Implement this function
                    centroid = calculate_centroid(vehicle)

                    
                    if vehicle_id in vehicle_tracks.keys():
                        prev_centroid = vehicle_tracks[vehicle_id]['prev_centroid']
                        motion_vector = calculate_motion_vector(prev_centroid, centroid)
                        vehicle_tracks[vehicle_id]['motion_vectors'].append(motion_vector)
                    else:
                        vehicle_tracks[vehicle_id] = {'prev_centroid': centroid, 'motion_vectors': []}

                    # update prev_centroid for reference 
                    vehicle_tracks[vehicle_id]['prev_centroid'] = centroid
        # Optionally, you can remove tracks that haven't been updated in a while
        print(vehicle_tracks)

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


        pointcloud = downsample_pointCloud(pointcloud, 0.10)
        point_cloud_lists.append(pointcloud)
        # here we have all the differences of point clouds
        
    point_cloud_list = background_subtract(point_cloud_lists)

    process_point_clouds(point_cloud_list)




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




# def process_point_clouds(point_cloud_sequence):
#     vehicle_tracks = {}  # Dictionary to keep track of vehicles across frames

#     for i, point_cloud in enumerate(point_cloud_sequence):
#         # Assume get_vehicle_clusters returns a list of vehicle clusters
#         vehicle_clusters = get_vehicle_clusters(point_cloud)

#         for vehicle in vehicle_clusters:
#             vehicle_id = identify_vehicle(vehicle, vehicle_tracks)  # Implement this function
#             centroid = calculate_centroid(vehicle)

#             if vehicle_id in vehicle_tracks:
#                 prev_centroid = vehicle_tracks[vehicle_id]['prev_centroid']
#                 motion_vector = calculate_motion_vector(prev_centroid, centroid)
#                 vehicle_tracks[vehicle_id]['motion_vectors'].append(motion_vector)
#             else:
#                 vehicle_tracks[vehicle_id] = {'prev_centroid': centroid, 'motion_vectors': []}

#             # update prev_centroid for reference 
#             vehicle_tracks[vehicle_id]['prev_centroid'] = centroid

#         # Optionally, you can remove tracks that haven't been updated in a while

#     return vehicle_tracks


            # pcd = new_point_cloud
            # with o3d.utility.VerbosityContextManager(
            #         o3d.utility.VerbosityLevel.Debug) as cm:
            #     labels = np.array(
            #         pcd.cluster_dbscan(eps=1.25, min_points=6, print_progress=True))

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
            # break



#   def make_clusters(pointcloud):
#     labels = np.array(pointcloud.cluster_dbscan(eps=1.25, min_points=6))
#     cluster_points_dict = {label: [] for label in np.unique(labels)}
#     for point, label in zip(np.asarray(pointcloud.points), labels):
#         cluster_points_dict[label].append(point)
#     return cluster_points_dict


# def get_viable_cars(cluster_dict):
#     bounding_box_list = []
#     for key in cluster_dict.keys():
#         if key != -1:
#             cluster_points_vector = o3d.utility.Vector3dVector(cluster_dict[key])
#             bounding_box = o3d.geometry.OrientedBoundingBox.create_from_points(cluster_points_vector)
#             bounding_box_list.append(bounding_box)

#     print(bounding_box_list)

'''