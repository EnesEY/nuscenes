from nuscenes.nuscenes import NuScenes
from pymongo import MongoClient 
from nuscenes.map_expansion.map_api import NuScenesMap
from nuscenes.map_expansion import arcline_path_utils
from multiprocessing import Process
from tqdm import tqdm

import os
import time
import numpy as np
import math
import sample_annotation_parameters as parameters
import sample_annotation_utils as utils
import visualize

class LoadImplementer:

    def __init__(self):
        self.nusc = NuScenes(version='v1.0-mini', dataroot='/data/sets/nuscenes', verbose=True)
        self.client=MongoClient('mongodb+srv://enesey:485f6483e3c8666b72fda603a7f87006b83549a54395f2504eb58935f35d00d9@nuscenescluster.jh1vw.mongodb.net/test')
        self.db = self.client.nuscenes

        self.nusc_map_singapore_onenorth = NuScenesMap(dataroot='/data/sets/nuscenes', map_name='singapore-onenorth')
        self.nusc_map_singapore_hollandvillage = NuScenesMap(dataroot='/data/sets/nuscenes', map_name='singapore-hollandvillage')
        self.nusc_map_singapore_queenstown = NuScenesMap(dataroot='/data/sets/nuscenes', map_name='singapore-queenstown')
        self.nusc_map_boston_seaport = NuScenesMap(dataroot='/data/sets/nuscenes', map_name='boston-seaport') 

        self._filter_unnecessary_instances()

        print('started loading annotaions')

        self.processes = []
        self.process_ranges = utils.split_processes(self.nusc.instance)

        thread_number = 0
        self._load_annotations(0, 1, thread_number)

        print('finished loading annotaions')



    # Filtering
    def _filter_unnecessary_instances(self):
        instances = []
        for instance in self.nusc.instance:
            if utils.get_vehicle_and_not_ego_vehicle(instance, self.nusc) == True:
                instances.append(instance)
        self.nusc.instance = instances



    # Load Annotations
    def _load_annotations(self, start_index, end_index, thread_number):
        self._load_instance_annotations(start_index, end_index, thread_number)
        self._load_map_annotations(start_index, end_index, thread_number)



    # instance annotations
    def _load_instance_annotations(self, start_index, end_index, thread_number):
        for index in range(start_index, end_index):
            sample_annotations = utils.get_sample_annotations_of_instance(self.nusc.instance[index], self.nusc)
            sample_annotations = [sample_annotations[0]]
            self._load_moving_state(sample_annotations)
            self._load_moved_before(sample_annotations)
            self._load_time_not_moved(sample_annotations)
            self._load_vehicle_volume(sample_annotations)


    def _load_moving_state(self, sample_annotations):
        for annotation in sample_annotations:
            velocities = list(self.nusc.box_velocity(annotation['token']))
            moving = False
            if abs(np.average(velocities)) > parameters.moving_treshold:
                moving = True
            annotation['movingState'] = moving


    def _load_moved_before(self, sample_annotations):
        movedBefore = False
        print(len(sample_annotations))
        for annotation in sample_annotations:
            if movedBefore == False and annotation['movingState'] == True:
                movedBefore = True
            annotation['movedBefore'] = movedBefore


    def _load_time_not_moved(self, sample_annotations):
        timeNotMoved = 0
        for annotation in sample_annotations:
            if annotation['movingState'] == True:
                timeNotMoved = 0
            if annotation['movingState'] == False:
                timeNotMoved += 1
            annotation['timeNotMoved'] = timeNotMoved


    def _load_vehicle_volume(self, sample_annotations):
        x,y,z = sample_annotations[0]['size']
        vehicle_volume = x * y * z
        for annotation in sample_annotations:
            annotation['vehicleVolume'] = vehicle_volume



    # map annotations
    def _load_map_annotations(self, start_index, end_index, thread_number):
        for index in tqdm(range(start_index, end_index), desc=f'map annotation progress of thread nr. {thread_number}:'):
            sample_annotations = utils.get_sample_annotations_of_instance(self.nusc.instance[index], self.nusc)
            sample_annotations = [sample_annotations[0]]
            nusc_map_name = utils.get_map_name_of_annotation(sample_annotations[0], self.nusc)
            nusc_map = self._get_map_of_annotation(nusc_map_name)
            for annotation in sample_annotations:
                road_segment = utils.get_road_segment_of_annotation(annotation, nusc_map)
                self._load_on_intersection(annotation, road_segment)
                # self._load_distance_to_boundaries(sample_annotations[i], nusc_map, road_segment, 50)
                # self._load_distance_to_next_stop_point(annotation, nusc_map)
                self._load_has_instance_in_front(annotation,nusc_map )

    def _load_on_intersection(self, annotation, road_segment):
        onIntersection = ''
        if road_segment != '':
            onIntersection = road_segment['is_intersection']
        annotation['onIntersection'] = onIntersection


    def _load_distance_to_boundaries(self, annotation, nusc_map, road_segment, sampling_rate):
        distance_to_left_boundary = ''
        distance_to_right_boundary = ''

        nusc_map.non_geometric_polygon_layers = ['drivable_area', 'road_segment', 'lane']
        nusc_map.non_geometric_line_layers = []
        nusc_map.non_geometric_layers = ['drivable_area', 'road_segment', 'lane']

        skip = False
        if road_segment == '' or road_segment['is_intersection'] == True:
            skip = True
        
        nearestLane = utils.get_closest_lane_of_annotaion(annotation, nusc_map)
        if nearestLane == '':
            skip = True

        if skip != True:
            x_center = annotation['translation'][0]
            y_center = annotation['translation'][1]

            # get direction of x and y of the nearest lane
            [delta_x_lane, delta_y_lane] = utils.get_delta_x_and_delta_y_of_lane(annotation, nearestLane)
            [orthogonal_delta_x_lane , orthogonal_delta_y_lane] = [delta_y_lane, (-1)*delta_x_lane]
            road_segment_nodes = utils.get_nodes_of_road_segment(road_segment, nusc_map)
            [same_direction_nodes, opposite_direction_nodes] = utils.get_opposite_and_same_direction_of_node_array(road_segment_nodes, delta_x_lane, delta_y_lane)
                    
            # Get average point of both lanes
            [average_x_same_direction, average_y_same_direction] = utils.get_average_point_of_node_array(same_direction_nodes)
            [average_x_opposite_direction, average_y_opposite_direction] = utils.get_average_point_of_node_array(opposite_direction_nodes)

            isLeftBoundaryValid = True
            if average_x_opposite_direction == 0 or average_y_opposite_direction == 0:
                isLeftBoundaryValid = False
                    
            isRightBoundaryValid = True
            if average_x_same_direction == 0 or average_y_same_direction == 0:
                isRightBoundaryValid = False

            if isLeftBoundaryValid == True:
                start_point_left_lane = [x_center, y_center]
                end_point_left_lane =  [x_center + ((-1) * orthogonal_delta_x_lane) , y_center + ((-1) * orthogonal_delta_y_lane)]
                lane_to_left_boundary = utils.interpolate(start_point_left_lane[0], start_point_left_lane[1], end_point_left_lane[0], end_point_left_lane[1], sampling_rate)
                closest_point_of_left_lane_to_opposite_average_point = utils.get_nearest_node_of_node_array_to_point(lane_to_left_boundary, average_x_opposite_direction, average_y_opposite_direction)
                distance_to_left_boundary = utils.get_distance_between_two_points(closest_point_of_left_lane_to_opposite_average_point['x'],closest_point_of_left_lane_to_opposite_average_point['y'],x_center, y_center)

            if isRightBoundaryValid == True:
                start_point_right_lane = [x_center, y_center] 
                end_point_right_lane = [x_center + orthogonal_delta_x_lane , y_center + orthogonal_delta_y_lane]
                lane_to_right_boundary = utils.interpolate(start_point_right_lane[0], start_point_right_lane[1], end_point_right_lane[0], end_point_right_lane[1], sampling_rate)
                closest_point_of_right_lane_to_same_average_point = utils.get_nearest_node_of_node_array_to_point(lane_to_right_boundary, average_x_same_direction, average_y_same_direction)
                distance_to_right_boundary = utils.get_distance_between_two_points(closest_point_of_right_lane_to_same_average_point['x'],closest_point_of_right_lane_to_same_average_point['y'],x_center, y_center)
        annotation['distanceToLeftBoundary'] = distance_to_left_boundary
        annotation['distanceToRightBoundary'] = distance_to_right_boundary


    # def _load_distance_to_next_stop_point(self, annotation, nusc_map):
    #     sample_token = self.nusc.sample[100]['token']

    #     print('only plotting static vehicles')

    #     my_sample_annotaions = []
    #     my_vehicle_sample_annotations = []
    #     my_pose_x = []
    #     my_pose_y = []
    #     for my_annotation in self.nusc.sample_annotation:
    #         if my_annotation['sample_token'] == sample_token:
    #             # insert movingState
    #             velocities = list(self.nusc.box_velocity(my_annotation['token']))
    #             moving = False
    #             if abs(np.average(velocities)) > parameters.moving_treshold:
    #                 moving = True
    #             my_annotation['movingState'] = moving

    #             # insert only not moving vehicles into plot
    #             if my_annotation['movingState'] == False:
    #                 my_sample_annotaions.append(my_annotation)
    #                 category_name = my_annotation['category_name']
    #                 category_name = category_name.split(".") 
    #                 x = my_annotation['translation'][0]
    #                 y = my_annotation['translation'][1]
    #                 if category_name[0] == 'vehicle':
    #                     my_pose_x.append(x)
    #                     my_pose_y.append(y)
    #                     my_vehicle_sample_annotations.append(my_annotation)


    #     annotation = my_sample_annotaions[4]
    #     x_center = annotation['translation'][0]
    #     y_center = annotation['translation'][1]
    #     zoom_out = 50
    #     my_patch = ((x_center - zoom_out), (y_center - zoom_out), (x_center + zoom_out), (y_center + zoom_out))
    #     map_name = utils.get_map_name_of_annotation(annotation, self.nusc)
    #     nusc_map =self._get_map_of_annotation(map_name)

    #     fig, ax = nusc_map.render_map_patch(my_patch, nusc_map.non_geometric_layers, figsize=(10, 10))
    #     ax.scatter(my_pose_x, my_pose_y, s=20, c='k', alpha=1.0, zorder=2)
    #     records_within_patch = nusc_map.get_records_in_patch(my_patch, nusc_map.non_geometric_layers, mode='within')


    def _load_has_instance_in_front(self, annotation, nusc_map):
        sampling_rate = 10

        my_sample_annotaions = []
        my_instance_pose_x = []
        my_instance_pose_y = []

        my_vehicle_sample_annotations = []
        my_vehicle_pose_x = []
        my_vehicle_pose_y = []

        my_vehicle_lanes = []
        my_lane_pose_x = []
        my_lane_pose_y = []

        vehicles_with_instances_in_front = []
        my_in_front_pose_x = []
        my_in_front_pose_y = []

        # load instances of this sample 
        sample_token = self.nusc.sample[100]['token']
        for my_annotation in self.nusc.sample_annotation:
            if my_annotation['sample_token'] == sample_token:
                my_sample_annotaions.append(my_annotation)
                my_instance_pose_x.append(my_annotation['translation'][0])
                my_instance_pose_y.append(my_annotation['translation'][1])

                category_name = my_annotation['category_name']
                category_name = category_name.split(".") 
                if category_name[0] == 'vehicle':
                    my_vehicle_sample_annotations.append(my_annotation)
                    my_vehicle_pose_x.append(my_annotation['translation'][0])
                    my_vehicle_pose_y.append(my_annotation['translation'][1])

        # load a specific annotation to get the map of this sample
        annotation = my_sample_annotaions[4]
        map_name = utils.get_map_name_of_annotation(annotation, self.nusc)
        nusc_map =self._get_map_of_annotation(map_name)

        # get direction of x and y of the nearest lane
        for vehicle in my_vehicle_sample_annotations:
            nearestLane = utils.get_closest_lane_of_annotaion(vehicle, nusc_map)
            print(nearestLane)
            if nearestLane == '':
                continue

            [delta_x_lane, delta_y_lane] = utils.get_delta_x_and_delta_y_of_lane(vehicle, nearestLane)

            start_x = vehicle['translation'][0]
            start_y = vehicle['translation'][1]
            end_x = (start_x + delta_x_lane) 
            end_y = (start_y + delta_y_lane)

            distance = utils.get_distance_between_two_points(start_x, start_y, end_x, end_y)

            end_x = start_x + ((delta_x_lane/distance) * 10)
            end_y = start_y + ((delta_y_lane/distance) * 10)


            current_lane = utils.interpolate(start_x, start_y, end_x, end_y, sampling_rate)
            for node in current_lane:
                my_lane_pose_x.append(node['x'])
                my_lane_pose_y.append(node['y'])
            my_vehicle_lanes.append(current_lane)
        
            for instance in my_sample_annotaions:
                current_x = instance['translation'][0]
                current_y = instance['translation'][1]

                for node in current_lane:
                    node_x = node['x']
                    node_y = node['y']
                    distance2 = utils.get_distance_between_two_points(node_x, node_y, current_x, current_y)
                    if abs(distance2) < 2 and instance['token'] != vehicle['token']:
                        vehicle['hasInstanceInFront'] = True
                        vehicles_with_instances_in_front.append(vehicle)
                        my_in_front_pose_x.append(start_x)
                        my_in_front_pose_y.append(start_y)

        x_center = annotation['translation'][0]
        y_center = annotation['translation'][1]
        zoom_out = 50
        my_patch = ((x_center - zoom_out), (y_center - zoom_out), (x_center + zoom_out), (y_center + zoom_out))
        fig, ax = nusc_map.render_map_patch(my_patch, nusc_map.non_geometric_layers, figsize=(10, 10))

        ax.scatter(my_instance_pose_x, my_instance_pose_y, s=20, c='#FF0000', alpha=1.0, zorder=2)
        ax.scatter(my_lane_pose_x, my_lane_pose_y, s=20, c='#0000FF', alpha=1.0, zorder=2)
        ax.scatter(my_vehicle_pose_x, my_vehicle_pose_y, s=20, c='#00FF00', alpha=1.0, zorder=2)
        ax.scatter(x_center, y_center, s=20, c='#FFFF00', alpha=1.0, zorder=2)
        ax.scatter(my_in_front_pose_x, my_in_front_pose_y, s=20, c='#FFFF00', alpha=1.0, zorder=2)





    # database loading
    def _load_in_database(self, startIndex, endIndex):
        for index in range(startIndex, endIndex):
            sample_annotations = utils.get_sample_annotations_of_instance(self.nusc.instance[index], self.nusc)
            self.db.sample_annotations.insert_many(sample_annotations)



    # helper methods
    def _get_map_of_annotation(self, map_name):
        if map_name == 'singapore-onenorth':
            return self.nusc_map_singapore_onenorth
        if map_name == 'singapore-hollandvillage':
            return self.nusc_map_singapore_hollandvillage
        if map_name == 'singapore-queenstown':
            return self.nusc_map_singapore_queenstown
        if map_name == 'boston-seaport':
            return self.nusc_map_boston_seaport



myInstance = LoadImplementer()
