from nuscenes.nuscenes import NuScenes
from pymongo import MongoClient 
from nuscenes.map_expansion.map_api import NuScenesMap
from nuscenes.map_expansion import arcline_path_utils
from multiprocessing import Process
from tqdm import tqdm

import os
import numpy as np
import math
import sample_annotation_parameters as parameters
import utils as utils


# this method loads sample_annotations into the mongodb database with all the features that this project added
def load_features(dbPath):
    LoadFeaturesFromInstance(dbPath)
    LoadFeaturesFromSamples(dbPath)

features = ["moving-state", "moved-before",
            "will-move", "time-not-moved", "vehicle-volume", "is-on-stop-line", "is-on-car-park-area","is-ego-vehicle",
            "on-intersection", "distance-to-left-boundary", "distance-to-right-boundary","has-instance-in-front"]

information = ["moving-state", "moved-before", "time-not-moved", "vehicle-volume", "is-on-stop-line", "is-on-car-park-area","is-ego-vehicle",
            "on-intersection", "distance-to-left-boundary", "distance-to-right-boundary","has-instance-in-front"]

labels = ["will-move"]

"""
This class goes through every sample annotation of all instances to load sample_annotations
"""
class LoadFeaturesFromInstance:

    def __init__(self, dbPath):
        self.nusc = NuScenes(version='v1.0-trainval', dataroot='/data/sets/nuscenes', verbose=True)
        self.client=MongoClient(dbPath)
        self.db = self.client.nuscenes

        self.nusc_map_singapore_onenorth = NuScenesMap(dataroot='/data/sets/nuscenes', map_name='singapore-onenorth')
        self.nusc_map_singapore_hollandvillage = NuScenesMap(dataroot='/data/sets/nuscenes', map_name='singapore-hollandvillage')
        self.nusc_map_singapore_queenstown = NuScenesMap(dataroot='/data/sets/nuscenes', map_name='singapore-queenstown')
        self.nusc_map_boston_seaport = NuScenesMap(dataroot='/data/sets/nuscenes', map_name='boston-seaport') 

        self._filter_unnecessary_instances()

        print('started loading annotaions from instances')
        self.processes = []
        self.process_ranges = utils.split_processes(self.nusc.instance)

        thread_number = 0
        for start_and_end in self.process_ranges:
            self.processes.append(Process(target=self._load_annotations, args=(start_and_end[0], start_and_end[1], thread_number,)))
            thread_number += 1 

        for process in self.processes:
            process.start()
        
        for process in self.processes:
            process.join()

        print('finished loading annotaions from instances')


    # Filtering
    def _filter_unnecessary_instances(self):
        instances = []
        for instance in self.nusc.instance:
            if utils.get_is_vehicle(instance, self.nusc) == True:
                instances.append(instance)
        self.nusc.instance = instances


    # instance annotations
    def _load_annotations(self, start_index, end_index, thread_number):
        self._load_instance_annotations(start_index, end_index, thread_number)
        self._load_instance_map_annotations(start_index, end_index, thread_number)
        self._load_in_database(start_index, end_index)


    def _load_instance_annotations(self, start_index, end_index, thread_number):
        for index in range(start_index, end_index):
            sample_annotations = utils.get_sample_annotations_of_instance(self.nusc.instance[index], self.nusc)
            self._load_moving_state(sample_annotations)
            self._load_moved_before(sample_annotations)
            self._load_will_move(sample_annotations)
            self._load_time_not_moved(sample_annotations)
            self._load_vehicle_volume(sample_annotations)      


    def _load_moving_state(self, sample_annotations):
        for annotation in sample_annotations:
            if annotation['attribute_tokens'] == '' or annotation['attribute_tokens'] == []:
                velocities = list(self.nusc.box_velocity(annotation['token']))
                moving = 0
                if abs(np.average(velocities)) > parameters.moving_treshold:
                    moving = 1
                annotation['moving-state'] = moving
                continue
            else:
                if annotation['attribute_tokens'][0] == "c3246a1e22a14fcb878aa61e69ae3329" or annotation['attribute_tokens'][0] == "58aa28b1c2a54dc88e169808c07331e3":
                    annotation['moving-state'] = 0
                    continue
                if annotation['attribute_tokens'][0] == "cb5118da1ab342aa947717dc53544259":
                    annotation['moving-state'] = 1
                    continue
            velocities = list(self.nusc.box_velocity(annotation['token']))
            moving = 0
            if abs(np.average(velocities)) > parameters.moving_treshold:
                moving = 1
            annotation['moving-state'] = moving


    def _load_moved_before(self, sample_annotations):
        movedBefore = 0
        for annotation in sample_annotations:
            if movedBefore == 0 and annotation['moving-state'] == 1:
                movedBefore = 1
            annotation['moved-before'] = movedBefore


    def _load_will_move(self, sample_annotations):
        willMove = 0
        lastAnnotation = sample_annotations[len(sample_annotations)-1]
        if lastAnnotation['moved-before'] == 1:
            willMove = 1
        for annotation in sample_annotations:
            annotation['will-move'] = willMove


    def _load_time_not_moved(self, sample_annotations):
        timeNotMoved = 0
        for annotation in sample_annotations:
            if annotation['moving-state'] == 1:
                timeNotMoved = 0
            if annotation['moving-state'] == 0:
                timeNotMoved += 1
            annotation['time-not-moved'] = timeNotMoved


    def _load_vehicle_volume(self, sample_annotations):
        x,y,z = sample_annotations[0]['size']
        vehicle_volume = x * y * z
        for annotation in sample_annotations:
            annotation['vehicle-volume'] = vehicle_volume


    # instance map annotations
    def _load_instance_map_annotations(self, start_index, end_index, thread_number):
        for index in tqdm(range(start_index, end_index), desc=f'instance map annotation progress of thread nr. {thread_number}:'):
            sample_annotations = utils.get_sample_annotations_of_instance(self.nusc.instance[index], self.nusc)
            nusc_map_name = utils.get_map_name_of_annotation(sample_annotations[0], self.nusc)

            if nusc_map_name == 'singapore-queenstown':
                for annotation in sample_annotations:
                    annotation['is-on-stop-line'] = -1
                    annotation['is-on-car-park-area'] = -1
                    annotation['is-ego-vehicle'] = -1
                    annotation['on-intersection'] = -1
                    annotation['distance-to-left-boundary'] = -1
                    annotation['distance-to-right-boundary'] = -1
                continue

            nusc_map = self._get_map_of_annotation(nusc_map_name)
            for annotation in sample_annotations:
                x = annotation['translation'][0]
                y = annotation['translation'][1]
                layers = nusc_map.layers_on_point(x, y)
                self._load_is_on_stop_line(annotation, layers, nusc_map_name)
                self._load_is_on_parking_area(annotation, layers, nusc_map_name)
                self._load_is_ego_vehicle(annotation)

                road_segment = utils.get_road_segment_of_annotation(annotation, nusc_map, layers)
                if road_segment == '':
                    annotation['on-intersection'] = -1
                    annotation['distance-to-left-boundary'] = -1
                    annotation['distance-to-right-boundary'] = -1
                    continue

                self._load_on_intersection(annotation, road_segment)

                nearestLane = utils.get_closest_lane_of_annotaion(annotation, nusc_map)
                if nearestLane == '' or annotation['on-intersection'] == 1:
                    annotation['distance-to-left-boundary'] = -1
                    annotation['distance-to-right-boundary'] = -1
                    continue

                self._load_distance_to_boundaries(annotation, nusc_map, road_segment, nearestLane)


    # loading of isOnStopLine
    def _load_is_on_stop_line(self, vehicle, layers, nusc_map_name):
        isOnStopLine = 0
        
        if nusc_map_name == 'singapore-queenstown':
            vehicle['is-on-stop-line'] = -1
            return

        if 'stop_line' in layers:
            isOnStopLine = 1

        vehicle['is-on-stop-line'] = isOnStopLine     


    # loading of isOnStopLine
    def _load_is_on_parking_area(self, vehicle, layers, nusc_map_name):
        isOnCarparkArea  = 0
        
        if nusc_map_name == 'singapore-queenstown':
            vehicle['is-on-car-park-area'] = -1
            return

        if 'carpark_area' in layers:
            isOnCarparkArea = 1

        vehicle['is-on-car-park-area'] = isOnCarparkArea  


    # loading of isEgoVehicle
    def _load_is_ego_vehicle(self, vehicle):
        isEgoVehicle = 0
        
        category_name = vehicle['category_name']
        category_name = category_name.split(".") 
        if category_name[0] == 'vehicle' and category_name[1] =='ego':
            isEgoVehicle = 1

        vehicle['is-ego-vehicle'] = isEgoVehicle    


    def _load_on_intersection(self, annotation, road_segment):
        onIntersection = road_segment['is_intersection']
        if onIntersection == True:
            annotation['on-intersection'] = 1
        else:
            annotation['on-intersection'] = 0


    def _load_distance_to_boundaries(self, annotation, nusc_map, road_segment, nearestLane):
        distance_to_left_boundary = -1
        distance_to_right_boundary = -1

        # just for performance improvement
        nusc_map.non_geometric_polygon_layers = ['drivable_area', 'road_segment', 'lane']
        nusc_map.non_geometric_line_layers = []
        nusc_map.non_geometric_layers = ['drivable_area', 'road_segment', 'lane']

        x_center = annotation['translation'][0]
        y_center = annotation['translation'][1]

        # get direction of x and y of the nearest lane
        [delta_x_lane, delta_y_lane] = utils.get_delta_x_and_delta_y_of_lane(annotation, nearestLane)
        [orthogonal_delta_x_lane , orthogonal_delta_y_lane] = [delta_y_lane, (-1)*delta_x_lane]
        road_segment_nodes = utils.get_nodes_of_road_segment(road_segment, nusc_map)
        [same_direction_nodes, opposite_direction_nodes] = utils.get_opposite_and_same_direction_of_node_array(road_segment_nodes, delta_x_lane, delta_y_lane)
                    
        sampling_rate = round(math.sqrt(pow(delta_x_lane, 2) + pow(delta_y_lane, 2)))*2

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

        annotation['distance-to-left-boundary'] = distance_to_left_boundary
        annotation['distance-to-right-boundary'] = distance_to_right_boundary      


    # database loading
    def _load_in_database(self, startIndex, endIndex):
        for index in range(startIndex, endIndex):
            sample_annotations = utils.get_sample_annotations_of_instance(self.nusc.instance[index], self.nusc)
            features = []
            for annotation in sample_annotations:
                feature = {
                    "token": annotation['token'],
                    "sample_token":annotation['sample_token'],
                    "category_name": annotation['category_name'],
                    "moving-state": annotation['moving-state'],
                    "moved-before": annotation['moved-before'],
                    "will-move": annotation['will-move'],
                    "time-not-moved": annotation['time-not-moved'],
                    "vehicle-volume":  annotation['vehicle-volume'],
                    "is-on-stop-line": annotation['is-on-stop-line'],  
                    "is-on-car-park-area": annotation['is-on-car-park-area'],
                    "is-ego-vehicle":  annotation['is-ego-vehicle'],
                    "on-intersection": annotation['on-intersection'],  
                    "distance-to-left-boundary": annotation['distance-to-left-boundary'],
                    "distance-to-right-boundary": annotation['distance-to-right-boundary'],
                }
                features.append(feature)

            self.db.features.insert_many(features)


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




"""
These methods go through scenes and their samples and all the sample_annotations in the samples to load sample_annotations
"""
class LoadFeaturesFromSamples:

    def __init__(self, dbPath):
        self.nusc = NuScenes(version='v1.0-trainval', dataroot='/data/sets/nuscenes', verbose=True)
        self.client=MongoClient(dbPath)
        self.db = self.client.nuscenes

        self.nusc_map_singapore_onenorth = NuScenesMap(dataroot='/data/sets/nuscenes', map_name='singapore-onenorth')
        self.nusc_map_singapore_hollandvillage = NuScenesMap(dataroot='/data/sets/nuscenes', map_name='singapore-hollandvillage')
        self.nusc_map_singapore_queenstown = NuScenesMap(dataroot='/data/sets/nuscenes', map_name='singapore-queenstown')
        self.nusc_map_boston_seaport = NuScenesMap(dataroot='/data/sets/nuscenes', map_name='boston-seaport') 

        print('started loading annotaions from samples')

        self.processes = []
        self.sample_process_ranges = utils.split_processes(self.nusc.sample)

        thread_number = 0
        for start_and_end in self.sample_process_ranges:
            self.processes.append(Process(target=self._load_annotations, args=(start_and_end[0], start_and_end[1], thread_number,)))
            thread_number += 1 

        for process in self.processes:
            process.start()
        
        for process in self.processes:
            process.join()

        print('finished loading annotaions from samples')


    def _load_annotations(self, start_index, end_index, thread_number):
        for index in tqdm(range(start_index, end_index), desc=f'sample map annotation progress of thread nr. {thread_number}:'):
            sample = self.nusc.sample[index]
            sample_annotations = utils.get_sample_annotations_of_sample(sample, self.nusc)
            vehicles_in_sample = utils.get_vehicles_from_sample_array(sample_annotations)
            nusc_map_name = utils.get_map_name_of_annotation(sample_annotations[0], self.nusc)

            if nusc_map_name == 'singapore-queenstown':
                for vehicle in vehicles_in_sample:
                    vehicle['has-instance-in-front'] = -1
                continue

            nusc_map = self._get_map_of_annotation(nusc_map_name)

            for vehicle in vehicles_in_sample:
                nearestLane = utils.get_closest_lane_of_annotaion(vehicle, nusc_map)
                if nearestLane == '':
                    vehicle['has-instance-in-front'] = -1
                    self._update_in_database(vehicle)
                    continue

                self._load_has_instance_in_front(vehicle, sample_annotations, nearestLane, nusc_map, parameters.in_front_sample_rate, parameters.in_front_point_radius, parameters.in_front_distance)
                self._update_in_database(vehicle)


    # loading of hasInstanceInFront
    def _load_has_instance_in_front(self, vehicle, sample_annotations, nearestLane, nusc_map, sample_rate, radius, capture_distance):
        hasInstanceInFront = 0
        [delta_x_lane, delta_y_lane] = utils.get_delta_x_and_delta_y_of_lane(vehicle, nearestLane)

        start_x = vehicle['translation'][0]
        start_y = vehicle['translation'][1]
        end_x = (start_x + delta_x_lane) 
        end_y = (start_y + delta_y_lane)

        distance = utils.get_distance_between_two_points(start_x, start_y, end_x, end_y)

        end_x = start_x + ((delta_x_lane/distance) * capture_distance)
        end_y = start_y + ((delta_y_lane/distance) * capture_distance)

        current_vector = utils.interpolate(start_x, start_y, end_x, end_y, sample_rate)

        for annotation in sample_annotations:
            current_x = annotation['translation'][0]
            current_y = annotation['translation'][1]

            for node in current_vector:
                node_x = node['x']
                node_y = node['y']
                distance2 = utils.get_distance_between_two_points(node_x, node_y, current_x, current_y)
                if abs(distance2) < radius and annotation['token'] != vehicle['token']:
                    hasInstanceInFront = 1

        vehicle['has-instance-in-front'] = hasInstanceInFront     


    # database loading
    def _update_in_database(self, vehicle):
        query = {"token": vehicle['token']}
        newValue = {"$set": {"has-instance-in-front": vehicle['has-instance-in-front'],}}
        self.db.features.update_one(query, newValue)


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



# myInstance = LoadSampleAnnotationsFromInstance()