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
import sample_annotation_utils as utils

"""
All methods start from instance basis and load sample_annotations from there
"""
class LoadSampleAnnotationsFromInstance:

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
        for start_and_end in self.process_ranges:
            self.processes.append(Process(target=self._load_annotations, args=(start_and_end[0], start_and_end[1], thread_number,)))
            thread_number += 1 

        for process in self.processes:
            process.start()
        
        for process in self.processes:
            process.join()

        print('finished loading annotaions')


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
                moving = False
                if abs(np.average(velocities)) > parameters.moving_treshold:
                    moving = True
                annotation['moving-state'] = moving
                continue
            else:
                if annotation['attribute_tokens'][0] == "c3246a1e22a14fcb878aa61e69ae3329" or annotation['attribute_tokens'][0] == "58aa28b1c2a54dc88e169808c07331e3":
                    annotation['moving-state'] = False
                    continue
                if annotation['attribute_tokens'][0] == "cb5118da1ab342aa947717dc53544259":
                    annotation['moving-state'] = True
                    continue
            velocities = list(self.nusc.box_velocity(annotation['token']))
            moving = False
            if abs(np.average(velocities)) > parameters.moving_treshold:
                moving = True
            annotation['moving-state'] = moving


    def _load_moved_before(self, sample_annotations):
        movedBefore = False
        for annotation in sample_annotations:
            if movedBefore == False and annotation['moving-state'] == True:
                movedBefore = True
            annotation['moved-before'] = movedBefore


    def _load_will_move(self, sample_annotations):
        willMove = False
        lastAnnotation = sample_annotations[len(sample_annotations)-1]
        if lastAnnotation['moved-before'] == True:
            willMove = True
        for annotation in sample_annotations:
            annotation['will-move'] = willMove


    def _load_time_not_moved(self, sample_annotations):
        timeNotMoved = 0
        for annotation in sample_annotations:
            if annotation['moving-state'] == True:
                timeNotMoved = 0
            if annotation['moving-state'] == False:
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
                    annotation['on-intersection'] = ''
                    annotation['distance-to-left-boundary'] = ''
                    annotation['distance-to-right-boundary'] = ''
                    continue

                self._load_on_intersection(annotation, road_segment)

                nearestLane = utils.get_closest_lane_of_annotaion(annotation, nusc_map)
                if nearestLane == '' or annotation['on-intersection'] == True:
                    annotation['distance-to-left-boundary'] = ''
                    annotation['distance-to-right-boundary'] = ''
                    continue

                self._load_distance_to_boundaries(annotation, nusc_map, road_segment, nearestLane)


    # loading of isOnStopLine
    def _load_is_on_stop_line(self, vehicle, layers, nusc_map_name):
        isOnStopLine = False
        
        if nusc_map_name == 'singapore-queenstown':
            vehicle['is-on-stop-line'] = ''
            return

        if 'stop_line' in layers:
            isOnStopLine = True

        vehicle['is-on-stop-line'] = isOnStopLine     


    # loading of isOnStopLine
    def _load_is_on_parking_area(self, vehicle, layers, nusc_map_name):
        isOnCarparkArea  = False
        
        if nusc_map_name == 'singapore-queenstown':
            vehicle['is-on-car-park-area'] = ''
            return

        if 'carpark_area' in layers:
            isOnCarparkArea = True

        vehicle['is-on-car-park-area'] = isOnCarparkArea  


    # loading of isEgoVehicle
    def _load_is_ego_vehicle(self, vehicle):
        isEgoVehicle = False
        
        category_name = vehicle['category_name']
        category_name = category_name.split(".") 
        if category_name[0] == 'vehicle' and category_name[1] =='ego':
            isEgoVehicle = True

        vehicle['is-ego-vehicle'] = isEgoVehicle    


    def _load_on_intersection(self, annotation, road_segment):
        onIntersection = road_segment['is_intersection']
        annotation['on-intersection'] = onIntersection


    def _load_distance_to_boundaries(self, annotation, nusc_map, road_segment, nearestLane):
        distance_to_left_boundary = ''
        distance_to_right_boundary = ''

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

# myInstance = LoadSampleAnnotationsFromInstance()