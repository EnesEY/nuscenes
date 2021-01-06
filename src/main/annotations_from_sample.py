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

"""
All methods start from sample basis and load sample_annotations from there
"""
class LoadSampleAnnotationsFromSamples:

    def __init__(self):
        self.nusc = NuScenes(version='v1.0-mini', dataroot='/data/sets/nuscenes', verbose=True)
        self.client=MongoClient('mongodb+srv://enesey:485f6483e3c8666b72fda603a7f87006b83549a54395f2504eb58935f35d00d9@nuscenescluster.jh1vw.mongodb.net/test')
        self.db = self.client.nuscenes

        self.nusc_map_singapore_onenorth = NuScenesMap(dataroot='/data/sets/nuscenes', map_name='singapore-onenorth')
        self.nusc_map_singapore_hollandvillage = NuScenesMap(dataroot='/data/sets/nuscenes', map_name='singapore-hollandvillage')
        self.nusc_map_singapore_queenstown = NuScenesMap(dataroot='/data/sets/nuscenes', map_name='singapore-queenstown')
        self.nusc_map_boston_seaport = NuScenesMap(dataroot='/data/sets/nuscenes', map_name='boston-seaport') 

        print('started loading annotaions')

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

        print('finished loading annotaions')


    def _load_annotations(self, start_index, end_index, thread_number):
        for index in tqdm(range(start_index, end_index), desc=f'sample map annotation progress of thread nr. {thread_number}:'):
            sample = self.nusc.sample[index]
            sample_annotations = utils.get_sample_annotations_of_sample(sample, self.nusc)
            vehicles_in_sample = utils.get_vehicles_from_sample_array(sample_annotations)
            nusc_map_name = utils.get_map_name_of_annotation(sample_annotations[0], self.nusc)
            nusc_map = self._get_map_of_annotation(nusc_map_name)

            for vehicle in vehicles_in_sample:
                nearestLane = utils.get_closest_lane_of_annotaion(vehicle, nusc_map)
                if nearestLane == '':
                    vehicle['has-instasnce-in-front'] = ''
                    self._update_in_database(vehicle)
                    continue

                self._load_has_instance_in_front(vehicle, sample_annotations, nearestLane, nusc_map, parameters.in_front_sample_rate, parameters.in_front_point_radius, parameters.in_front_distance)
                self._update_in_database(vehicle)


    # loading of hasInstanceInFront
    def _load_has_instance_in_front(self, vehicle, sample_annotations, nearestLane, nusc_map, sample_rate, radius, capture_distance):
        hasInstanceInFront = False
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
                    hasInstanceInFront = True

        vehicle['has-instasnce-in-front'] = hasInstanceInFront     


    # database loading
    def _update_in_database(self, vehicle):
        query = {"token": vehicle['token']}
        newValue = {"$set": {"has-instasnce-in-front": vehicle['has-instasnce-in-front'],}}
        self.db.sample_annotations.update_one(query, newValue)


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

# myInstance = LoadSampleAnnotationsFromSamples()