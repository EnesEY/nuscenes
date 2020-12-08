from nuscenes.nuscenes import NuScenes
from pymongo import MongoClient 
from nuscenes.map_expansion.map_api import NuScenesMap
from nuscenes.map_expansion import arcline_path_utils
from multiprocessing import Process

import os
import time
import numpy as np
import math
import sample_annotation_parameters as parameters
import sample_annotation_utils as utils

"""
All methods start from instance basis and load sample_annotations from there
"""
class LoadSampleAnnotation:

    def __init__(self):
        self.nusc = NuScenes(version='v1.0-mini', dataroot='/data/sets/nuscenes', verbose=True)
        self.client=MongoClient('mongodb+srv://enesey:485f6483e3c8666b72fda603a7f87006b83549a54395f2504eb58935f35d00d9@nuscenescluster.jh1vw.mongodb.net/test')
        self.db = self.client.nuscenes

        self.nusc_map_singapore_onenorth = NuScenesMap(dataroot='/data/sets/nuscenes', map_name='singapore-onenorth')
        self.nusc_map_singapore_hollandvillage = NuScenesMap(dataroot='/data/sets/nuscenes', map_name='singapore-hollandvillage')
        self.nusc_map_singapore_queenstown = NuScenesMap(dataroot='/data/sets/nuscenes', map_name='singapore-queenstown')
        self.nusc_map_boston_seaport = NuScenesMap(dataroot='/data/sets/nuscenes', map_name='boston-seaport') 

        print('started filtering process')

        self._load_only_necessary_instances()

        print('finished filtering process')

        print('started loading annotaions')
        self.processes = []
        self.process_ranges = utils.split_processes(self.nusc.instance)

        for start_and_end in self.process_ranges:
             self.processes.append(Process(target=self._load_annotations, args=(start_and_end[0], start_and_end[1],)))

        for process in self.processes:
            process.start()
        
        for process in self.processes:
            process.join()
        print('finished loading annotaions')



    def _load_annotations(self, start_index, end_index):
        self._load_moving_state(start_index, end_index)
        self._load_moved_before(start_index, end_index)
        self._load_time_not_moved(start_index, end_index)
        self._load_in_database(start_index, end_index)


    def _load_only_necessary_instances(self):
        instances = []
        for instance in self.nusc.instance:
            if utils.get_vehicle_and_not_ego_vehicle(instance, self.nusc) == True:
                instances.append(instance)
        self.nusc.instance = instances


    def _load_moving_state(self, startIndex, endIndex):
        for index in range(startIndex, endIndex):
            sample_annotations = utils.get_sample_annotations_of_instance(self.nusc.instance[index], self.nusc)
            for annotation in sample_annotations:
                velocities = list(self.nusc.box_velocity(annotation['token']))
                moving = False
                if abs(np.average(velocities)) > parameters.moving_treshold:
                    moving = True
                annotation['movingState'] = moving


    def _load_moved_before(self, startIndex, endIndex):
        for index in range(startIndex, endIndex):
            instance = self.nusc.instance[index]
            movedBefore = False
            first_annotation_token = instance['first_annotation_token']
            last_annotation_token = instance['last_annotation_token']
            first_annotation = self.nusc.get('sample_annotation', first_annotation_token)
            next_annotation_token = first_annotation['next']

            if movedBefore == False and first_annotation['movingState'] == True:
                movedBefore = True
            first_annotation['movedBefore'] = movedBefore

            while next_annotation_token != last_annotation_token and next_annotation_token != '':
                current_annotation = self.nusc.get('sample_annotation', next_annotation_token)
                if movedBefore == False and current_annotation['movingState'] == True:
                    movedBefore = True
                current_annotation['movedBefore'] = movedBefore
                next_annotation_token = current_annotation['next']


    def _load_time_not_moved(self, startIndex, endIndex):
        for index in range(startIndex, endIndex):
            instance = self.nusc.instance[index]
            timeNotMoved = 0
            first_annotation_token = instance['first_annotation_token']
            last_annotation_token = instance['last_annotation_token']
            first_annotation = self.nusc.get('sample_annotation', first_annotation_token)
            next_annotation_token = first_annotation['next']

            if first_annotation['movingState'] == True:
                timeNotMoved = 0
            if first_annotation['movingState'] == False:
                timeNotMoved += 1
            first_annotation['timeNotMoved'] = timeNotMoved

            while next_annotation_token != last_annotation_token and next_annotation_token != '':
                current_annotation = self.nusc.get('sample_annotation', next_annotation_token)
                if current_annotation['movingState'] == True:
                    timeNotMoved = 0
                if current_annotation['movingState'] == False:
                    timeNotMoved += 1
                current_annotation['timeNotMoved'] = timeNotMoved
                next_annotation_token = current_annotation['next']


    def _load_in_database(self, startIndex, endIndex):
        for index in range(startIndex, endIndex):
            sample_annotations = utils.get_sample_annotations_of_instance(self.nusc.instance[index], self.nusc)
            self.db.sample_annotations.insert_many(sample_annotations)


myInstance = LoadSampleAnnotation()