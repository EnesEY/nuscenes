from nuscenes.nuscenes import NuScenes
from nuscenes.map_expansion.map_api import NuScenesMap
from nuscenes.map_expansion import arcline_path_utils


import numpy as np
import math
import sample_annotation_parameters as parameters
import sample_annotation_utils as utils

"""
All methods start from instance basis and load sample_annotations from there
"""
class LoadSinglePlot:

    def __init__(self):
        self.nusc = NuScenes(version='v1.0-mini', dataroot='/data/sets/nuscenes', verbose=True)
        self.nusc_map_singapore_onenorth = NuScenesMap(dataroot='/data/sets/nuscenes', map_name='singapore-onenorth')
        self.nusc_map_singapore_hollandvillage = NuScenesMap(dataroot='/data/sets/nuscenes', map_name='singapore-hollandvillage')
        self.nusc_map_singapore_queenstown = NuScenesMap(dataroot='/data/sets/nuscenes', map_name='singapore-queenstown')
        self.nusc_map_boston_seaport = NuScenesMap(dataroot='/data/sets/nuscenes', map_name='boston-seaport') 

        print('started loading annotaions')

        self._load_annotations()

        print('finished loading annotaions')


    def _load_annotations(self):
        sample = self.nusc.sample[0]
        sample_annotations = utils.get_sample_annotations_of_sample(sample, self.nusc)
        vehicles_in_sample = utils.get_vehicles_from_sample_array(sample_annotations)
        nusc_map_name = utils.get_map_name_of_annotation(sample_annotations[0], self.nusc)
        nusc_map = self._get_map_of_annotation(nusc_map_name)

        for vehicle in vehicles_in_sample:
            nearestLane = utils.get_closest_lane_of_annotaion(vehicle, nusc_map)
            if nearestLane == '':
                vehicle['hasInstanceInFront'] = ''
            self._load_has_instance_in_front(vehicle, sample_annotations, nearestLane, nusc_map, parameters.in_front_sample_rate, parameters.in_front_point_radius, parameters.in_front_distance)
       
       # self._plot_all_annotations_in_sample(sample_annotations, nusc_map)


    # loading of hasInstanceInFront
    def _load_has_instance_in_front(self, vehicle, sample_annotations, nearestLane, nusc_map, sample_rate, radius, capture_distance):
        zoom_out = 50
        hasInstanceInFront = False
        [delta_x_lane, delta_y_lane] = utils.get_delta_x_and_delta_y_of_lane(vehicle, nearestLane)

        x_center = vehicle['translation'][0]
        y_center = vehicle['translation'][1]
        my_patch = ((x_center - zoom_out), (y_center - zoom_out), (x_center + zoom_out), (y_center + zoom_out))

        start_x = vehicle['translation'][0]
        start_y = vehicle['translation'][1]
        end_x = (start_x + delta_x_lane) 
        end_y = (start_y + delta_y_lane)

        distance = utils.get_distance_between_two_points(start_x, start_y, end_x, end_y)

        end_x = start_x + ((delta_x_lane/distance) * capture_distance )
        end_y = start_y + ((delta_y_lane/distance) * capture_distance )

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

        vehicle['hasInstanceInFront'] = hasInstanceInFront     

        token = vehicle['token']
        fig, ax = nusc_map.render_map_patch(my_patch, nusc_map.non_geometric_layers, figsize=(10, 10))

        self._add_node_array_to_plot(ax, current_vector, '#0000FF')

        ax.scatter(x_center, y_center, s=20, c='#FFFFFF', alpha=1.0, zorder=2)
        fig.savefig(f'single_plots_in_front/{token}.png', dpi=fig.dpi, bbox_inches='tight')


    def _plot_all_annotations_in_sample(self, sample_annotations, nusc_map):
        annotation = sample_annotations[0]
        zoom_out = 50
        x_center = annotation['translation'][0]
        y_center = annotation['translation'][1]
        my_patch = ((x_center - zoom_out), (y_center - zoom_out), (x_center + zoom_out), (y_center + zoom_out))

        fig, ax = nusc_map.render_map_patch(my_patch, nusc_map.non_geometric_layers, figsize=(10, 10))

        poses_nothing_in_front_x = []
        poses_nothing_in_front_y = []

        poses_instance_in_front_x = []
        poses_instance_in_front_y = []

        poses_instance_x = []
        poses_instance_y = []

        vehicles_nothing_in_front = []
        vehicles_instance_in_front = []
        instance = []

        for annotation in sample_annotations:
            category_name = annotation['category_name']
            category_name = category_name.split(".") 
            if category_name[0] == 'vehicle':
                if annotation['hasInstanceInFront'] == True:
                    vehicles_instance_in_front.append(annotation)
                if annotation['hasInstanceInFront'] == False:
                    vehicles_nothing_in_front.append(annotation)
            else:
                instance.append(annotation)

        for point in vehicles_nothing_in_front:
            poses_nothing_in_front_x.append(point['translation'][0])
            poses_nothing_in_front_y.append(point['translation'][1])

        for point in vehicles_instance_in_front:
            poses_instance_in_front_x.append(point['translation'][0])
            poses_instance_in_front_y.append(point['translation'][1])
        
        for point in instance:
            poses_instance_x.append(point['translation'][0])
            poses_instance_y.append(point['translation'][1])
        
        token = annotation['token']
        ax.scatter(poses_nothing_in_front_x, poses_nothing_in_front_y, s=20, c='#FF0000', alpha=1.0, zorder=2)
        ax.scatter(poses_instance_in_front_x, poses_instance_in_front_y, s=20, c='#0000FF', alpha=1.0, zorder=2)
        ax.scatter(poses_instance_x, poses_instance_y, s=20, c='#FFFFFF', alpha=1.0, zorder=2)

        fig.savefig(f'all_instances_in_sample/{token}.png', dpi=fig.dpi, bbox_inches='tight')


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


    def _add_node_array_to_plot(self, ax, node_array, color):
        poses_x = []
        poses_y = []
        for value in node_array:
            poses_x.append(value['x'])
            poses_y.append(value['y'])
        ax.scatter(poses_x, poses_y, s=20, c=color, alpha=1.0, zorder=2)


myInstance = LoadSinglePlot() 