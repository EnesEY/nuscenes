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

        annotation = self.nusc.get('sample_annotation', '924ee6ac1fed440a9d9e3720aac635a0')
        sample = self.nusc.sample[0]

        print('started plotting figures')

        self._load_distance_to_lane_boundary_plot_init(annotation)
        self._load_area_in_front(annotation, parameters.in_front_sample_rate, parameters.in_front_distance)
        self._load_has_instance_in_front_of_all_vehicles_in_sample_init(sample)

        print('finished plotting figures')


    def _load_has_instance_in_front_of_all_vehicles_in_sample_init(self, sample):
        sample_annotations = utils.get_sample_annotations_of_sample(sample, self.nusc)
        vehicles_in_sample = utils.get_vehicles_from_sample_array(sample_annotations)
        nusc_map_name = utils.get_map_name_of_annotation(sample_annotations[0], self.nusc)
        nusc_map = self._get_map_of_annotation(nusc_map_name)

        for vehicle in vehicles_in_sample:
            nearestLane = utils.get_closest_lane_of_annotaion(vehicle, nusc_map)
            if nearestLane == '':
                vehicle['hasInstanceInFront'] = ''
            self._load_has_instance_in_front(vehicle, sample_annotations, nearestLane, nusc_map, parameters.in_front_sample_rate, parameters.in_front_point_radius, parameters.in_front_distance)

        self._load_has_instance_in_front_of_all_vehicles_in_sample(sample_annotations, nusc_map)


    def _load_has_instance_in_front_of_all_vehicles_in_sample(self, sample_annotations, nusc_map):
        annotation = sample_annotations[0]
        zoom_out = 50
        x_center = annotation['translation'][0]
        y_center = annotation['translation'][1]
        my_patch = ((x_center - zoom_out), (y_center - zoom_out), (x_center + zoom_out), (y_center + zoom_out))

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

        fig, ax = nusc_map.render_map_patch(my_patch, nusc_map.non_geometric_layers, figsize=(10, 10))
        self._add_annotations_to_plot(ax, vehicles_nothing_in_front, '#FF0000')
        self._add_annotations_to_plot(ax, vehicles_instance_in_front, '#0000FF')
        self._add_annotations_to_plot(ax, instance, '#FFFFFF')
        fig.savefig(f'plots/instances-in-sample.png', dpi=fig.dpi, bbox_inches='tight')


    def _load_area_in_front(self, annotation, sample_rate, capture_distance):
        zoom_out = 50

        nusc_map_name = utils.get_map_name_of_annotation(annotation, self.nusc)
        nusc_map = self._get_map_of_annotation(nusc_map_name)        

        nearestLane = utils.get_closest_lane_of_annotaion(annotation, nusc_map)
        if nearestLane == '':
            return

        [delta_x_lane, delta_y_lane] = utils.get_delta_x_and_delta_y_of_lane(annotation, nearestLane)

        x_center = annotation['translation'][0]
        y_center = annotation['translation'][1]
        my_patch = ((x_center - zoom_out), (y_center - zoom_out), (x_center + zoom_out), (y_center + zoom_out))

        start_x = annotation['translation'][0]
        start_y = annotation['translation'][1]
        end_x = (start_x + delta_x_lane) 
        end_y = (start_y + delta_y_lane)

        distance = utils.get_distance_between_two_points(start_x, start_y, end_x, end_y)

        end_x = start_x + ((delta_x_lane/distance) * capture_distance )
        end_y = start_y + ((delta_y_lane/distance) * capture_distance )

        current_vector = utils.interpolate(start_x, start_y, end_x, end_y, sample_rate)

        fig, ax = nusc_map.render_map_patch(my_patch, nusc_map.non_geometric_layers, figsize=(10, 10))
        self._add_node_array_to_plot(ax, current_vector, '#0000FF')
        ax.scatter(x_center, y_center, s=20, c='#FFFFFF', alpha=1.0, zorder=2)
        fig.savefig(f'plots/area-in-front.png', dpi=fig.dpi, bbox_inches='tight')


    # plot to outer lane boundaries
    def _load_distance_to_lane_boundary_plot_init(self, annotation):
        nusc_map_name = utils.get_map_name_of_annotation(annotation, self.nusc)
        nusc_map = self._get_map_of_annotation(nusc_map_name)

        x = annotation['translation'][0]
        y = annotation['translation'][1]
        layers = nusc_map.layers_on_point(x, y)
        self._load_is_on_lane(annotation, layers)

        road_segment = utils.get_road_segment_of_annotation(annotation, nusc_map, layers)
        if road_segment == '':
            return

        self._load_on_intersection(annotation, road_segment)

        nearestLane = utils.get_closest_lane_of_annotaion(annotation, nusc_map)
        if nearestLane == '' or annotation['on-intersection'] == True or annotation['is-on-lane'] == False:
            return

        self._load_distance_to_boundaries(annotation, nusc_map, road_segment, nearestLane, x, y)
    

    def _load_distance_to_boundaries(self, annotation, nusc_map, road_segment, nearestLane, x_center, y_center):
        zoom_out = 10
        sampling_rate = 100
        my_patch = ((x_center - zoom_out), (y_center - zoom_out), (x_center + zoom_out), (y_center + zoom_out))

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

        if isRightBoundaryValid == True:
            start_point_right_lane = [x_center, y_center] 
            end_point_right_lane = [x_center + orthogonal_delta_x_lane , y_center + orthogonal_delta_y_lane]
            lane_to_right_boundary = utils.interpolate(start_point_right_lane[0], start_point_right_lane[1], end_point_right_lane[0], end_point_right_lane[1], sampling_rate)
            closest_point_of_right_lane_to_same_average_point = utils.get_nearest_node_of_node_array_to_point(lane_to_right_boundary, average_x_same_direction, average_y_same_direction)

        # plotting and saving the plot
        fig, ax = nusc_map.render_map_patch(my_patch, nusc_map.non_geometric_layers, figsize=(10, 10))
        self._add_node_array_to_plot(ax, same_direction_nodes, '#0000FF')
        self._add_node_array_to_plot(ax, opposite_direction_nodes, '#00FF00')
        if isRightBoundaryValid == True:
            self._add_node_array_to_plot(ax, lane_to_right_boundary, '#FFFFFF')
            ax.scatter(closest_point_of_right_lane_to_same_average_point['x'], closest_point_of_right_lane_to_same_average_point['y'], s=20, c='#FF0000', alpha=1.0, zorder=2)
        if isLeftBoundaryValid == True:
            self._add_node_array_to_plot(ax, lane_to_left_boundary, '#FFFFFF')
            ax.scatter(closest_point_of_left_lane_to_opposite_average_point['x'], closest_point_of_left_lane_to_opposite_average_point['y'], s=20, c='#FF0000', alpha=1.0, zorder=2)
        ax.scatter(x_center, y_center, s=20, c='#000000', alpha=1.0, zorder=2)
        fig.savefig(f'plots/distance-to-lane-boundary.png', dpi=fig.dpi, bbox_inches='tight')



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

    def _load_on_intersection(self, annotation, road_segment):
        onIntersection = road_segment['is_intersection']
        annotation['on-intersection'] = onIntersection

    def _load_is_on_lane(self, vehicle, layers):
        isOnLane = False
        if 'lane' in layers:
            isOnLane = True
        vehicle['is-on-lane'] = isOnLane     

    def _add_node_array_to_plot(self, ax, node_array, color):
        poses_x = []
        poses_y = []
        for value in node_array:
            poses_x.append(value['x'])
            poses_y.append(value['y'])
        ax.scatter(poses_x, poses_y, s=20, c=color, alpha=1.0, zorder=2)

    def _add_annotations_to_plot(self, ax, annotations, color):
        poses_x = []
        poses_y = []
        for value in annotations:
            poses_x.append(value['translation'][0])
            poses_y.append(value['translation'][1])
        ax.scatter(poses_x, poses_y, s=20, c=color, alpha=1.0, zorder=2)

    def _load_has_instance_in_front(self, vehicle, sample_annotations, nearestLane, nusc_map, sample_rate, radius, capture_distance):
        hasInstanceInFront = False
        [delta_x_lane, delta_y_lane] = utils.get_delta_x_and_delta_y_of_lane(vehicle, nearestLane)

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



myInstance = LoadSinglePlot() 