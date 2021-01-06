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

        self._load_instance_map_annotation()

        print('finished loading annotaions')


    # instance map annotations
    def _load_instance_map_annotation(self):
        annotation = ''

        for my_annotation in self.nusc.sample_annotation:
            if my_annotation['token'] == '924ee6ac1fed440a9d9e3720aac635a0':
                annotation = my_annotation
        
        nusc_map_name = utils.get_map_name_of_annotation(annotation, self.nusc)
        nusc_map = self._get_map_of_annotation(nusc_map_name)
        x = annotation['translation'][0]
        y = annotation['translation'][1]
        layers = nusc_map.layers_on_point(x, y)

        self._load_is_on_lane(annotation, layers)

        road_segment = utils.get_road_segment_of_annotation(annotation, nusc_map, layers)
        if road_segment == '':
            annotation['distanceToLeftBoundary'] = ''
            annotation['distanceToRightBoundary'] = ''

        self._load_on_intersection(annotation, road_segment)

        # REMOVE THE onIntersection PART TO HAVE EXAMPLES WHERE IT TRIES TO DO SHIT ON INTERSECTION
        # REMOVE THE isOnLane PART TO HAVE EXAMPLES WHERE IT TRIES TO DO SHIT ON INTERSECTION
        nearestLane = utils.get_closest_lane_of_annotaion(annotation, nusc_map)
        if nearestLane == '' or annotation['onIntersection'] == True or annotation['isOnLane'] == False:
            annotation['distanceToLeftBoundary'] = ''
            annotation['distanceToRightBoundary'] = ''

        self._load_distance_to_boundaries(annotation, nusc_map, road_segment, nearestLane)
    

    def _load_on_intersection(self, annotation, road_segment):
        onIntersection = road_segment['is_intersection']
        annotation['onIntersection'] = onIntersection

    # loading of isOnLane
    def _load_is_on_lane(self, vehicle, layers):
        isOnLane = False

        if 'lane' in layers:
            isOnLane = True

        vehicle['isOnLane'] = isOnLane     


    def _load_distance_to_boundaries(self, annotation, nusc_map, road_segment, nearestLane):
        zoom_out = 10
        distance_to_left_boundary = ''
        distance_to_right_boundary = ''

        # just for performance improvement
        nusc_map.non_geometric_polygon_layers = ['drivable_area', 'road_segment', 'lane']
        nusc_map.non_geometric_line_layers = []
        nusc_map.non_geometric_layers = ['drivable_area', 'road_segment', 'lane']

        x_center = annotation['translation'][0]
        y_center = annotation['translation'][1]
        my_patch = ((x_center - zoom_out), (y_center - zoom_out), (x_center + zoom_out), (y_center + zoom_out))

        # get direction of x and y of the nearest lane
        [delta_x_lane, delta_y_lane] = utils.get_delta_x_and_delta_y_of_lane(annotation, nearestLane)
        [orthogonal_delta_x_lane , orthogonal_delta_y_lane] = [delta_y_lane, (-1)*delta_x_lane]
        road_segment_nodes = utils.get_nodes_of_road_segment(road_segment, nusc_map)
        [same_direction_nodes, opposite_direction_nodes] = utils.get_opposite_and_same_direction_of_node_array(road_segment_nodes, delta_x_lane, delta_y_lane)

        # sampling_rate = round(math.sqrt(pow(delta_x_lane, 2) + pow(delta_y_lane, 2)))*2
        sampling_rate = 100
          
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

        token = annotation['token']
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

        fig.savefig(f'single_plots/{token}.png', dpi=fig.dpi, bbox_inches='tight')


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