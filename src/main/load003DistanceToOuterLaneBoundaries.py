import matplotlib.pyplot as plt
import load000setup 
import parameters
import math
import utils
import os
from threading import Thread
from multiprocessing import Process


# loads the attribute movedBefore into every sample_annotation
def loadDistanceToOuterLaneBoundaries(annotations, number):
    print('start adding distance to outer lane boundaries annotations     ')

    processes = []

    process_step = math.floor(len(annotations)/16)

    for t in range(0, os.cpu_count()-1):
        processes.append(Process(target=execute_process, args=(t,annotations, process_step,)))

    for process in processes:
        process.start()
    
    for process in processes:
        process.join()

    for index in range(process_step * os.cpu_count()-1, len(annotations)):
        extractPointsOfLaneDirection(annotations[index])

    print('finished adding distance to outer lane boundaries annotations     ')


def execute_process(number_of_process, annotations, process_step):
    print(f'STARTED process Nr:{number_of_process}     ')
    for index in range(process_step * number_of_process, (process_step * number_of_process) + process_step):
        extractPointsOfLaneDirection(annotations[index])
    print(f'ENDED process Nr:{number_of_process} from range {process_step * number_of_process} to {(process_step * number_of_process) + process_step}     ')


# Filter every point that does not on a lane with the same direction as the closest lane
def extractPointsOfLaneDirection(annotation):
    closest_point_of_left_lane_to_opposite_average_distance = ''
    closest_point_of_right_lane_to_same_average_distance = ''

    if annotation is None:
        print('this annotatino has been empty')

    nusc_map = load000setup.getMapOfSampleAnnotation(annotation)

    # remove unnecessary layers for performance improvement
    nusc_map.non_geometric_polygon_layers = ['drivable_area', 'road_segment', 'lane']
    nusc_map.non_geometric_line_layers = []
    nusc_map.non_geometric_layers = ['drivable_area', 'road_segment', 'lane']

    if nusc_map == '':
        return ''

    # wcheck if this annotation is for a vehicle that is not the ego vehicle
    annotation['isEgoVehicle'] = load000setup.getIsThisVehicleAndNotEgoVehicle(annotation)
    if annotation['isEgoVehicle'] == False:
        return ''

    # check if it is on top of a road segment (this feature does not work if this is not the case)
    road_segment = load000setup.getMapLayerOfSampleAnnotatino(annotation, 'road_segment', nusc_map)
    if road_segment == '':
        return ''

    # can't calculate the distance in intersections
    annotation['onIntersection'] = road_segment['is_intersection']
    if road_segment['is_intersection'] == True:
        return ''

    nearestLane = load000setup.getClosestLaneOfSampleAnnotation(annotation, nusc_map)
    if nearestLane == '':
        return ''

    steps = 50 
    x_center, y_center, z_center = annotation['translation']

    # get direction of x and y of the nearest lane
    [delta_x_lane, delta_y_lane] = utils.getDeltaXAndDeltaYOfNearestLane(annotation, nearestLane)
    [delta_orthogonal_lane_x , delta_orthogonal_lane_y] = [delta_y_lane, (-1)*delta_x_lane]
    non_intersection_road_segments_nodes = getNodesOfRoadSegments(road_segment, nusc_map)
    [same_direction_nodes, opposite_direction_nodes] = getOppositeAndSameDirectionNodes(non_intersection_road_segments_nodes, delta_x_lane, delta_y_lane)
    
    # Get average point of both lanes
    [average_x_same_direction, average_y_same_direction] = getAverageXAndYOfLane(same_direction_nodes)
    [average_x_opposite_direction, average_y_opposite_direction] = getAverageXAndYOfLane(opposite_direction_nodes)

    isLeftBoundaryValid = True
    if average_x_opposite_direction == 0 or average_y_opposite_direction == 0:
        isLeftBoundaryValid = False
    
    isRightBoundaryValid = True
    if average_x_same_direction == 0 or average_y_same_direction == 0:
        isRightBoundaryValid = False

    if isLeftBoundaryValid == True:
        start_point_left_lane = [x_center, y_center]
        end_point_left_lane =  [x_center + ((-1) * delta_orthogonal_lane_x) , y_center + ((-1) * delta_orthogonal_lane_y)]
        lane_to_left_boundary = utils.interpolate(start_point_left_lane[0], start_point_left_lane[1], end_point_left_lane[0], end_point_left_lane[1], steps)
        closest_point_of_left_lane_to_opposite_average_point = getClosestPointOfLaneToPoint(lane_to_left_boundary, average_x_opposite_direction, average_y_opposite_direction)
        closest_point_of_left_lane_to_opposite_average_distance = calculateDistanceBetweenTwoNodes(closest_point_of_left_lane_to_opposite_average_point['x'],closest_point_of_left_lane_to_opposite_average_point['y'],x_center, y_center)

    if isRightBoundaryValid == True:
        start_point_right_lane = [x_center, y_center] 
        end_point_right_lane = [x_center + delta_orthogonal_lane_x , y_center + delta_orthogonal_lane_y]
        lane_to_right_boundary = utils.interpolate(start_point_right_lane[0], start_point_right_lane[1], end_point_right_lane[0], end_point_right_lane[1], steps)
        closest_point_of_right_lane_to_same_average_point = getClosestPointOfLaneToPoint(lane_to_right_boundary, average_x_same_direction, average_y_same_direction)
        closest_point_of_right_lane_to_same_average_distance = calculateDistanceBetweenTwoNodes(closest_point_of_right_lane_to_same_average_point['x'],closest_point_of_right_lane_to_same_average_point['y'],x_center, y_center)

    annotation['distanceToLeftBoundary'] = closest_point_of_left_lane_to_opposite_average_distance
    annotation['distanceToRightBoundary'] = closest_point_of_right_lane_to_same_average_distance

    load000setup.db.sample_annotation.insert_one(annotation)


def getNodesOfRoadSegments(road_segments, nusc_map):
    # setup
    output = []
    # execute
    for nodes_token in road_segments['exterior_node_tokens']:
        for node in nusc_map.node:
            if node['token'] == nodes_token:
                output.append(node)
    # output
    return output

def getOppositeAndSameDirectionNodes(non_intersection_road_segments_nodes, delta_x_lane, delta_y_lane):
    same_direction_nodes = []
    opposite_direction_nodes = []
    for i in range(0, len(non_intersection_road_segments_nodes)-1):
        current_delta_x = non_intersection_road_segments_nodes[i+1]['x'] - non_intersection_road_segments_nodes[i]['x']
        current_delta_y = non_intersection_road_segments_nodes[i+1]['y'] - non_intersection_road_segments_nodes[i]['y']
        if (delta_x_lane * current_delta_x) > 0 and (delta_y_lane * current_delta_y) > 0:
            same_direction_nodes.append(non_intersection_road_segments_nodes[i])
        if (delta_x_lane * current_delta_x) < 0 and (delta_y_lane * current_delta_y) < 0:
            opposite_direction_nodes.append(non_intersection_road_segments_nodes[i])
    return [same_direction_nodes, opposite_direction_nodes]


def getAverageXAndYOfLane(nodes):
    average_x = 0
    average_y = 0
    for point in nodes:
        average_x += point['x']
        average_y += point['y']
    if len(nodes) != 0:
        average_x = (average_x/len(nodes))
        average_y = (average_y/len(nodes))
    return [average_x, average_y]

def getClosestPointOfLaneToPoint(lane, x_of_point, y_of_point):
    max = 999999
    for point in lane:
        delta_x = x_of_point - point['x']
        delta_y = y_of_point - point['y']
        distance = math.sqrt(pow(delta_x, 2) + pow(delta_y, 2))
        if distance < max:
            max = distance
            closest_point = point
    return closest_point

def calculateDistanceBetweenTwoNodes(x1, y1, x2, y2):
    delta_x = x2 - x1
    delta_y = y2 - y1

    distance = math.sqrt(pow(delta_x, 2) + pow(delta_y, 2))
    return distance


# annotation = load000setup.nusc.sample_annotation[100]
# extractPointsOfLaneDirection(annotation)

# annotation = []
# annotation.append(load000setup.nusc.sample_annotation[100])
# annotation.append(load000setup.nusc.sample_annotation[101])
# loadDistanceToOuterLaneBoundaries(annotation, 0)