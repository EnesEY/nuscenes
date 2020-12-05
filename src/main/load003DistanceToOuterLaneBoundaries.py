import matplotlib.pyplot as plt
import load000setup 
import parameters
import math
import utils

# loads the attribute movedBefore into every sample_annotation
def loadDistanceToOuterLaneBoundaries():
    print('start adding distance to outer lane boundaries annotations')
    # Pick a sample and render the front camera image.
    my_point = (390, 1100)
    print(my_point)
    print('finished adding distance to outer lane boundaries annotations')


# Filter every point that does not on a lane with the same direction as the closest lane
def extractPointsOfLaneDirection2(steps):
    #setup
    annotation = load000setup.nusc.sample_annotation[100]
    road_segment = load000setup.getMapLayerOfSampleAnnotatino(annotation, 'road_segment')
    zoomOut = 20
    x_center, y_center, z_center = annotation['translation']
    my_patch = ((x_center - zoomOut), (y_center - zoomOut), (x_center + zoomOut), (y_center + zoomOut))
    nusc_map = load000setup.getMapOfSampleAnnotation(annotation)

    # execute
    # get direction of x and y of the nearest lane
    [delta_x_lane, delta_y_lane] = utils.getDeltaXAndDeltaYOfNearestLane(annotation)
    [delta_orthogonal_lane_x , delta_orthogonal_lane_y] = [delta_y_lane, (-1)*delta_x_lane]
    non_intersection_road_segments_nodes = getNodesOfRoadSegments(road_segment, nusc_map)
    [same_direction_nodes, opposite_direction_nodes] = getOppositeAndSameDirectionNodes(non_intersection_road_segments_nodes, delta_x_lane, delta_y_lane)
    
    start_point_left_lane = [x_center, y_center]
    end_point_left_lane =  [x_center + ((-1) * delta_orthogonal_lane_x) , y_center + ((-1) * delta_orthogonal_lane_y)]
    start_point_right_lane = [x_center, y_center] 
    end_point_right_lane = [x_center + delta_orthogonal_lane_x , y_center + delta_orthogonal_lane_y]
    # we create lane arrays with the points mentioned above
    lane_to_right_boundary = utils.interpolate(start_point_right_lane[0], start_point_right_lane[1], end_point_right_lane[0], end_point_right_lane[1], steps)
    lane_to_left_boundary = utils.interpolate(start_point_left_lane[0], start_point_left_lane[1], end_point_left_lane[0], end_point_left_lane[1], steps)

    # Get average point of both lanes
    # get minimal point of the interpolated thing
    [average_x_same_direction, average_y_same_direction] = getAverageXAndYOfLane(same_direction_nodes)
    [average_x_opposite_direction, average_y_opposite_direction] = getAverageXAndYOfLane(opposite_direction_nodes)

    # get closest point of lane to average
    closest_point_of_left_lane_to_opposite_average_point = getClosestPointOfLaneToPoint(lane_to_left_boundary, average_x_opposite_direction, average_y_opposite_direction)
    closest_point_of_right_lane_to_same_average_point = getClosestPointOfLaneToPoint(lane_to_right_boundary, average_x_same_direction, average_y_same_direction)
    closest_point_of_left_lane_to_opposite_average_distance = calculateDistanceBetweenTwoNodes(closest_point_of_left_lane_to_opposite_average_point['x'],closest_point_of_left_lane_to_opposite_average_point['y'],x_center, y_center)
    closest_point_of_right_lane_to_same_average_distance = calculateDistanceBetweenTwoNodes(closest_point_of_right_lane_to_same_average_point['x'],closest_point_of_right_lane_to_same_average_point['y'],x_center, y_center)
    # return [closest_point_of_left_lane_to_opposite_average_distance, closest_point_of_right_lane_to_same_average_distance]

    # output
    # plot everything
    # load both arrays to poses arrays that will be plotted
    poses_x_same_direction = []
    poses_y_same_direction = []
    poses_x_opposite_direction = []
    poses_y_opposite_direction = []
    poses_lane_to_left_boundary_x = []
    poses_lane_to_left_boundary_y = []
    poses_lane_to_right_boundary_x = []
    poses_lane_to_right_boundary_y = []
    pose_average_x = []
    pose_average_y = []
    pose_average_x.append(average_x_same_direction)
    pose_average_x.append(average_x_opposite_direction)
    pose_average_y.append(average_y_same_direction)
    pose_average_y.append(average_y_opposite_direction)
    pose_average_x.append(closest_point_of_left_lane_to_opposite_average_point['x'])
    pose_average_y.append(closest_point_of_left_lane_to_opposite_average_point['y'])
    pose_average_x.append(closest_point_of_right_lane_to_same_average_point['x'])
    pose_average_y.append(closest_point_of_right_lane_to_same_average_point['y'])

    for value in same_direction_nodes:
        poses_x_same_direction.append(value['x'])
        poses_y_same_direction.append(value['y'])
    for value in opposite_direction_nodes:
        poses_x_opposite_direction.append(value['x'])
        poses_y_opposite_direction.append(value['y'])
    for value in lane_to_left_boundary:
        poses_lane_to_left_boundary_x.append(value['x'])
        poses_lane_to_left_boundary_y.append(value['y'])
    for value in lane_to_right_boundary:
        poses_lane_to_right_boundary_x.append(value['x'])
        poses_lane_to_right_boundary_y.append(value['y'])

    fig, ax = nusc_map.render_map_patch(my_patch, nusc_map.non_geometric_layers, figsize=(10, 10))
    ax.scatter(x_center, y_center, s=20, c='#FF0000', alpha=1.0, zorder=2)
    ax.scatter(poses_x_same_direction, poses_y_same_direction, s=20, c='#00FF00', alpha=1.0, zorder=2)
    ax.scatter(poses_x_opposite_direction, poses_y_opposite_direction, s=20, c='#0000FF', alpha=1.0, zorder=2)
    ax.scatter(poses_lane_to_left_boundary_x, poses_lane_to_left_boundary_y, s=20, c='#FFFF00', alpha=1.0, zorder=2)
    ax.scatter(poses_lane_to_right_boundary_x, poses_lane_to_right_boundary_y, s=20, c='#00FFFF', alpha=1.0, zorder=2)
    ax.scatter(pose_average_x, pose_average_y, s=20, c='k', alpha=1.0, zorder=2)



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


extractPointsOfLaneDirection2(50)