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
# def extractPointsOfLaneDirection(steps):
#     #setup
#     annotation = load000setup.nusc.sample_annotation[100]
#     road_segment = load000setup.getMapLayerOfSampleAnnotatino(annotation, 'road_segment')
#     zoomOut = 10
#     x_center, y_center, z_center = annotation['translation']
#     my_patch = ((x_center - zoomOut), (y_center - zoomOut), (x_center + zoomOut), (y_center + zoomOut))
#     nusc_map = load000setup.getMapOfSampleAnnotation(annotation)
#     non_intersection_road_segments_nodes = []
#     same_direction_nodes = []
#     interpolated_same_direction_nodes = []

#     # execute
#     # make sure that the road segment is not an intersection
#     if road_segment['is_intersection'] == True:
#         return
#     # get direction of the nearest lane that the road segment is on
#     # we assume that the road segment has a similar direction
#     laneDirection = utils.getDirectionOfNearestLane(annotation)

    


#     orthogonalLaneDirection = utils.getOrthogonalDirectionOfNearestLane(annotation)
#     # load the nodes of the road segment into an array
#     non_intersection_road_segments_nodes = getNodesOfRoadSegments(road_segment, nusc_map)
#     # we extract nodes of the road segment that are in the same direction of the lane
#     for i in range(0, len(non_intersection_road_segments_nodes)-1):
#         direction2 = utils.getDirectionOfTwoNodes(non_intersection_road_segments_nodes[i] , non_intersection_road_segments_nodes[i+1])
#         direction3 = utils.getDirectionOfTwoNodes(non_intersection_road_segments_nodes[i] , non_intersection_road_segments_nodes[i-1])
#         if utils.compareDirections(laneDirection, direction2) == 'same_direction' or utils.compareDirections(laneDirection, direction3) == 'same_direction':
#             same_direction_nodes.append(non_intersection_road_segments_nodes[i])
#     # start and end points for the vectors to the outer lanes are created
#     start_point_right_lane = [x_center, y_center]
#     end_point_right_lane = [x_center + (x_center * orthogonalLaneDirection), y_center+ (y_center * orthogonalLaneDirection)]
#     start_point_left_lane = [x_center+((-1)*x_center * orthogonalLaneDirection),y_center+ ((-1)*y_center * orthogonalLaneDirection)]
#     end_point_left_lane = [x_center, y_center] 
#     # we create lane arrays with the points mentioned above
#     laneToRightLaneBoundary = utils.interpolate(start_point_right_lane[0], start_point_right_lane[1], end_point_right_lane[0], end_point_right_lane[1], steps)
#     laneToLeftLaneBoundary = utils.interpolate(start_point_left_lane[0], start_point_left_lane[1], end_point_left_lane[0], end_point_left_lane[1], steps)


#     # get all directions between two points in same direction node array 
#     first_lane = []
#     second_lane = []
#     for i in range(0, len(same_direction_nodes)-1):
#         current_x = same_direction_nodes[i]['x']
#         next_x = same_direction_nodes[i+1]['x']
#         if (current_x - next_x) < 0:
#             first_lane.append(same_direction_nodes[i])
#         if (current_x - next_x) > 0: 
#             second_lane.append(same_direction_nodes[i])

#     poses_first_lane_x = []
#     poses_first_lane_y = []
#     for value in first_lane:
#         poses_first_lane_x.append(value['x'])
#         poses_first_lane_y.append(value['y'])

    
#     poses_second_lane_x = []
#     poses_second_lane_y = []
#     for value in second_lane:
#         poses_second_lane_x.append(value['x'])
#         poses_second_lane_y.append(value['y'])

#     fig, ax = nusc_map.render_map_patch(my_patch, nusc_map.non_geometric_layers, figsize=(10, 10))
#     ax.scatter(x_center, y_center, s=20, c='#33FFFC', alpha=1.0, zorder=2)
#     ax.scatter(poses_first_lane_x, poses_first_lane_y, s=20, c='#33F33C', alpha=1.0, zorder=2)
#     ax.scatter(poses_second_lane_x, poses_second_lane_y, s=20, c='#55533C', alpha=1.0, zorder=2)


def getNonInterSectionRoadSegments(road_segments):
    # setup
    output = []
    # execute
    for road_segment in road_segments:
        if road_segments['is_intersection'] == False:
            output.append(road_segment)
    # output
    return output

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


# Filter every point that does not on a lane with the same direction as the closest lane
def extractPointsOfLaneDirection2(steps):
    #setup
    annotation = load000setup.nusc.sample_annotation[100]
    road_segment = load000setup.getMapLayerOfSampleAnnotatino(annotation, 'road_segment')
    zoomOut = 20
    x_center, y_center, z_center = annotation['translation']
    my_patch = ((x_center - zoomOut), (y_center - zoomOut), (x_center + zoomOut), (y_center + zoomOut))
    nusc_map = load000setup.getMapOfSampleAnnotation(annotation)
    same_direction_nodes = []
    # execute
    # make sure that the road segment is not an intersection
    if road_segment['is_intersection'] == True:
        return
    # get direction of x and y of the nearest lane
    [delta_x_lane, delta_y_lane] = utils.getDeltaXAndDeltaYOfNearestLane(annotation)
    [delta_orthogonal_lane_x , delta_orthogonal_lane_y] = [delta_y_lane, (-1)*delta_x_lane]


    non_intersection_road_segments_nodes = getNodesOfRoadSegments(road_segment, nusc_map)
    same_direction_nodes = []
    opposite_direction_nodes = []
    for i in range(0, len(non_intersection_road_segments_nodes)-1):
        current_delta_x = non_intersection_road_segments_nodes[i+1]['x'] - non_intersection_road_segments_nodes[i]['x']
        current_delta_y = non_intersection_road_segments_nodes[i+1]['y'] - non_intersection_road_segments_nodes[i]['y']
        if (delta_x_lane * current_delta_x) > 0 and (delta_y_lane * current_delta_y) > 0:
            same_direction_nodes.append(non_intersection_road_segments_nodes[i])
        if (delta_x_lane * current_delta_x) < 0 and (delta_y_lane * current_delta_y) < 0:
            opposite_direction_nodes.append(non_intersection_road_segments_nodes[i])

    lane_to_left_boundary = []
    lane_to_right_boundary = []
    
    start_point_left_lane = [x_center, y_center]
    end_point_left_lane =  [x_center + ((-1) * delta_orthogonal_lane_x) , y_center + ((-1) * delta_orthogonal_lane_y)]
    start_point_right_lane = [x_center, y_center] 
    end_point_right_lane = [x_center + delta_orthogonal_lane_x , y_center + delta_orthogonal_lane_y]
    # we create lane arrays with the points mentioned above
    lane_to_right_boundary = utils.interpolate(start_point_right_lane[0], start_point_right_lane[1], end_point_right_lane[0], end_point_right_lane[1], steps)
    lane_to_left_boundary = utils.interpolate(start_point_left_lane[0], start_point_left_lane[1], end_point_left_lane[0], end_point_left_lane[1], steps)

    # Get average point of both lanes
    # get minimal point of the interpolated thing
    average_x_same_direction = 0
    average_y_same_direction = 0
    for point in same_direction_nodes:
        average_x_same_direction += point['x']
        average_y_same_direction += point['y']
    average_x_same_direction = (average_x_same_direction/len(same_direction_nodes))
    average_y_same_direction = (average_y_same_direction/len(same_direction_nodes))

    average_x_opposite_direction = 0
    average_y_opposite_direction = 0
    for point in opposite_direction_nodes:
        average_x_opposite_direction += point['x']
        average_y_opposite_direction += point['y']
    average_x_opposite_direction = (average_x_opposite_direction/len(opposite_direction_nodes))
    average_y_opposite_direction = (average_y_opposite_direction/len(opposite_direction_nodes))


    # get closest point of lane to average
    closest_point_of_left_lane_to_opposite_average_distance = 999999
    closest_point_of_right_lane_to_same_average_distance = 999999
    closest_point_of_left_lane_to_opposite_average_point = []
    closest_point_of_right_lane_to_same_average_point = []

    for point in lane_to_left_boundary:
        delta_x = average_x_opposite_direction - point['x']
        delta_y = average_y_opposite_direction - point['y']
        distance = math.sqrt(pow(delta_x, 2) + pow(delta_y, 2))
        if distance < closest_point_of_left_lane_to_opposite_average_distance:
            closest_point_of_left_lane_to_opposite_average_distance = distance
            closest_point_of_left_lane_to_opposite_average_point = point

    for point in lane_to_right_boundary:
        delta_x = average_x_same_direction - point['x']
        delta_y = average_y_same_direction - point['y']
        distance = math.sqrt(pow(delta_x, 2) + pow(delta_y, 2))
        if distance < closest_point_of_right_lane_to_same_average_distance:
            closest_point_of_right_lane_to_same_average_distance = distance
            closest_point_of_right_lane_to_same_average_point = point


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




extractPointsOfLaneDirection2(50)