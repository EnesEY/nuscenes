import load000setup
import matplotlib.pyplot as plt 
import utils
import load003DistanceToOuterLaneBoundaries

# method that prints out the lane points and the sample points in a view
def visualizeSampleAndLayerNodes(annotation, layer_type, zoomOut):
    x_center, y_center, z_center = annotation['translation']
    my_patch = ((x_center - zoomOut), (y_center - zoomOut), (x_center + zoomOut), (y_center + zoomOut))
    nusc_map = load000setup.getMapOfSampleAnnotation(annotation)
    layer = load000setup.getMapLayerOfSampleAnnotatino(annotation, layer_type)

    poses_x = []
    poses_y = []
    if layer_type == 'road_segment':
        my_node_tokens = layer['exterior_node_tokens']
        for token in my_node_tokens:
            for node in nusc_map.node:
                if node['token'] == token:
                    poses_x.append(node['x'])
                    poses_y.append(node['y'])
    if layer_type == 'lane':
        lane_record = nusc_map.get_lane(layer['token'])
        myNodes = load000setup.arcline_path_utils.discretize_lane(lane_record, resolution_meters=1)
        for value in myNodes:
            poses_x.append(value[0])
            poses_y.append(value[1])

    fig, ax = nusc_map.render_map_patch(my_patch, nusc_map.non_geometric_layers, figsize=(10, 10))
    ax.scatter(x_center, y_center, s=20, c='#33FFFC', alpha=1.0, zorder=2)
    ax.scatter(poses_x, poses_y, s=20, c='k', alpha=1.0, zorder=2)


    # method that plots points on lane divider
def visualizeInterpolatedLaneDividerOfSample(annotation, steps, zoomOut):
    lane = load000setup.getMapLayerOfSampleAnnotatino(annotation, 'lane')
    nusc_map = load000setup.getMapOfSampleAnnotation(annotation)
    x_center, y_center, z_center = annotation['translation']
    my_patch = ((x_center - zoomOut), (y_center - zoomOut), (x_center + zoomOut), (y_center + zoomOut))
    left_segments_node_tokens = []
    left_segments_nodes = []
    right_segments_node_tokens = []
    right_segments_nodes = []

    for segment in lane['left_lane_divider_segments']:
        left_segments_node_tokens.append(segment['node_token'])
    for segment in lane['right_lane_divider_segments']:
        right_segments_node_tokens.append(segment['node_token'])

    for node in nusc_map.node:
        for token in left_segments_node_tokens:
            if node['token'] == token:
                left_segments_nodes.append(node)

    for node in nusc_map.node:
        for token in right_segments_node_tokens:
            if node['token'] == token:
                right_segments_nodes.append(node)

    left_segments_nodes = utils.interpolateBetweenTwoPoints(left_segments_nodes[0], left_segments_nodes[1], steps)
    right_segments_nodes = utils.interpolateBetweenTwoPoints(right_segments_nodes[0], right_segments_nodes[1], steps)

    poses_x_left_segment = []
    poses_y_left_segment = []
    for value in left_segments_nodes:
        poses_x_left_segment.append(value['x'])
        poses_y_left_segment.append(value['y'])

    poses_x_right_segment = []
    poses_y_right_segment = []
    for value in right_segments_nodes:
        poses_x_right_segment.append(value['x'])
        poses_y_right_segment.append(value['y'])

    fig, ax = nusc_map.render_map_patch(my_patch, nusc_map.non_geometric_layers, figsize=(10, 10))
    ax.scatter(x_center, y_center, s=20, c='#33FFFC', alpha=1.0, zorder=2)
    ax.scatter(poses_x_left_segment, poses_y_left_segment, s=20, c='k', alpha=1.0, zorder=2)
    ax.scatter(poses_x_right_segment, poses_y_right_segment, s=20, c='k', alpha=1.0, zorder=2)



# Filter every point that does not on a lane with the same direction as the closest lane
def extractPointsOfLaneDirection(annotation):
    closest_point_of_left_lane_to_opposite_average_distance = ''
    closest_point_of_right_lane_to_same_average_distance = ''

    if annotation is None:
        print('this annotatino has been empty')

    nusc_map = load000setup.getMapOfSampleAnnotation(annotation)
    if nusc_map == '':
        return ''

    # wcheck if this annotation is for a vehicle that is not the ego vehicle
    if load000setup.getIsThisVehicleAndNotEgoVehicle(annotation) == False:
        return ''

    # check if it is on top of a road segment (this feature does not work if this is not the case)
    road_segment = load000setup.getMapLayerOfSampleAnnotatino(annotation, 'road_segment', nusc_map)
    if road_segment == '':
        return ''

    # can't calculate the distance in intersections
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
    non_intersection_road_segments_nodes = load003DistanceToOuterLaneBoundaries.getNodesOfRoadSegments(road_segment, nusc_map)
    [same_direction_nodes, opposite_direction_nodes] = load003DistanceToOuterLaneBoundaries.getOppositeAndSameDirectionNodes(non_intersection_road_segments_nodes, delta_x_lane, delta_y_lane)
    
    # Get average point of both lanes
    [average_x_same_direction, average_y_same_direction] = load003DistanceToOuterLaneBoundaries.getAverageXAndYOfLane(same_direction_nodes)
    [average_x_opposite_direction, average_y_opposite_direction] = load003DistanceToOuterLaneBoundaries.getAverageXAndYOfLane(opposite_direction_nodes)

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
        closest_point_of_left_lane_to_opposite_average_point = load003DistanceToOuterLaneBoundaries.getClosestPointOfLaneToPoint(lane_to_left_boundary, average_x_opposite_direction, average_y_opposite_direction)
        closest_point_of_left_lane_to_opposite_average_distance = load003DistanceToOuterLaneBoundaries.calculateDistanceBetweenTwoNodes(closest_point_of_left_lane_to_opposite_average_point['x'],closest_point_of_left_lane_to_opposite_average_point['y'],x_center, y_center)

    if isRightBoundaryValid == True:
        start_point_right_lane = [x_center, y_center] 
        end_point_right_lane = [x_center + delta_orthogonal_lane_x , y_center + delta_orthogonal_lane_y]
        lane_to_right_boundary = utils.interpolate(start_point_right_lane[0], start_point_right_lane[1], end_point_right_lane[0], end_point_right_lane[1], steps)
        closest_point_of_right_lane_to_same_average_point = load003DistanceToOuterLaneBoundaries.getClosestPointOfLaneToPoint(lane_to_right_boundary, average_x_same_direction, average_y_same_direction)
        closest_point_of_right_lane_to_same_average_distance = load003DistanceToOuterLaneBoundaries.calculateDistanceBetweenTwoNodes(closest_point_of_right_lane_to_same_average_point['x'],closest_point_of_right_lane_to_same_average_point['y'],x_center, y_center)

    annotation['distanceToLeftBoundary'] = closest_point_of_left_lane_to_opposite_average_distance
    annotation['distanceToRightBoundary'] = closest_point_of_right_lane_to_same_average_distance

    # PLOT 
    zoomOut = 10
    my_patch = ((x_center - zoomOut), (y_center - zoomOut), (x_center + zoomOut), (y_center + zoomOut))

    poses_x_same_direction = []
    poses_y_same_direction = []
    poses_lane_to_left_boundary_x = []
    poses_lane_to_left_boundary_y = []
    poses_x_opposite_direction = []
    poses_y_opposite_direction = []
    poses_lane_to_right_boundary_x = []
    poses_lane_to_right_boundary_y = []
    pose_average_x = []
    pose_average_y = []

    if isLeftBoundaryValid == True:
        pose_average_x.append(average_x_same_direction)
        pose_average_y.append(average_y_same_direction)
        pose_average_x.append(closest_point_of_left_lane_to_opposite_average_point['x'])
        pose_average_y.append(closest_point_of_left_lane_to_opposite_average_point['y'])

    if isRightBoundaryValid == True:
        pose_average_x.append(average_x_opposite_direction)
        pose_average_y.append(average_y_opposite_direction)
        pose_average_x.append(closest_point_of_right_lane_to_same_average_point['x'])
        pose_average_y.append(closest_point_of_right_lane_to_same_average_point['y'])

    for value in same_direction_nodes:
        poses_x_same_direction.append(value['x'])
        poses_y_same_direction.append(value['y'])
    for value in opposite_direction_nodes:
        poses_x_opposite_direction.append(value['x'])
        poses_y_opposite_direction.append(value['y'])
    
    if isLeftBoundaryValid == True:
        for value in lane_to_left_boundary:
            poses_lane_to_left_boundary_x.append(value['x'])
            poses_lane_to_left_boundary_y.append(value['y'])
    if isRightBoundaryValid == True:
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

sample = load000setup.nusc.sample_annotation[100]
extractPointsOfLaneDirection(sample)