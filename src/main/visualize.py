import load000setup
import matplotlib.pyplot as plt 
import utils

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
