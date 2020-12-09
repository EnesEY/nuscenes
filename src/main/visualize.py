import matplotlib.pyplot as plt 

# Filter every point that does not on a lane with the same direction as the closest lane
def visualize_node_array(nusc_map, node_array, x_center, y_center, zoom_out):
    poses_x = []
    poses_y = []
    my_patch = ((x_center - zoom_out), (y_center - zoom_out), (x_center + zoom_out), (y_center + zoom_out))

    for value in node_array:
        poses_x.append(value['x'])
        poses_y.append(value['y'])

    print('we are plotting')
    fig, ax = nusc_map.render_map_patch(my_patch, nusc_map.non_geometric_layers, figsize=(10, 10))
    ax.scatter(x_center, y_center, s=20, c='#FF0000', alpha=1.0, zorder=2)
    ax.scatter(poses_x, poses_y, s=20, c='#00FF00', alpha=1.0, zorder=2)

