import math
import os
import numpy as np

features = ["moving-state", "moved-before",
            "will-move", "time-not-moved", "vehicle-volume", "is-on-stop-line", "is-on-car-park-area","is-ego-vehicle",
            "on-intersection", "distance-to-left-boundary", "distance-to-right-boundary","has-instance-in-front"]

information = ["moving-state", "moved-before", "time-not-moved", "vehicle-volume", "is-on-stop-line", "is-on-car-park-area","is-ego-vehicle",
            "on-intersection", "distance-to-left-boundary", "distance-to-right-boundary","has-instance-in-front"]

labels = ["will-move"]

def split_processes(arrayToSplit):
    start_and_end_of_processes = []
    step_length = math.floor(len(arrayToSplit)/os.cpu_count())

    for thread_index in range(0, os.cpu_count()):
        start_and_end_of_processes.append([thread_index*step_length, (thread_index*step_length)+step_length - 1])

    rest = len(arrayToSplit)%os.cpu_count()
    for i in range(0, os.cpu_count()):
        if i > rest:
            start_and_end_of_processes[i][0] += rest
            start_and_end_of_processes[i][1] += rest+1
        else:
            start_and_end_of_processes[i][0] += i
            start_and_end_of_processes[i][1] += i+1

    start_and_end_of_processes[os.cpu_count()-1][1] -= 1
    return start_and_end_of_processes


def get_sample_annotations_of_instance(instance, nusc):
    output = []
    first_annotation_token = instance['first_annotation_token']
    last_annotation_token = instance['last_annotation_token']
    first_annotation = nusc.get('sample_annotation', first_annotation_token)
    output.append(first_annotation)
    next_annotation_token = first_annotation['next']

    while next_annotation_token != last_annotation_token and next_annotation_token != '':
        current_annotation = nusc.get('sample_annotation', next_annotation_token)
        output.append(current_annotation)
        next_annotation_token = current_annotation['next']
    return output


def get_is_vehicle(instance, nusc):
    first_annotation_token = instance['first_annotation_token']
    annotation = nusc.get('sample_annotation', first_annotation_token)
    category_name = annotation['category_name']
    category_name = category_name.split(".") 
    
    if category_name[0] == 'vehicle':
        return True
    else:
        return False


def get_map_name_of_annotation(annotation, nusc):
    sample_token = annotation['sample_token']
    sample = nusc.get('sample', sample_token)
    scene_token = sample['scene_token']
    scene = nusc.get('scene', scene_token)
    log_token = scene['log_token']
    log = nusc.get('log', log_token)
    map_name = log['location']
    return map_name


def get_road_segment_of_annotation(annotation, nusc_map, layers):
    output = ''
    layer_type = 'road_segment'
    if layers[layer_type] == '':
        return output

    table = nusc_map.road_segment
    if table == '':
        return output

    for element in table:
        if element['token'] == layers[layer_type]:
            output = element

    return output


def get_closest_lane_of_annotaion(annotation, nusc_map):
    output = ''

    closest_lane_token = nusc_map.get_closest_lane(annotation['translation'][0], annotation['translation'][1], radius=2)
    if closest_lane_token == '':
        return output

    output = nusc_map.get_arcline_path(closest_lane_token)

    return output


def get_delta_x_and_delta_y_of_lane(annotation, lane):
    start_x = lane[0]['start_pose'][0]
    start_y = lane[0]['start_pose'][1]
    end_x = lane[0]['end_pose'][0]
    end_y = lane[0]['end_pose'][1]

    delta_x = (end_x - start_x)
    delta_y = (end_y - start_y)

    return [delta_x, delta_y]


def get_nodes_of_road_segment(road_segment, nusc_map):
    output = []
    for nodes_token in road_segment['exterior_node_tokens']:
        for node in nusc_map.node:
            if node['token'] == nodes_token:
                output.append(node)
    return output


def get_opposite_and_same_direction_of_node_array(node_array, delta_x_lane, delta_y_lane):
    same_direction_nodes = []
    opposite_direction_nodes = []
    for i in range(0, len(node_array)-1):
        current_delta_x = node_array[i+1]['x'] - node_array[i]['x']
        current_delta_y = node_array[i+1]['y'] - node_array[i]['y']
        if (delta_x_lane * current_delta_x) > 0 and (delta_y_lane * current_delta_y) > 0:
            same_direction_nodes.append(node_array[i])
        if (delta_x_lane * current_delta_x) < 0 and (delta_y_lane * current_delta_y) < 0:
            opposite_direction_nodes.append(node_array[i])
    return [same_direction_nodes, opposite_direction_nodes]


def get_average_point_of_node_array(node_array):
    average_x = 0
    average_y = 0
    for point in node_array:
        average_x += point['x']
        average_y += point['y']
    if len(node_array) != 0:
        average_x = (average_x/len(node_array))
        average_y = (average_y/len(node_array))
    return [average_x, average_y]


def interpolate(x_start, y_start, x_end, y_end, steps):
    output = []
    deltaX = (x_end - x_start)
    deltaY = (y_end - y_start)

    for i in range(0, steps):
        output.append({ 'x': (x_start + ((deltaX/steps)*i)), 'y': (y_start + ((deltaY/steps)*i))})

    return output


def get_nearest_node_of_node_array_to_point(node_array, x_of_point, y_of_point):
    max = 999999
    for point in node_array:
        delta_x = x_of_point - point['x']
        delta_y = y_of_point - point['y']
        distance = math.sqrt(pow(delta_x, 2) + pow(delta_y, 2))
        if distance < max:
            max = distance
            closest_point = point
    return closest_point


def get_distance_between_two_points(x1, y1, x2, y2):
    delta_x = x2 - x1
    delta_y = y2 - y1

    distance = math.sqrt(pow(delta_x, 2) + pow(delta_y, 2))
    return distance


def get_sample_annotations_of_sample(sample, nusc):
    output = []
    sample_token = sample['token']
    for my_annotation in nusc.sample_annotation:
        if my_annotation['sample_token'] == sample_token:
            output.append(my_annotation)
    return output


def get_vehicles_from_sample_array(sample_annotations):
    output = []
    for my_annotation in sample_annotations:
        category_name = my_annotation['category_name']
        category_name = category_name.split(".") 
        if category_name[0] == 'vehicle':
            output.append(my_annotation)
    return output


def empty_value_exists(db, field_name):
    temp_value = db.features.find_one({field_name: { '$exists': False }})
    if temp_value == None:
        return 0
    else:
        return 1


def load_meta_data_and_empty_fields(db, field_name):
    value = {}
    empty_field_exists = (empty_value_exists(db, field_name))
    if empty_field_exists == 1:
        query = {field_name: { '$exists': False }}
        newValue = {"$set": {field_name: -1,}}
        db.features.update_many(query, newValue) 

    value['maximum'] = (db.features.find_one({field_name: {"$exists": True}}, sort=[(field_name, -1)])[field_name])
    value['minimum'] = (db.features.find_one({field_name: {"$exists": True}}, sort=[(field_name, 1)])[field_name])
    return value


# Util methods for classifiers
def preprocess(features, feature_metadata, feature_name):
    max = feature_metadata[feature_name]['maximum']
    min = feature_metadata[feature_name]['minimum']

    for feature in features:
        feature[feature_name] = (feature[feature_name] - min) / (max - min)


def seperate_data_from_labels(data):
    # split data in information and labels
    information_array = []
    feature_array = []
    for point in data:
        temp_info = []
        temp_lab = []
        for info in information:
            temp_info.append(point[info])
        for lab in labels:
            temp_lab.append(point[lab])
        information_array.append(temp_info)
        feature_array.append(temp_lab)
    
    # put data in numpy arrays
    inf_np_arr = np.array(information_array)
    feat_np_arr = np.array(feature_array)

    return inf_np_arr, feat_np_arr
