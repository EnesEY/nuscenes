from nuscenes.nuscenes import NuScenes
from pymongo import MongoClient 
from pprint import pprint
from nuscenes.map_expansion.map_api import NuScenesMap
from nuscenes.map_expansion import arcline_path_utils

#setup
nusc = NuScenes(version='v1.0-mini', dataroot='/data/sets/nuscenes', verbose=True)
client=MongoClient('mongodb+srv://enesey:485f6483e3c8666b72fda603a7f87006b83549a54395f2504eb58935f35d00d9@nuscenescluster.jh1vw.mongodb.net/test')

nusc_map_singapore_onenorth = NuScenesMap(dataroot='/data/sets/nuscenes', map_name='singapore-onenorth')
nusc_map_singapore_hollandvillage = NuScenesMap(dataroot='/data/sets/nuscenes', map_name='singapore-hollandvillage')
nusc_map_singapore_queenstown = NuScenesMap(dataroot='/data/sets/nuscenes', map_name='singapore-queenstown')
nusc_map_boston_seaport = NuScenesMap(dataroot='/data/sets/nuscenes', map_name='boston-seaport')

def getMapNameOfSampleAnnotation(annotation):
    sample_token = annotation['sample_token']
    sample = nusc.get('sample', sample_token)
    scene_token = sample['scene_token']
    scene = nusc.get('scene', scene_token)
    log_token = scene['log_token']
    log = nusc.get('log', log_token)
    map_name = log['location']
    return map_name

def load_sample_annotation_with_map_separation():
    print('started loading map-seperated sample_annotation')
    sample_annotations_singapore_onenorth = []
    sample_annotations_singapore_hollandvillage = []
    sample_annotations_singapore_queenstown = []
    sample_annotations_boston_seaport = []
    for sample in nusc.sample_annotation:
        if 'map_name' in sample.keys():
            if sample['map_name'] == 'singapore-onenorth':
                sample_annotations_singapore_onenorth.append(sample)
            if sample['map_name'] == 'singapore-hollandvillage':
                sample_annotations_singapore_hollandvillage.append(sample)
            if sample['map_name'] == 'singapore-queenstown':
                sample_annotations_singapore_queenstown.append(sample)
            if sample['map_name'] == 'boston-seaport':
                sample_annotations_boston_seaport.append(sample)
    print('finished loading map-seperated sample_annotation')
    return [sample_annotations_singapore_onenorth, sample_annotations_singapore_hollandvillage, sample_annotations_singapore_queenstown, sample_annotations_boston_seaport]


def load_map_annotations():
    print('started loading map_name annotation')
    for instance in nusc.instance:
            first_annotation_token = instance['first_annotation_token']
            first_annotation = nusc.get('sample_annotation', first_annotation_token)
            last_annotation_token = instance['last_annotation_token']
            next_annotation_token = first_annotation['next']

            while next_annotation_token != last_annotation_token and next_annotation_token != '':
                current_annotation = nusc.get('sample_annotation', next_annotation_token)
                current_annotation['map_name'] = getMapNameOfSampleAnnotation(current_annotation)
                next_annotation_token = current_annotation['next']
    print('finished loading map_name annotation')

load_map_annotations()
sample_annotations_singapore_onenorth, sample_annotations_singapore_hollandvillage, sample_annotations_singapore_queenstown, sample_annotations_boston_seaport = load_sample_annotation_with_map_separation()

db = client.nuscenes_sample_annotations







def getMap(name):
    if name == 'singapore-onenorth':
        return nusc_map_singapore_onenorth
    if name == 'singapore-hollandvillage':
        return nusc_map_singapore_hollandvillage
    if name == 'singapore-queenstown':
        return nusc_map_singapore_queenstown
    if name == 'boston-seaport':
        return nusc_map_boston_seaport

# method that gets the map that the current sample_annotation is on
def getMapOfSampleAnnotation(annotation):
    sample_token = annotation['sample_token']
    sample = nusc.get('sample', sample_token)
    scene_token = sample['scene_token']
    scene = nusc.get('scene', scene_token)
    log_token = scene['log_token']
    log = nusc.get('log', log_token)
    map_name = log['location']
    nusc_map = getMap(map_name)
    return nusc_map

def getMapLayerTable(nusc_map, layer_type):
    table = ''
    if layer_type == 'road_segment':
        table = nusc_map.road_segment
    if layer_type == 'drivable_area':
        table = nusc_map.drivable_area
    if layer_type == 'lane':
        table = nusc_map.lane
    if layer_type == 'road_block':
        table = nusc_map.road_block
    if layer_type == 'ped_crossing':
        table = nusc_map.ped_crossing
    if layer_type == 'walkway':
        table = nusc_map.walkway
    if layer_type == 'stop_line':
        table = nusc_map.stop_line
    if layer_type == 'carpark_area':
        table = nusc_map.carpark_area

    return table

def getMapLayerOfSampleAnnotatino(annotation, layer_type, nusc_map):
    output = ''

    layers = nusc_map.layers_on_point(annotation['translation'][0], annotation['translation'][1])
    if layers[layer_type] == '':
        return output

    table = getMapLayerTable(nusc_map, layer_type)
    if table == '':
        return output

    for element in table:
        if element['token'] == layers[layer_type]:
            output = element

    return output

# method that gets the lane of an annotation
def getClosestLaneOfSampleAnnotation(annotation, nusc_map):
    output = ''

    closest_lane_token = nusc_map.get_closest_lane(annotation['translation'][0], annotation['translation'][1], radius=2)
    if closest_lane_token == '':
        return output

    output = nusc_map.get_lane(closest_lane_token)

    return output


# method that gets the lane of an annotation
def getIsThisVehicleAndNotEgoVehicle(annotation):
    category_name = annotation['category_name']
    category_name = category_name.split(".") 
    
    if category_name[0] == 'vehicle':
        if category_name[1] != 'ego':
            return True
    else:
        return False










