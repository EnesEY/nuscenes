from nuscenes.nuscenes import NuScenes
from pymongo import MongoClient 
from pprint import pprint
from nuscenes.map_expansion.map_api import NuScenesMap
from nuscenes.map_expansion import arcline_path_utils

#setup
nusc = NuScenes(version='v1.0-mini', dataroot='/data/sets/nuscenes', verbose=True)
client=MongoClient('mongodb+srv://enesey:485f6483e3c8666b72fda603a7f87006b83549a54395f2504eb58935f35d00d9@nuscenescluster.jh1vw.mongodb.net/test')
nusc_map_singapore = NuScenesMap(dataroot='/data/sets/nuscenes', map_name='singapore-onenorth')
db = client.nuscenes

def getMap(name):
    if name == 'singapore-onenorth':
        return nusc_map_singapore

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

def getMapLayerOfSampleAnnotatino(annotation, layer_type):
    nusc_map = getMapOfSampleAnnotation(annotation)
    layers = nusc_map.layers_on_point(annotation['translation'][0], annotation['translation'][1])
    table = getMapLayerTable(nusc_map, layer_type)

    for element in table:
        if element['token'] == layers[layer_type]:
            output = element

    return output

# method that gets the lane of an annotation
def getClosestLaneOfSampleAnnotation(annotation):
    nusc_map = getMapOfSampleAnnotation(annotation)
    closest_lane_token = nusc_map.get_closest_lane(annotation['translation'][0], annotation['translation'][1], radius=2)
    lane_record = nusc_map.get_lane(closest_lane_token)

    return lane_record
