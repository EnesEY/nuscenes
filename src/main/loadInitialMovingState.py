from nuscenes.nuscenes import NuScenes
from pymongo import MongoClient 
from pprint import pprint
from nuscenes.map_expansion.map_api import NuScenesMap
from nuscenes.map_expansion import arcline_path_utils

import matplotlib.pyplot as plt
import tqdm
import numpy as np
import parameters

#setup
nusc = NuScenes(version='v1.0-mini', dataroot='/data/sets/nuscenes', verbose=True)
client=MongoClient('mongodb+srv://enesey:485f6483e3c8666b72fda603a7f87006b83549a54395f2504eb58935f35d00d9@nuscenescluster.jh1vw.mongodb.net/test')
nusc_map = NuScenesMap(dataroot='/data/sets/nuscenes', map_name='singapore-onenorth')
db = client.nuscenes
sample_annotation = db["sample_annotation"]
 
def getMovingState(sample_annotation_token):
    velocities = nusc.box_velocity(sample_annotation_token)
    velocity_data = list(velocities)
    moving = False
    if abs(np.average(velocity_data)) > parameters.moving_treshold:
        moving = True
    return moving

def induceMovingStateInSampleAnnotation():
    print('start of inducing moving state in sample annotation')
    for sample_annotation in nusc.sample_annotation:
        movingState = getMovingState(sample_annotation['token'])
        sample_annotation['movingState'] = movingState
    print('end of inducing moving state in sample annotation')
    return nusc.sample_annotation