from nuscenes.nuscenes import NuScenes
from pymongo import MongoClient 
from pprint import pprint
from nuscenes.map_expansion.map_api import NuScenesMap
from nuscenes.map_expansion import arcline_path_utils

import matplotlib.pyplot as plt
import tqdm
import numpy as np
import parameters
import load000setup
 
def getMovingState(sample_annotation_token):
    velocities = load000setup.nusc.box_velocity(sample_annotation_token)
    velocity_data = list(velocities)
    moving = False
    if abs(np.average(velocity_data)) > parameters.moving_treshold:
        moving = True
    return moving

def induceMovingStateInSampleAnnotation():
    print('start of inducing moving state in sample annotation')
    for sample_annotation in load000setup.nusc.sample_annotation:
        movingState = getMovingState(sample_annotation['token'])
        sample_annotation['movingState'] = movingState
    print('end of inducing moving state in sample annotation')