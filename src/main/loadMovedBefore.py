from nuscenes.nuscenes import NuScenes
from pymongo import MongoClient 
from pprint import pprint
from nuscenes.map_expansion.map_api import NuScenesMap
from nuscenes.map_expansion import arcline_path_utils

import matplotlib.pyplot as plt
import tqdm
import numpy as np
import loadInitialMovingState 
import parameters

# loads the attribute movedBefore into every sample_annotation
def loadMovedBefore():
    print('start adding moved before annotation')
    # load moving state first
    loadInitialMovingState.induceMovingStateInSampleAnnotation()
    for instance in loadInitialMovingState.nusc.instance:
        # information weather this instance has moved or not
        movedBefore = False
        # first and last annotation token of instance
        first_annotation_token = instance['first_annotation_token']
        last_annotation_token = instance['last_annotation_token']
        # first annotation of instance
        first_annotation = loadInitialMovingState.nusc.get('sample_annotation', first_annotation_token)
        # next annotation token from first_annotation
        next_annotation_token = first_annotation['next']

        movedBefore = updateMovedBefore(movedBefore, first_annotation)
        while next_annotation_token != last_annotation_token and next_annotation_token != '':
            current_annotation = loadInitialMovingState.nusc.get('sample_annotation', next_annotation_token)
            movedBefore = updateMovedBefore(movedBefore, current_annotation)
            current_annotation['movedBefore'] = movedBefore
            next_annotation_token = current_annotation['next']
    print('finished loading moved before annotation')

# gets the current moved before state and the annotation
# looks if the moved before state has changed and inputs movedBefore into the annotation also
def updateMovedBefore(movedBefore, annotation):
    if movedBefore == True:
        movedBefore = True
    if movedBefore == False and annotation['movingState'] == True:
        movedBefore = True
    return movedBefore

loadMovedBefore()
