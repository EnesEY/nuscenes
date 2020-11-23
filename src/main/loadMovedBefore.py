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
 
 # gets the information weather this instance has moved before or not
def getMovedBefore(sample_annotation):
    # get first sample annotation of an instance and look if it is moving
    # input the state into the sample annotation
    movedBefore = False
    for instance in nusc.instance.find():

        first_annotation_token = instance['first_annotation_token']
        first_annotation = nusc.get('sample_annotation', first_annotation_token)
        movedBefore = first_annotation['movingState']

        last_annotation_token = instance['last_annotation_token']
        next_annotation_token = instance['next']
        while next_annotation_token != last_annotation_token:
            current_annotation = nusc.get('sample_annotation', next_annotation_token)
            movedBefore = current_annotation['']
            last_annotation = nusc.get('sample_annotation', last_annotation_token)
            first_annotation = nusc.get('sample_annotation', first_annotation_token)

        next_annotation_token = instance['next']
        
        movedBefore = first_annotation['movedBefore']

    # if it didin't move iterate over all the sample annotations and see if it moves

    return False



# this method has to come after the induceMovingStateInSampleAnnotation() method when loading sample_annotations
# def induceMovedBeforeInSampleAnnotations():
#     print('start of inducing moved before in sample annotation')
#     for sample_annotation in nusc.sample_annotation:
#         movedBefore = getMovedBefore(sample_annotation)
#         sample_annotation['movedBefore'] = movedBefore
#     print('end of inducing moved before in sample annotation')
#     return nusc.sample_annotation