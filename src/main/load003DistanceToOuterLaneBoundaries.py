from nuscenes.nuscenes import NuScenes
from pymongo import MongoClient 
from pprint import pprint
from nuscenes.map_expansion.map_api import NuScenesMap
from nuscenes.map_expansion import arcline_path_utils

import matplotlib.pyplot as plt
import tqdm
import numpy as np
import load000setup 
import parameters

# loads the attribute movedBefore into every sample_annotation
def loadDistanceToOuterLaneBoundaries():
    print('start adding distance to outer lane boundaries annotations')
    print(load000setup.nusc.instance[0])
    print('finished adding distance to outer lane boundaries annotations')

loadDistanceToOuterLaneBoundaries()