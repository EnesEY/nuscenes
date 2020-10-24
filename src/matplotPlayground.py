from nuscenes.nuscenes import NuScenes
from pymongo import MongoClient 
from pprint import pprint
from nuscenes.map_expansion.map_api import NuScenesMap
from nuscenes.map_expansion import arcline_path_utils

import matplotlib.pyplot as plt
import tqdm
import numpy as np

#setup
nusc = NuScenes(version='v1.0-mini', dataroot='/data/sets/nuscenes', verbose=True)
client=MongoClient('mongodb+srv://enesey:485f6483e3c8666b72fda603a7f87006b83549a54395f2504eb58935f35d00d9@nuscenescluster.jh1vw.mongodb.net/test')
nusc_map = NuScenesMap(dataroot='/data/sets/nuscenes', map_name='singapore-onenorth')
db = client.nuscenes

def matplot():
    db.sample_annotation.find_one({"token": '70aecbe9b64f4722ab3c230391a3beb8'})
    nusc.render_annotation('70aecbe9b64f4722ab3c230391a3beb8')
    fig, ax = nusc_map.render_layers(nusc_map.non_geometric_layers, figsize=1)


matplot()