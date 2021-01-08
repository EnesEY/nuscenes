from nuscenes.nuscenes import NuScenes
from pymongo import MongoClient 
from pprint import pprint
from nuscenes.map_expansion.map_api import NuScenesMap
from nuscenes.map_expansion import arcline_path_utils
from nuscenes.map_expansion.bitmap import BitMap

import matplotlib.pyplot as plt
import tqdm
import numpy as np

nusc_map = NuScenesMap(dataroot='/data/sets/nuscenes', map_name='singapore-onenorth')
nusc = NuScenes(version='v1.0-mini', dataroot='/data/sets/nuscenes', verbose=True)

print(nusc.attribute[0])

fig, ax = nusc_map.render_layers(nusc_map.non_geometric_layers, figsize=1)
fig.savefig('full_figure.png', dpi=fig.dpi, bbox_inches='tight')
