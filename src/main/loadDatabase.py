from nuscenes.nuscenes import NuScenes
from pymongo import MongoClient 
from pprint import pprint
from nuscenes.map_expansion.map_api import NuScenesMap
from nuscenes.map_expansion import arcline_path_utils

import matplotlib.pyplot as plt
import tqdm
import numpy as np
import load_annotations
import utils_annotations as utils

nusc = NuScenes(version='v1.0-mini', dataroot='/data/sets/nuscenes', verbose=True)
dbPath = 'mongodb+srv://enesey:485f6483e3c8666b72fda603a7f87006b83549a54395f2504eb58935f35d00d9@nuscenescluster.jh1vw.mongodb.net/test'
client=MongoClient(dbPath)
db = client.nuscenes

def loadDatabase():
    # #load scene
    # db.scene.insert_many(nusc.scene)
    # print('loaded scene complete')
    # #load sample
    # db.sample.insert_many(nusc.sample)
    # print('loaded sample complete')
    # #load sample_data
    # db.sample_data.insert_many(nusc.sample_data)
    # print('loaded sample_data complete')
    # #load ego_pose
    # db.ego_pose.insert_many(nusc.ego_pose)
    # print('loaded ego_pose complete')
    # #load sample_annotation (unmodified)
    # db.sample_annotation.insert_many(nusc.sample_annotation)
    # print('loaded unmodified sample annotations')
    # #load sample_annotation (unmodified)
    load_annotations.load_features(dbPath)
    print('loaded modified sample annotations')
    # #load attribute
    # db.attribute.insert_many(nusc.attribute)
    # print('loaded attribute complete')
    # #load visibility
    # db.visibility.insert_many(nusc.visibility)
    # print('loaded visibility complete')
    # #load instance
    # db.instance.insert_many(nusc.instance)
    # print('loaded instance complete')
    # #load category
    # db.category.insert_many(nusc.category)
    # print('loaded category complete')
    # #load log
    # db.log.insert_many(nusc.log)
    # print('loaded log complete')
    # #load calibrated_sensor
    # db.calibrated_sensor.insert_many(nusc.calibrated_sensor)
    # print('loaded calibrated_sensor complete')
    # #load sensor
    # db.sensor.insert_many(nusc.sensor)
    # print('loaded sensor complete')
    load_preprocessed_features()


def load_preprocessed_features():
    value = {}
    features = list(db.features.find())

    # load min, max and add empty rows as -1
    for field_name in load_annotations.features:
        value[field_name] = utils.load_meta_data_and_empty_fields(db, field_name)
    
    # normalize data (put the data in ranges of 0 to 1)
    for feature_name in (load_annotations.features):
        utils.preprocess(features, value, feature_name)
    

    # insert in databse
    db.preprocessed_features.insert_many(features)
    print('load preprocessed features done')


loadDatabase()