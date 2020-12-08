from nuscenes.nuscenes import NuScenes
from pymongo import MongoClient 
from pprint import pprint
from nuscenes.map_expansion.map_api import NuScenesMap
from nuscenes.map_expansion import arcline_path_utils

import matplotlib.pyplot as plt
import tqdm
import numpy as np
import load000setup
import load001InitialMovingState 
import load002MovedBefore
import load003DistanceToOuterLaneBoundaries


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
    # #load sample_annotation




    loadSampleAnnotations()



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


def loadSampleAnnotations():
    print('started loading sample_annotation')
    #001
    load001InitialMovingState.induceMovingStateInSampleAnnotation()
    #002
    load002MovedBefore.loadMovementAttributes()
    #003
    load003DistanceToOuterLaneBoundaries.loadDistanceToOuterLaneBoundaries(load000setup.sample_annotations_boston_seaport, 0)
    load003DistanceToOuterLaneBoundaries.loadDistanceToOuterLaneBoundaries(load000setup.sample_annotations_singapore_hollandvillage, 1)
    load003DistanceToOuterLaneBoundaries.loadDistanceToOuterLaneBoundaries(load000setup.sample_annotations_singapore_onenorth, 2)
    load003DistanceToOuterLaneBoundaries.loadDistanceToOuterLaneBoundaries(load000setup.sample_annotations_singapore_queenstown, 3)

    # load000setup.db.boston_seaport.insert_many(load000setup.sample_annotations_boston_seaport)
    # load000setup.db.singapore_hollandvillage.insert_many(load000setup.sample_annotations_singapore_hollandvillage)
    # load000setup.db.singapore_onenorth.insert_many(load000setup.sample_annotations_singapore_onenorth)
    # load000setup.db.singapore_queenstown.insert_many(load000setup.sample_annotations_singapore_queenstown)
    print('ended loading sample_annotation')



loadDatabase()
#loadSampleAnnotations()