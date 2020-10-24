from nuscenes.nuscenes import NuScenes
from pymongo import MongoClient 
from pprint import pprint
import scenes
import sample
import instance
import visibility
import sensor

#setup
nusc = NuScenes(version='v1.0-mini', dataroot='/data/sets/nuscenes', verbose=True)
client=MongoClient('mongodb+srv://enesey:485f6483e3c8666b72fda603a7f87006b83549a54395f2504eb58935f35d00d9@nuscenescluster.jh1vw.mongodb.net/test') 
db = client.nuscenes

def loadDataBase():
    scenes.createAllScenes()
    instance.createInstances()
    visibility.createVisibility()
    sensor.createSensor()


    for x in nusc.scene:
        sample.createSamplesOfScene(x)

    print('loaded database')

loadDataBase()