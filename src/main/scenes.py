from nuscenes.nuscenes import NuScenes
from pymongo import MongoClient 
from pprint import pprint

#setup
nusc = NuScenes(version='v1.0-mini', dataroot='/data/sets/nuscenes', verbose=True)
client=MongoClient('mongodb+srv://enesey:485f6483e3c8666b72fda603a7f87006b83549a54395f2504eb58935f35d00d9@nuscenescluster.jh1vw.mongodb.net/test') 
db = client.nuscenes
myCol = db["scenes"]

def createScene(sceneIndex):
    my_scene = nusc.scene[sceneIndex]
    db.scenes.insert_one(my_scene)

def createAllScenes():
    for x in nusc.scene:
        db.scenes.insert_one(x)

def getScene(myToken):
    return db.scenes.find_one({"token": myToken})

#createScene(0)
#createAllScenes()
#getScene('cc8c0bf57f984915a77078b10eb33198')
