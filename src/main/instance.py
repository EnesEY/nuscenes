from nuscenes.nuscenes import NuScenes
from pymongo import MongoClient 
from pprint import pprint
import scenes

#setup
nusc = NuScenes(version='v1.0-mini', dataroot='/data/sets/nuscenes', verbose=True)
client=MongoClient('mongodb+srv://enesey:485f6483e3c8666b72fda603a7f87006b83549a54395f2504eb58935f35d00d9@nuscenescluster.jh1vw.mongodb.net/test') 
db = client.nuscenes

def createInstances():
    for x in nusc.instance:
        db.instance.insert_one(x)

    outputString = 'loaded instances'
    print(outputString)

createInstances()