from nuscenes.nuscenes import NuScenes
from pymongo import MongoClient 
from pprint import pprint
import scenes

#setup
nusc = NuScenes(version='v1.0-mini', dataroot='/data/sets/nuscenes', verbose=True)
client=MongoClient('mongodb+srv://enesey:485f6483e3c8666b72fda603a7f87006b83549a54395f2504eb58935f35d00d9@nuscenescluster.jh1vw.mongodb.net/test') 
db = client.nuscenes

#does not work for some reason

# def loadAllAttributesFromExistingAnnotations():
#     for annotation in db.sample_annotation.find():
#         for token in annotation['attribute_tokens']:
#             current_attr = nusc.get('attribute', token)
#             db.attribute.insert_one(current_attr)
#     print('attributes have been loaded')


def createAttributes(sample_annotation):
    tokensArray = sample_annotation['attribute_tokens']
    for x in tokensArray:
        current_attr = nusc.get('attribute', x)
        db.attribute.insert_one(current_attr)

createAttributes('ef63a697930c4b20a6b9791f423351da')
#loadAllAttributesFromExistingAnnotations()