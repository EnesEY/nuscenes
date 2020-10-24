from nuscenes.nuscenes import NuScenes
from pymongo import MongoClient 
from pprint import pprint
import scenes
import attributes

#setup
nusc = NuScenes(version='v1.0-mini', dataroot='/data/sets/nuscenes', verbose=True)
client=MongoClient('mongodb+srv://enesey:485f6483e3c8666b72fda603a7f87006b83549a54395f2504eb58935f35d00d9@nuscenescluster.jh1vw.mongodb.net/test') 
db = client.nuscenes

def createSampleAnnotations(sampleToken):
    my_sample = nusc.get('sample', sampleToken)
    my_sample_anns = my_sample['anns']

    for x in my_sample_anns:
        token = x
        current_data = nusc.get('sample_annotation', token)
        db.sample_annotation.insert_one(current_data)
        attributes.createAttributes(current_data)

    print('loaded sample_annotatin for {sampleToken}')



#createSampleAnnotations('39586f9d59004284a7114a68825e8eec')