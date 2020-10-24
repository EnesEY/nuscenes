from nuscenes.nuscenes import NuScenes
from pymongo import MongoClient 
from pprint import pprint
import scenes

#setup
nusc = NuScenes(version='v1.0-mini', dataroot='/data/sets/nuscenes', verbose=True)
client=MongoClient('mongodb+srv://enesey:485f6483e3c8666b72fda603a7f87006b83549a54395f2504eb58935f35d00d9@nuscenescluster.jh1vw.mongodb.net/test') 
db = client.nuscenes

def createSampleData(sampleToken):
    #get sample data
    my_sample = nusc.get('sample', sampleToken)
    my_sample_data = my_sample['data']

    for x in my_sample_data:
        sensor = x
        current_data = nusc.get('sample_data', my_sample['data'][sensor])
        db.sample_data.insert_one(current_data)

    print('loaded sample_data for {sampleToken}')

#createSampleData('39586f9d59004284a7114a68825e8eec')