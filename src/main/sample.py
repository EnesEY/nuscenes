from nuscenes.nuscenes import NuScenes
from pymongo import MongoClient 
from pprint import pprint
import scenes
import sample_data
import sample_annotation

#setup
nusc = NuScenes(version='v1.0-mini', dataroot='/data/sets/nuscenes', verbose=True)
client=MongoClient('mongodb+srv://enesey:485f6483e3c8666b72fda603a7f87006b83549a54395f2504eb58935f35d00d9@nuscenescluster.jh1vw.mongodb.net/test') 
db = client.nuscenes

#creates a sample with all its data and annotations
def createSample(sampleToken):
    sample = nusc.get('sample', sampleToken)
    db.sample.insert_one(sample)
    sample_data.createSampleData(sampleToken)
    sample_annotation.createSampleAnnotations(sampleToken)
    print('created sample {sampleToken}')

def getNextSample(sampleToken):
    sample = nusc.get('sample', sampleToken)
    return sample['next']

# creates all samples of a scene
def createSamplesOfScene(scene):
    current_sample_token = scene['first_sample_token']
    last_sample_token = scene['last_sample_token']
    createSample(current_sample_token)

    while current_sample_token != last_sample_token:
        current_sample_token = getNextSample(current_sample_token)
        createSample(current_sample_token)

#createSample('ca9a282c9e77460f8360f564131a8af5')
#createSamplesOfScene('cc8c0bf57f984915a77078b10eb33198')
#createSampleOfDatabase()
#getNextSample('ca9a282c9e77460f8360f564131a8af5')


