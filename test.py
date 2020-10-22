from nuscenes.nuscenes import NuScenes

nusc = NuScenes(version='v1.0-mini', dataroot='/data/sets/nuscenes', verbose=True)

my_scene = nusc.scene[0]
my_scene

first_sample_token = my_scene['first_sample_token']

my_sample = nusc.get('sample', first_sample_token)
my_sample

sensor = 'CAM_FRONT'
cam_front_data = nusc.get('sample_data', my_sample['data'][sensor])
cam_front_data

my_annotation_token = my_sample['anns'][18]
my_annotation_metadata =  nusc.get('sample_annotation', my_annotation_token)
my_annotation_metadata

print(my_annotation_metadata)

