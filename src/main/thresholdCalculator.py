from nuscenes.nuscenes import NuScenes
from pymongo import MongoClient 
from pprint import pprint
from nuscenes.map_expansion.map_api import NuScenesMap
from nuscenes.map_expansion import arcline_path_utils
import annotations_from_instance
import annotations_from_sample
import matplotlib.pyplot as plt
from tqdm import tqdm
import matplotlib
import numpy as np


nusc = NuScenes(version='v1.0-mini', dataroot='/data/sets/nuscenes', verbose=True)
client=MongoClient('mongodb+srv://enesey:485f6483e3c8666b72fda603a7f87006b83549a54395f2504eb58935f35d00d9@nuscenescluster.jh1vw.mongodb.net/test')
db = client.nuscenes


# loads all the sample_annotations that are motionless and are moving
motionless = []
moving = []
for attribute in nusc.sample_annotation:
    if attribute['attribute_tokens'] == '' or attribute['attribute_tokens'] == []:
        continue
    if attribute['attribute_tokens'][0] == "c3246a1e22a14fcb878aa61e69ae3329" or attribute['attribute_tokens'][0] == "58aa28b1c2a54dc88e169808c07331e3":
        motionless.append(attribute)
    if attribute['attribute_tokens'][0] == "cb5118da1ab342aa947717dc53544259":
        moving.append(attribute)


# finds the maximum speed of motionless annotations
maximum = 0
current_maximum_motionless = ''
for annotation in motionless:
    velocities = list(nusc.box_velocity(annotation['token']))
    if abs(np.average(velocities)) > maximum:
        maximum = abs(np.average(velocities))
        current_maximum_motionless = annotation


minimum = 9999999.9
current_minimum_moving = ''
for annotation in moving:
    velocities = list(nusc.box_velocity(annotation['token']))
    if abs(np.average(velocities)) < minimum:
        minimum = abs(np.average(velocities))
        current_minimum_moving = annotation


# Det curve to find the optimal
def number_wrong_calculator(threshold, print_results):
    wrong_assumed_moving = []
    wrong_assumed_motionless = []
    for annotation in motionless:
        velocities = list(nusc.box_velocity(annotation['token']))
        if abs(np.average(velocities)) > threshold:
            wrong_assumed_moving.append(annotation)

    for annotation in moving:
        velocities = list(nusc.box_velocity(annotation['token']))
        if abs(np.average(velocities)) < threshold:
            wrong_assumed_motionless.append(annotation)

    total_wrong = len(wrong_assumed_moving) + len(wrong_assumed_motionless)

    if print_results == True:
        total_annotations = len(motionless) + len(moving)
        total_moving = len(moving)
        total_motionless = len(motionless)
        total_wrong = len(wrong_assumed_moving) + len(wrong_assumed_motionless)
        wrong_moving = len(wrong_assumed_moving)
        wrong_motionless = len(wrong_assumed_motionless)
        accuracy = ((total_annotations - total_wrong)/total_annotations) * 100
        print(f'total_annotations: {total_annotations}')
        print(f'total_moving: {total_moving}')
        print(f'total_motionless: {total_motionless}')
        print(f'total_wrong: {total_wrong}')
        print(f'wrong_moving: {wrong_moving}')
        print(f'wrong_motionless: {wrong_motionless}')
        print(f'accuracy in %: {accuracy}')
    
    return total_wrong


# plotting
x = []
y = []
steps = np.arange(0.0, 2.0, 0.01)
for i in steps:
    x.append(i)
    y.append(number_wrong_calculator(i, False))

minimum = 9999999.9
minimum_index = 0
index = 0
for value in y:
    if value < minimum:
        minimum = value
        minimum_index = index
    index += 1

best_threshold = x[minimum_index]

print(f'minimum errors is: {minimum}')
print(f'best_threshold is: {best_threshold}')

fig, ax = plt.subplots()
ax.plot(x, y)

ax.set(xlabel='moving-threshold', ylabel='wrongly classified moving states',
       title='plot to determine moving-threshold')
ax.grid()

plt.show()

number_wrong_calculator(best_threshold, True)

