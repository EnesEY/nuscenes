import math
import os
import numpy as np


def split_processes(arrayToSplit):
    start_and_end_of_processes = []
    step_length = math.floor(len(arrayToSplit)/os.cpu_count())

    for thread_index in range(0, os.cpu_count()):
        start_and_end_of_processes.append([thread_index*step_length, (thread_index*step_length)+step_length - 1])

    rest = len(arrayToSplit)%os.cpu_count()
    for i in range(0, os.cpu_count()):
        if i > rest:
            i = rest
        start_and_end_of_processes[i][0] += i
        start_and_end_of_processes[i][1] += i+1
    start_and_end_of_processes[os.cpu_count()-1][1] -= 1
    
    return start_and_end_of_processes

def get_sample_annotations_of_instance(instance, nusc):
    output = []
    first_annotation_token = instance['first_annotation_token']
    last_annotation_token = instance['last_annotation_token']
    first_annotation = nusc.get('sample_annotation', first_annotation_token)
    output.append(first_annotation)
    next_annotation_token = first_annotation['next']

    while next_annotation_token != last_annotation_token and next_annotation_token != '':
        current_annotation = nusc.get('sample_annotation', next_annotation_token)
        output.append(current_annotation)
        next_annotation_token = current_annotation['next']
    return output

def get_vehicle_and_not_ego_vehicle(instance, nusc):
    first_annotation_token = instance['first_annotation_token']
    annotation = nusc.get('sample_annotation', first_annotation_token)
    category_name = annotation['category_name']
    category_name = category_name.split(".") 
    
    if category_name[0] == 'vehicle':
        if category_name[1] != 'ego':
            return True
    else:
        return False