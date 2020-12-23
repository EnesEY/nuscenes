from nuscenes.nuscenes import NuScenes
import tensorflow as tf
from tensorflow import keras
import numpy as np
import matplotlib.pyplot as plt
from pymongo import MongoClient 
import numpy as np


# # load data
# data = keras.datasets.fashion_mnist

# (train_images, train_labels), (test_images, test_labels) = data.load_data()

# class_names = ['T-shirt/top', 'Trouser' , 'Pullover', 'Dress', 'Coat', 
#                 'Sandal', 'Shirt', 'Sneaker', 'Bag', 'Ankle boot']

# print(train_images)

# train_images = train_images/255.0
# test_images = test_images/255.0

# model = keras.Sequential([
#     keras.layers.Flatten(input_shape=(28,28)),
#     keras.layers.Dense(128, activation="relu"),
#     keras.layers.Dense(10, activation="softmax")
# ])

# model.compile(optimizer="adam", loss="sparse_categorical_crossentropy", metrics=["accuracy"])

# model.fit(train_images, train_labels, epochs=10)

# test_loss, test_acc = model.evaluate(test_images, test_labels)

# print('tested acc:', test_acc)











def split_list(a_list):
    half = (len(a_list)//10)*8
    return a_list[:half], a_list[half:]

def boolean_preprocessor(my_boolean):
    if my_boolean == True:
        return 1
    if my_boolean == False:
        return 0.5
    if my_boolean == '':
        return 0

def time_not_moved_preprocessor(time_not_moved):
    if time_not_moved == '':
        return 0 
    return (time_not_moved/40)

def vehicle_volume_preprocessor(vehicle_volume):
    if vehicle_volume == '':
        return 0
    return (vehicle_volume/200)

def distance_to_boundary_preprocessor(distance_to_boundary):
    if distance_to_boundary == '':
        return 0 
    return (distance_to_boundary/25)


#Load Database and split it in training and testing
client=MongoClient('mongodb+srv://enesey:485f6483e3c8666b72fda603a7f87006b83549a54395f2504eb58935f35d00d9@nuscenescluster.jh1vw.mongodb.net/test')
db = client.nuscenes
collection=db['sample_annotations']

cursor_list = list(collection.find())
train_set, test_set = split_list(cursor_list)

# split those into features and labels
train_set_features = []
train_set_labels = []

test_set_features = []
test_set_labels = []

for item in train_set:
    input_features = []
    input_labels = []
    # load features of train set
    input_features.append(boolean_preprocessor(item['movingState']))
    input_features.append(boolean_preprocessor(item['movedBefore']))
    input_features.append( time_not_moved_preprocessor(item['timeNotMoved']))
    input_features.append( vehicle_volume_preprocessor(item['vehicleVolume']))
    input_features.append( boolean_preprocessor(item['isOnStopLine']))
    input_features.append( boolean_preprocessor(item['isOnCarparkArea']))
    input_features.append( boolean_preprocessor(item['isEgoVehicle']))
    input_features.append( boolean_preprocessor(item['onIntersection']))
    input_features.append( boolean_preprocessor(item['hasInstanceInFront']))
    input_features.append( distance_to_boundary_preprocessor(item['distanceToLeftBoundary']))
    input_features.append( distance_to_boundary_preprocessor(item['distanceToRightBoundary']))
    # load labels of train set
    input_labels.append( boolean_preprocessor(item['willMove']))
    # append
    train_set_features.append(input_features)
    train_set_labels.append(input_labels)

for item in test_set:
    input_features = []
    input_labels = []
    # load features of train set
    input_features.append(boolean_preprocessor(item['movingState']))
    input_features.append(boolean_preprocessor(item['movedBefore']))
    input_features.append( time_not_moved_preprocessor(item['timeNotMoved']))
    input_features.append( vehicle_volume_preprocessor(item['vehicleVolume']))
    input_features.append( boolean_preprocessor(item['isOnStopLine']))
    input_features.append( boolean_preprocessor(item['isOnCarparkArea']))
    input_features.append( boolean_preprocessor(item['isEgoVehicle']))
    input_features.append( boolean_preprocessor(item['onIntersection']))
    input_features.append( boolean_preprocessor(item['hasInstanceInFront']))
    input_features.append( distance_to_boundary_preprocessor(item['distanceToLeftBoundary']))
    input_features.append( distance_to_boundary_preprocessor(item['distanceToRightBoundary']))
    # load labels of train set
    input_labels.append( boolean_preprocessor(item['willMove']))
    # append
    test_set_features.append(input_features)
    test_set_labels.append(input_labels)

class_names = ['has_no_intention_to_move', 'has_intention_to_move']

# make them into numpy array
train_set_features_arr = np.array(train_set_features)
train_set_labels_arr = np.array(train_set_labels)

test_set_features_arr = np.array(test_set_features)
test_set_labels_arr = np.array(test_set_labels)

#Get shapes of these arrays as a test
print(f'shape of train_set_features_arr: {train_set_features_arr.shape}')
print(f'shape of train_set_labels_arr: {train_set_labels_arr.shape}')
print(f'shape of test_set_features_arr: {test_set_features_arr.shape}')
print(f'shape of test_set_labels_arr: {test_set_labels_arr.shape}')

model = keras.Sequential([
    keras.layers.Dense(11),
    keras.layers.Dense(2, activation="relu"),
    keras.layers.Dense(2, activation="softmax")
])

model.compile(optimizer="adam", loss="sparse_categorical_crossentropy", metrics=["accuracy"])

model.fit(train_set_features, train_set_labels, epochs=2)

test_loss, test_acc = model.evaluate(test_set_features, test_set_labels)


