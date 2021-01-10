# TensorFlow and tf.keras
import tensorflow as tf
import numpy as np
import matplotlib.pyplot as plt
from pymongo import MongoClient 
from tqdm import tqdm
import utils_annotations as utils
import load_annotations


# get database and preprocessed features
client=MongoClient('mongodb+srv://enesey:485f6483e3c8666b72fda603a7f87006b83549a54395f2504eb58935f35d00d9@nuscenescluster.jh1vw.mongodb.net/test')
db = client.nuscenes
preprocessed_features = list(db.preprocessed_features.find())

# split 80 / 20 for training and validation
split_number = round(len(preprocessed_features) * 0.8)
train_data = []
for index in tqdm(range(0, split_number), desc='load training data'):
    train_data.append(preprocessed_features[index])

val_data = []
for index in tqdm(range(split_number, len(preprocessed_features) -1), desc='load val data'):
    val_data.append(preprocessed_features[index])

# seperate info and labels for training and validation data and put them in numpy arrays
train_information , train_labels = utils.seperate_data_from_labels(train_data)
val_information, val_labels = utils.seperate_data_from_labels(val_data)

# creatae and train neural network  
model = tf.keras.Sequential([
    tf.keras.layers.Dense(11),
    tf.keras.layers.Dense(8, activation='relu'),
    tf.keras.layers.Dense(2, activation='relu'),
    tf.keras.layers.Dense(2, activation='softmax')
])

model.compile(optimizer='adam',
              loss=tf.keras.losses.SparseCategoricalCrossentropy(from_logits=True),
              metrics=['accuracy'])

model.fit(train_information, train_labels, epochs=5)

model.summary()

# validate and show results
test_loss, test_acc = model.evaluate(val_information,  val_labels, verbose=2)
print('\nTest accuracy:', test_acc)

# save the model
model.save('classifiers/neural_network.h5')

# # load the model 
# new_model = tf.keras.models.load_model('classifiers/neural_network.h5')
# model.summary()
