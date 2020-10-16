
#%%
from sklearn.metrics import confusion_matrix, precision_score
from sklearn.model_selection import train_test_split
from sklearn.utils import class_weight

from sklearn.model_selection import StratifiedKFold
from sklearn.model_selection import cross_val_score
from sklearn.preprocessing import LabelEncoder

import tensorflow as tf
import tensorflow.keras as keras
import tensorflow.keras.backend as keras_backend

import matplotlib.pyplot as plt
import pandas as pd
import numpy as np


seed = 7
np.random.seed(seed)

print("Training a DNN with 3 inputs and 1 ouput!")

file_input = 'x_train.csv'
file_input2 = 't_train.csv'

datain  = pd.read_csv(file_input, names = ['terrain','angle'])
dataout = pd.read_csv(file_input2,names = ['velocity'])

x = datain
y = dataout

print(x.head())
print(y.head())


print("---------")
# Split your data for training adn testing
x_train,x_test,y_train,y_test = train_test_split(x,y,test_size=0.15,random_state=0)
print(x_train.shape,y_train.shape,x_test.shape,y_test.shape)
print("---------")

# You can change the size of the layers, activation functions, and input output dimensions
model = keras.models.Sequential()
model.add(keras.layers.Dense(20,activation='relu',input_dim=2,kernel_regularizer=keras.regularizers.l2(0.0000001)))
model.add(keras.layers.Dense(10,activation='relu'))
model.add(keras.layers.Dense(10,activation='relu'))
model.add(keras.layers.Dense(1,activation='relu'))

model.compile(loss='mse',optimizer='adam',metrics=['accuracy'])

print(model.summary())
print("---------")

# Train the model. You can change the number of iterations (epoch)
model_output = model.fit(x_train,y_train,epochs=200,batch_size=10,verbose=1,validation_data=(x_test,y_test))

# Save the model
model.save("NNet_all.h5")

#Evaluate the model
test_loss, test_acc = model.evaluate(x,y)
print('Test accuracy:', test_acc)


y_pred = model.predict(x)
n, bins, patches = plt.hist(y_pred, 200, facecolor='green', alpha=0.75)


rounded = [round(x[0]) for x in y_pred]
y_pred1= np.array(rounded,dtype='int64')

#cm = confusion_matrix(y,y_pred1)
#print('true negative : ' + str(cm[0,0]))
#print('false positive : ' + str(cm[0,1]))
#print('false negative : ' + str(cm[1,0]))
#print('true positive : ' + str(cm[1,1]))

#np.savetxt("confusion_matrix_all.csv", cm, delimiter=",")
np.savetxt("y_train_all.csv", y_pred, delimiter=",")

data_acc = model_output.history["accuracy"]

np.savetxt("data_acc_all.csv", data_acc, delimiter=",")

