import csv
import cv2
import numpy as np
from sklearn.utils import shuffle
from sklearn.model_selection import train_test_split

def joinSourceList(datapath, pathList, isMyData):
	lines = []
	with open(datapath,'r') as csvFile:
		next(csvFile)
		reader = csv.reader(csvFile)
		for line in reader:
			lines.append(line)

		for line in lines:
			if isMyData == 0:
				centerPath = 'data/' + line[0]
				leftPath = 'data/' + line[1][1:]
				rightPath = 'data/' + line[2][1:]
			elif isMyData == 1:
				rootPath = datapath.split('/')[0]

				sourcePath = line[0]
				centerName = sourcePath.split('/')[-1]
				centerPath = rootPath + '/IMG/' + centerName

				sourcePath = line[1]
				leftName = sourcePath.split('/')[-1]
				leftPath = rootPath + '/IMG/' + leftName

				sourcePath = line[2]
				rightName = sourcePath.split('/')[-1]
				rightPath = rootPath + '/IMG/' + rightName

			centerVal = line[3]
			pathList.extend([[centerPath, leftPath, rightPath, centerVal]])

	return pathList

def generator3Cams(pathList, batch_size):
	num_samples = len(pathList)
	# Loop forever so the generator never terminates
	while (1):
		pathList = shuffle(pathList)

		for offset in range(0, num_samples, batch_size):
			if offset+batch_size > num_samples-1:
				end = num_samples-1
				batch_samples = pathList[offset:end]
				#print(offset)
				#print(end)
				
			else:
				batch_samples = pathList[offset:offset+batch_size]

			carImages = []
			steerMeas = []
			for batch_sample in batch_samples:
				centerImage = cv2.imread(batch_sample[0])
				centerImage = centerImage[50:130, :]
				centerFlip = cv2.flip(centerImage, 1)

				leftImage = cv2.imread(batch_sample[1])
				leftImage = leftImage[50:130, :]
				leftFlip = cv2.flip(leftImage, 1)

				rightImage = cv2.imread(batch_sample[2])
				rightImage = rightImage[50:130, :]
				rightFlip = cv2.flip(rightImage, 1)

				carImages.extend([centerImage, centerFlip, leftImage, leftFlip, rightImage, rightFlip])
				#carImages.extend([centerImage, centerFlip, leftImage, rightImage])

				lrBias = 0.12 # Tuning parameter
				centerMeas = float(batch_sample[3])
				centerFlipMeas = -centerMeas
				leftMeas = centerMeas + lrBias
				leftFlipMeas = -leftMeas
				rightMeas = centerMeas - lrBias
				rightFlipMeas = -rightMeas

				steerMeas.extend([centerMeas, centerFlipMeas, leftMeas, leftFlipMeas, rightMeas, rightFlipMeas])
				#steerMeas.extend([centerMeas, centerFlipMeas, leftMeas, rightMeas])

			# trim image to only see section with road
			X_train = np.array(carImages)
			y_train = np.array(steerMeas)
			yield shuffle(X_train, y_train)


### Open and read data file
pathList = []
carImages = []
steerMeas = []

pathList = joinSourceList('data/driving_log.csv', pathList, isMyData = 0)
pathList = joinSourceList('myData_C1_CCW/driving_log.csv', pathList, isMyData = 1)
pathList = joinSourceList('myData_C1_CW/driving_log.csv', pathList, isMyData = 1)
pathList = joinSourceList('myData_C1_CCW2/driving_log.csv', pathList, isMyData = 1)
pathList = joinSourceList('myData_C1_CCW3/driving_log.csv', pathList, isMyData = 1)
pathList = joinSourceList('myData_C2_1/driving_log.csv', pathList, isMyData = 1)
pathList = joinSourceList('myData_C2_2/driving_log.csv', pathList, isMyData = 1)
pathList = joinSourceList('myData_C2_4/driving_log.csv', pathList, isMyData = 1)
pathList = joinSourceList('myData_large2/driving_log.csv', pathList, isMyData = 1)
pathList = joinSourceList('myData_large3/driving_log.csv', pathList, isMyData = 1)
pathList = joinSourceList('myData_large4/driving_log.csv', pathList, isMyData = 1)
print('Data loading complete.')

### Make training and validation sets from pathList
miniBatchSize = 64
stepSize = int(len(pathList)/miniBatchSize)
trainSamples, validationSamples = train_test_split(pathList, test_size=0.25)
trainGenerator = generator3Cams(trainSamples, batch_size = miniBatchSize)
validationGenerator = generator3Cams(validationSamples, batch_size = miniBatchSize)

### NN Model
from keras.models import Sequential
from keras.layers import Flatten, Dropout, Dense, Conv2D, Activation, MaxPooling2D, Lambda
from keras.optimizers import Adam
from keras.callbacks import ModelCheckpoint

# This model is from NVIDIA self driving car
# Ref: https://images.nvidia.com/content/tegra/automotive/images/2016/solutions/pdf/end-to-end-dl-using-px.pdf
model = Sequential()

model.add(Lambda(lambda x: x/255 - 0.5, input_shape = (80,320,3)))
model.add(Conv2D(24, (5,5), activation = 'relu', strides = (2,2)))
model.add(Conv2D(36, (5,5), activation = 'relu', strides = (2,2)))
model.add(Conv2D(48, (5,5), activation = 'relu', strides = (2,2)))
model.add(Conv2D(64, (3,3), activation = 'relu', strides = (1,1)))
model.add(Conv2D(64, (3,3), activation = 'relu', strides = (1,1)))

model.add(Flatten())
model.add(Dropout(0.5))
model.add(Dense(100))
model.add(Dropout(0.5))
model.add(Dense(50))
model.add(Dense(10))
model.add(Dense(1))


# Model training
adam = Adam(lr=0.001, beta_1=0.9, beta_2=0.999, epsilon=1e-08, decay=0.)
model.compile(loss = 'mse', optimizer = adam)
checkpointer = ModelCheckpoint(filepath="model.h5", verbose=1, save_best_only=True)
model.fit_generator(trainGenerator, steps_per_epoch= stepSize, validation_data=validationGenerator, \
			validation_steps= stepSize, epochs=10, callbacks=[checkpointer])

# Save model
model.save('model.h5')
