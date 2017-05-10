**Behavioral Cloning** 


---

**Behavioral Cloning Project**

The goals / steps of this project are the following:
* Use the simulator to collect data of good driving behavior
* Build, a convolution neural network in Keras that predicts steering angles from images
* Train and validate the model with a training and validation set
* Test that the model successfully drives around track one without leaving the road
* Summarize the results with a written report


[//]: # (Image References)

[image1]: ./examples/placeholder.png "Model Visualization"
[image2]: ./examples/placeholder.png "Grayscaling"
[image3]: ./examples/placeholder_small.png "Recovery Image"
[image4]: ./examples/placeholder_small.png "Recovery Image"
[image5]: ./examples/placeholder_small.png "Recovery Image"
[image6]: ./examples/placeholder_small.png "Normal Image"
[image7]: ./examples/placeholder_small.png "Flipped Image"

## Rubric Points
###Here I will consider the [rubric points](https://review.udacity.com/#!/rubrics/432/view) individually and describe how I addressed each point in my implementation.  

---
###Files Submitted & Code Quality

####1. Submission includes all required files and can be used to run the simulator in autonomous mode

My project includes the following files:
* model.py containing the script to create and train the model
* drive.py for driving the car in autonomous mode
* model.h5 containing a trained convolution neural network 
* writeup_report.md or writeup_report.pdf summarizing the results

####2. Submission includes functional code
Using the Udacity provided simulator and my drive.py file, the car can be driven autonomously around the track by executing 
```sh
python drive.py model.h5
```

####3. Submission code is usable and readable

The model.py file contains the code for training and saving the convolution neural network. The file shows the pipeline I used for training and validating the model, and it contains comments to explain how the code works.

###Model Architecture and Training Strategy

####1. An appropriate model architecture has been employed

My model consists of a convolution neural network with 3x3 filter sizes and depths between 32 and 128 (model.py lines 18-24) 

The model includes RELU layers to introduce nonlinearity (code line 20), and the data is normalized in the model using a Keras lambda layer (code line 18). 

####2. Attempts to reduce overfitting in the model

The model contains dropout layers in order to reduce overfitting (model.py lines 21). 

The model was trained and validated on different data sets to ensure that the model was not overfitting (code line 10-16). The model was tested by running it through the simulator and ensuring that the vehicle could stay on the track.

####3. Model parameter tuning

The model used an adam optimizer, so the learning rate was not tuned manually (model.py line 143).

####4. Appropriate training data

Training data was chosen to keep the vehicle driving on the road. I used a combination of center lane driving, recovering from the left and right sides of the road ... 

For details about how I created the training data, see the next section. 

###Model Architecture and Training Strategy

####1. Solution Design Approach

The overall strategy:

I used the model published by NVIDIA as my base, since the model is relatively simple and the paper describes they were able to train the model with relatively small (still, 100 hours of driving data, though). (https://images.nvidia.com/content/tegra/automotive/images/2016/solutions/pdf/end-to-end-dl-using-px.pdf)

In order to gauge how well the model was working, I split my image and steering angle data into a training and validation set. Initially, I started with the dataset provided by Udacity, along with a very small dirving data that I created by myself. Both training and validation accuracies were good, but when I ran the model in the simulator, the vehicle was running in zigzag. After some observation, I noticed this behavior gets worse when I increase the epoch number, indicating this is caused by overfitting. 

Therefore, I increased the dataset size by the factor of 4 (although this is still small) and trained for only 1 epoch. This gave me the first successfully-running model, although the behavior on the bridge was not very stable.

At the end of the process, the vehicle is able to drive autonomously around the track without leaving the road.

####2. Final Model Architecture

The final model architecture (model.py lines 124-139) consisted of a convolution neural network with the following layers and layer sizes:

Normalization

Convolution: 5x5x24, strides = (2,2)

Convolution: 5x5x36, strides = (2,2)

Convolution: 5x5x48, strides = (2,2)

Convolution: 3x3x64, strides = (1,1)

Convolution: 3x3x64, strides = (1,1)

Fully Connected: 100

Fully Connected: 50

Fully Connected: 10

Fully Connected: 1

Model Architecture:

![Model Architecture](../modelArchitecture.png)

OR

please refer to the paper mentioned in "Solution Design Approach > The overall strategy"

####3. Creation of the Training Set & Training Process

To capture good driving behavior, I first recorded two laps on track one using center lane driving. However, as mentioned before, this training set was too small to be a useful dataset. Therefore, I recorded more than 10 laps in both two courses in both directions (although, again, it is not sufficient . Few recovering maneuver from sideways are also recorded.

To augment the data sat, I also flipped images and angles thinking that this would alleviate the bias toward on direction. After the collection process, I had more than 612,000 number of data points. I then preprocessed this data by trimming the image to remove the background (see images below). Normalization is also performed in the first stage of the model.

![Original](../original.jpg)
![Cropped Image](../cropped.jpg)

I finally randomly shuffled the data set and put 25% of the data into a validation set. 

I used this training data for training the model. The validation set helped determine if the model was over or under fitting. The ideal number of epochs was 1 as evidenced through experiments. I used an adam optimizer so that manually changing training the learning rate wasn't necessary.

####4. Next step
Although the vehicle successfully completed driving a full lap, the performance was not very stable. As can be seen in the video, it drove zigzag on the bridge, and it also sometimes failed to follow the path depending on the initial position or speed. Furthermore, it is still unable to drive on the course 2. Hence, my next step is to increase the robustness of the model, largely depends on the size of the training data. I will generate more driving data from course 2 and re-train the model.

Also, I am trying to employ inception module for this project, although it is not quite successful yet. The bottle neck is the enourmous amount of nodes on flatten layer, which caused memory outage on AWS g2.xlarge environment in some settings. In addition, it is showing the sign of overfitting, which I suspect due to the complaxity of the architecture. I will try again with bigger dataset.
