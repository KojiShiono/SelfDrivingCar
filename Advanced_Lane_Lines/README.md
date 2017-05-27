## Advanced Lane Finding Project

The goal of this project is to identify and mark the driving lanes in the provided input images/videos.  

The steps of this project are the following:

* Setups

&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; S1.  Compute the camera calibration matrix and distortion coefficients given a set of chessboard images  
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; S2.  Obtain warp matrix for perspective transform  
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; S3.  Setup helper functions for color transform / thresholding  
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; S4.  Setup helper functions for lane recognition  
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; S5.  Setup helper functions for curvature calculation  

* Applications  

&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; A1. Apply a distortion correction to raw images  
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; A2. Use color transforms, gradients, etc., to create a thresholded binary image  
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; A3. Apply a perspective transform to rectify binary image ("birds-eye view")  
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; A4. Detect lane pixels and fit to find the lane boundary  
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; A5. Determine the curvature of the lane and vehicle position with respect to center  
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; A6. Warp the detected lane boundaries back onto the original image  
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; A7. Output visual display of the lane, estimation of lane curvature and vehicle position

[//]: # (Image References)

[image1]: ./examples/undistort_output.png "Undistorted"
[image2]: ./test_images/test1.jpg "Road Transformed"
[image3]: ./examples/binary_combo_example.jpg "Binary Example"
[image4]: ./examples/warped_straight_lines.jpg "Warp Example"
[image5]: ./examples/color_fit_lines.jpg "Fit Visual"
[image6]: ./examples/example_output.jpg "Output"
[video1]: ./project_video.mp4 "Video"

## [Rubric Points](https://review.udacity.com/#!/rubrics/571/view)

### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---

### S1. Camera Calibration

The code for this step is contained in [IPython notebook](advLaneFind.ipynb), section "S1. Get camera calibration matrix".

I start by preparing "object points", which will be the (x, y, z) coordinates of the chessboard corners in the world. Here I am assuming the chessboard is fixed on the (x, y) plane at z=0, such that the object points are the same for each calibration image.  Thus, `objp` is just a replicated array of coordinates, and `objpoints` will be appended with a copy of it every time I successfully detect all chessboard corners in a test image.  `imgpoints` will be appended with the (x, y) pixel position of each of the corners in the image plane with each successful chessboard detection.  

I then used the output `objpoints` and `imgpoints` to compute the camera calibration and distortion coefficients using the `cv2.calibrateCamera()` function.  I applied this distortion correction to the test image using the `cv2.undistort()` function and obtained this result: 

![Fig.1 Camera Calibration][outputs/imageUndistort.png]

### Pipeline (single images)

#### 1. Provide an example of a distortion-corrected image.

To demonstrate this step, I will describe how I apply the distortion correction to one of the test images like this one:
![alt text][image2]

#### 2. Describe how (and identify where in your code) you used color transforms, gradients or other methods to create a thresholded binary image.  Provide an example of a binary image result.

I used a combination of color and gradient thresholds to generate a binary image (thresholding steps at lines # through # in `another_file.py`).  Here's an example of my output for this step.  (note: this is not actually from one of the test images)

![alt text][image3]

#### 3. Describe how (and identify where in your code) you performed a perspective transform and provide an example of a transformed image.

The code for my perspective transform includes a function called `warper()`, which appears in lines 1 through 8 in the file `example.py` (output_images/examples/example.py) (or, for example, in the 3rd code cell of the IPython notebook).  The `warper()` function takes as inputs an image (`img`), as well as source (`src`) and destination (`dst`) points.  I chose the hardcode the source and destination points in the following manner:

```python
src = np.float32(
    [[(img_size[0] / 2) - 55, img_size[1] / 2 + 100],
    [((img_size[0] / 6) - 10), img_size[1]],
    [(img_size[0] * 5 / 6) + 60, img_size[1]],
    [(img_size[0] / 2 + 55), img_size[1] / 2 + 100]])
dst = np.float32(
    [[(img_size[0] / 4), 0],
    [(img_size[0] / 4), img_size[1]],
    [(img_size[0] * 3 / 4), img_size[1]],
    [(img_size[0] * 3 / 4), 0]])
```

This resulted in the following source and destination points:

| Source        | Destination   | 
|:-------------:|:-------------:| 
| 585, 460      | 320, 0        | 
| 203, 720      | 320, 720      |
| 1127, 720     | 960, 720      |
| 695, 460      | 960, 0        |

I verified that my perspective transform was working as expected by drawing the `src` and `dst` points onto a test image and its warped counterpart to verify that the lines appear parallel in the warped image.

![alt text][image4]

#### 4. Describe how (and identify where in your code) you identified lane-line pixels and fit their positions with a polynomial?

Then I did some other stuff and fit my lane lines with a 2nd order polynomial kinda like this:

![alt text][image5]

#### 5. Describe how (and identify where in your code) you calculated the radius of curvature of the lane and the position of the vehicle with respect to center.

I did this in lines # through # in my code in `my_other_file.py`

#### 6. Provide an example image of your result plotted back down onto the road such that the lane area is identified clearly.

I implemented this step in lines # through # in my code in `yet_another_file.py` in the function `map_lane()`.  Here is an example of my result on a test image:

![alt text][image6]

---

### Pipeline (video)

#### 1. Provide a link to your final video output.  Your pipeline should perform reasonably well on the entire project video (wobbly lines are ok but no catastrophic failures that would cause the car to drive off the road!).

Here's a [link to my video result](./project_video.mp4)

---

### Discussion/Future work

There are 3 videos with different difficulties, provided by Udacity. For the easiest version ([easiest original](project_video.mp4)), the advanced techniques mentioned above (left-right split, segment check, confidence check, and polynomial check) were not really necessary: even without them, the output of the video looked very good ([easiest output](output_project_video.mp4)). These were implemented, however, to tackle more difficult videos, medium ([medium original](challenge_video.mp4)) and hardest ([hardest original](harder_challenge_video.mp4)).  

For the medium level, the algorithm looks to be working fine for the most of the time, although it still requires some further refinement. The result is shown here ([medium output](outputs/challenge_video.mp4)). The key factor for this particular video was the segment check and filtering: the original naive algorithm tend to pick the noises as the lines, which are totally misaligned with the vehicle, and segment check enforces the lines to start from two edges of the hood. Filterling smoothes out the measurement, as well as eliminates noise in measurements if it is only for 1-2 frames.  

For the hardest level, my algorithm is insufficient, since it relies on the assumption that good meeasurement shall be the majority, as well as left and right lanes shall be found and be parallel to each other. As shown in the result ([hardest output](outputs/harder_challenge_fideo.mp4)), it has no idea when one of the two lane lines is missing for significant amount of time.  

The possible solution for this is to use only one of the two lanes as reference if the measurement confidence is high, just as what Tesla Autopilot does. However, this requires even more sophisticated algorithm, since incorrect measurement on one lane can result in fatal consequence. One way to handle this is letting the system to say "no lane found", just as, again, Tesla vehicles, but I need to make further research for this. The other potential solution is to use sensors like radar or ultrasonic sensor to identify the boundary, but this is out of the scope of this project.
