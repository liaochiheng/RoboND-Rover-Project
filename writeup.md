## Project: Search and Sample Return
### This is a writeup about how I accomplished this project.

---


**The goals / steps of this project are the following:**  

**Training / Calibration**  

* Download the simulator and take data in "Training Mode"
* Test out the functions in the Jupyter Notebook provided
* Add functions to detect obstacles and samples of interest (golden rocks)
* Fill in the `process_image()` function with the appropriate image processing steps (perspective transform, color threshold etc.) to get from raw images to a map.  The `output_image` you create in this step should demonstrate that your mapping pipeline works.
* Use `moviepy` to process the images in your saved dataset with the `process_image()` function.  Include the video you produce as part of your submission.

**Autonomous Navigation / Mapping**

* Fill in the `perception_step()` function within the `perception.py` script with the appropriate image processing functions to create a map and update `Rover()` data (similar to what you did with `process_image()` in the notebook). 
* Fill in the `decision_step()` function within the `decision.py` script with conditional statements that take into consideration the outputs of the `perception_step()` in deciding how to issue throttle, brake and steering commands. 
* Iterate on your perception and decision function until your rover does a reasonable (need to define metric) job of navigating and mapping.  

[//]: # (Image References)

[rock_ob]: ./misc/rock_ob.jpg

## [Rubric](https://review.udacity.com/#!/rubrics/916/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---
### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.  

You're reading it!

### Notebook Analysis
#### 1. Run the functions provided in the notebook on test images (first with the test data provided, next on data you have recorded). Add/modify functions to allow for color selection of obstacles and rock samples.
* The code located under section "Thresh out the rocks and obstacles.".
* Code #10 is for rock-thresh, which identify rocks by R > thresh_R & G > thresh_G & B < thresh_B
* Code #11 if for obstacle-thresh, which is all the left except navigables and rocks based on warped-one-image.
```
ob_threshed = np.zeros_like(rock_img[:, :, 0])
ob_threshed[(warped_one == 1) & (nav_threshed == 0) & (rock_threshed == 0)] = 1
```
* This image shows how these work for rocks and obstacles.
![How to thresh out rocks and obstacles][rock_ob]


#### 2. Populate the `process_image()` function with the appropriate analysis steps to map pixels identifying navigable terrain, obstacles and rock samples into a worldmap.  Run `process_image()` on your test data using the `moviepy` functions provided to create video output of your result. 
* Code #14 is for process_image.
* First, i calculated 3 binary images: nav_threshed, rock_threshed, and ob_threshed.
* Then, i updated data.worldmap.


### Autonomous Navigation and Mapping

#### 1. Fill in the `perception_step()` (at the bottom of the `perception.py` script) and `decision_step()` (in `decision.py`) functions in the autonomous mapping scripts and an explanation is provided in the writeup of how and why these functions were modified as they were.

1. perception_step
* Calculate 3 binary images: rock_img(line #132), nav_img(line #133), ob_img(line #143).
* Check pitch and roll, only update worldmap while near to 0, <0.5 for my code(line #153).
* Update worldmap with each binary image(line #155-167).
* For rock identification:
  + calculate rock polar coords(line #160): `rock_dists, rock_angles = to_polar_coords(xpix, ypix)`
  + calculate nav polar coords for a nearby clip of the whole nav image(line #174): `xpix, ypix = rover_coords(nav_img[110:, :])
	Rover.nav_dists, Rover.nav_angles = to_polar_coords(xpix, ypix)
`
  + check if number of rock pixels is more than 10, then consider as a rock.
  + set rock_angle and rock_yaw for pickup, when distance is near enough(< 80) and the navigable pixels are enough(> 50).(line #189).

2. decision_step
* I added a few modes: start, stuck, stuck_forward, rock_stop, rock_forward.
* `mode = start` means the very begining. Rover will go straight till wall then will stop.
* `mode = stop` not changed. Rover stops and then turns for a well-to-go position.
* `mode = forward` 1) check if there is a rock ahead by `if Rover.rock_angle is not None`.
* `mode = forward` 2) if a rock ahead, then switch to `mode = rock_stop`.
* `mode = forward` 3) else then `check_stuck`, switch to `mode = stuck` if stuck, ortherwise go forward.
* `mode = forward` 4）if go forward, then the question is how to find steer to keep a wall left. The code for this is `update_steer`(line #197), Rover is trying to keep a certain rate of nav pixels on left(0.2 mean 20%).
* `mode = stuck` Rover will keep turning to right untill `len(Rover.nav_angles) >= Rover.go_forward`, then switch to `mode = stuck_forward`.
* `mode = stuck_forward` if stuck, switch to `mode = stuck`, otherwise try forward with `Rover.steer = np.clip(np.mean(Rover.nav_angles * 180/np.pi), -15, 15)` untill `Rover.vel >= 0.5` which means get-out-of-stuck.
* `mode = rock_stop` first stop, second turn to the right direction, then switch to `mode = rock_forward`.
* `mode = rock_forward` go forward untill the rock and pick up, if stuck switch to `mode = stuck`.


#### 2. Launching in autonomous mode your rover can navigate and map autonomously.  Explain your results and how you might improve them in your writeup.  

**Note: running the simulator with different choices of resolution and graphics quality may produce different results, particularly on different machines!  Make a note of your simulator settings (resolution and graphics quality set on launch) and frames per second (FPS output to terminal by `drive_rover.py`) in your writeup when you submit the project so your reviewer can reproduce your results.**

Here I'll talk about the approach I took, what techniques I used, what worked and why, where the pipeline might fail and how I might improve it if I were going to pursue this project further.  

**Test environment:**
* resolution: 800 * 600
* graphics quality: Good
* MacOS 10.13.3 with intel i5 3.5G and AMD Raden R9 M290X 2GB
* FPS: 38

Overall, Rover works like this:
1. Go straight untill block by wall or other obstacles.
2. Turn right untill find a way to go. The wall should be on the left.
3. Keep left pixels around 20%.
4. Pickup if find a rock ahead.
5. Check stuck in most of modes, and if stuck, turn right untill find a way to go.
That's all.

However, there are 3 problems left to be solved:
1. Sometimes, Rover will stuck in `mode = stuck` while turning right, because it's blocked to turn right.
2. In the middle area, with a few black rocks, Rover struggled to get out of stuck. It could get out in most of times, but still get stuck in few times.
3. Since it always turning right on `mode = stop` or `mode = stuck`, Rover will turn back after picking up a rock on the right side of road. It will cost more time to finish the map. It's not sufficient.


