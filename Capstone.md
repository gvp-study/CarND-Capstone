
---

** Capstone Project **

The goal of this project is to integrate a ROS system which incorporates a traffic light detector on a self driving car with the drive by wire system of the car and lets it follow a list of waypoints in the road while obeying the traffic lights.


## [Rubric Points](https://review.udacity.com/#!/rubrics/1140/view)
All the code for this project has been derived from the example code in the course and is in this directory.
[Here](https://github.com/gvp-study/CarND-Capstone.git)

# ROS
## Implementing the SDC with Traffic Light Detection

I implemented this project based on the lessons and the code walk through done by the instructors. I found that the bulk of the time I spent was on getting the subsystems working together into a whole.

Due to time constraints I was unable to join a team in my cohort. When I did register with a team, the team members individually decided to implement the modules by themselves. So I decided to follow suit and do all the modules by myself. Unfortunately, I was not able to implement the tl_detector/light_classification module with real images.

## Waypoint Updater
The track the car navigates through is represented by a series of waypoints recorded in a map file. The waypoint_updater.py file updates the map points that the car has to follow in space and time based on the state of the traffic lights.

## Traffic Light Detector
The tl_detector.py file sets up a image processing routine which will subscribe to the /image_color topic and compute the trajectory as a set of waypoints with the correct velocities. These waypoints are computed for a fixed look ahead distance and will have velocities adjusted to make sure they decelerate gently to a stop if the traffic light is red and smoothly accelerate to full speed when the traffic light is green.

## DBW Node
The dbw_node.py file subscribes to a /twist_cmd topic and computes the throttle brake and steering values of the car and publishes them.
