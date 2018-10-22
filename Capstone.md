
---

** Capstone Project **

The goal of this project is to integrate a ROS system which incorporates a traffic light detector on a self driving car with the drive by wire system of the car that lets it follow a list of waypoints in the road while obeying the traffic lights along the way.

[//]: # (Image References)
[image1]: ./examples/final-project-ros-graph-v2.png
[image2]: ./examples/waypoint-updater-ros-graph.png
[image3]: ./examples/tl-detector-ros-graph.png
[image4]: ,/examples/dbw-node-ros-graph.png
[video1]: ,/examples/capstone.mp4

## [Rubric Points](https://review.udacity.com/#!/rubrics/1140/view)
All the code for this project has been derived from the example code in the course and is in this directory.
[Here](https://github.com/gvp-study/CarND-Capstone.git)

# ROS
## Implementing the Self Driving Car with Traffic Light Detection

I implemented this project based on the lessons and the code walk through done by the instructors. The bulk of the time I spent was on getting the subsystems working together. The figure below shows the system architecture.
![alt text][image1]

Note that I have not implemented the tl_detector/image_classification module due to time constraints. I hope to finish this and submit a complete project in the coming weeks.

## Waypoint Updater
The track the car navigates through is represented by a series of waypoints recorded in a map file. The waypoint_updater.py file updates the waypoints that the car has to follow in space and time based on the state of the traffic lights.
![alt text][image2]


## Traffic Light Detector
The tl_detector.py file sets up a image processing routine which will subscribe to the /image_color topic and compute the trajectory as a set of waypoints along with appropriate velocities. These waypoints are computed for a fixed look ahead distance and will have velocities adjusted to make sure they decelerate gently to a stop if the traffic light is red and smoothly accelerate to full speed when the traffic light is green.
![alt text][image3]

## DBW Node
The dbw_node.py file subscribes to a /twist_cmd topic and computes the throttle brake and steering values of the car based on the velocities set in the waypoints and publishes them.
![alt text][image4]

### Output
The movie of the simulator driven with this ROS system.[link to my video](./examples/capstone.mp4)
![alt text][video1]
