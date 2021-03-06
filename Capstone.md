
---

# Capstone Project

The goal of this project is to integrate a ROS system which incorporates a traffic light detector on a self driving car with the drive by wire system of the car that lets it follow a list of waypoints in the road while obeying the traffic lights along the way.

## Team
Due to time constraints I was unable to join a team in my cohort. When I did register with a team, the team members individually decided to implement the modules by themselves. So I decided to follow suit and do all the modules by myself. The team details of my signup is as follows:
Looking for a Team:	Craig Martin	craig@craigmartin.com, Christie English	ChristieEnglish@gmail.com, George V. Paul	gvpsha@gmail.com	Nikola Noxon	nikolanoxon@daimler.com

[//]: # (Image References)
[image1]: ./examples/final-project-ros-graph-v2.png
[image2]: ./examples/waypoint-updater-ros-graph.png
[image3]: ./examples/tl-detector-ros-graph.png
[image4]: ./examples/dbw-node-ros-graph.png
[video1]: ,/examples/capstone.mp4

## [Rubric Points](https://review.udacity.com/#!/rubrics/1140/view)
All the code for this project has been derived from the example code in the course and is in this directory.
[Here](https://github.com/gvp-study/CarND-Capstone.git)

## Implementing the Self Driving Car with Traffic Light Detection using ROS

I implemented this project based on the lessons and the code walk through done by the instructors. The bulk of the time I spent was on getting the subsystems working together. The figure below shows the system architecture.
![alt text][image1]

## Waypoint Updater
The track the car navigates through is represented by a series of waypoints recorded in a map file. The waypoint_updater.py file updates the waypoints that the car has to follow in space and time based on the state of the traffic lights.
![alt text][image2]


## Traffic Light Detector
The tl_detector.py file sets up a image processing routine which will subscribe to the /image_color topic and compute the trajectory as a set of waypoints along with appropriate velocities. These waypoints are computed for a fixed look ahead distance and will have velocities adjusted to make sure they decelerate gently to a stop if the traffic light is red and smoothly accelerate to full speed when the traffic light is green.
![alt text][image3]

In this version, I use a two step method to implement the tl_detector/image_classification module. The first step detects the traffic light in the image if there is one and the second step classifies the detected traffic light for the color. This particular implementation uses Tensorflow object detection using a pretrained model from the COCO dataset for the first step. Once the traffic light is detected and the bounding box found, a second network is used to recognize the state of the traffic light as being green, yellow or red. The two networks operate in series on every camera image from the car.

## DBW Node
The dbw_node.py file subscribes to a /twist_cmd topic and computes the throttle brake and steering values of the car based on the velocities set in the waypoints and publishes them.
![alt text][image4]

### Output
The movie of the simulator driven with this ROS system.[link to my video](./examples/capstone.mp4)
![alt text][video1]

### References
https://medium.com/@WuStangDan/step-by-step-tensorflow-object-detection-api-tutorial-part-1-selecting-a-model-a02b6aabe39e
