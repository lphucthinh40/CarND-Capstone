[//]: # (Image References)
[image_0]: ./imgs/carla_architecture.png "Carla"
[image_1]: ./imgs/ros_nodes.png "Ros Nodes"
[image_2]: ./imgs/rosgraph.png "Ros Graph"
[image_3]: ./imgs/demo.gif "Demo"
[ref_0]: https://github.com/tensorflow/models/blob/master/research/object_detection/g3doc/tf1_detection_zoo.md
[ref_1]: https://drive.google.com/file/d/0B-Eiyn-CUQtxdUZWMkFfQzdObUE/view

# System Integration Capstone Project :oncoming_automobile:
Self-Driving Car Engineer Nanodegree Program 

**Objective:** <br>
The goal of this project is to control and navigate Carla - Udacity's autonomous vehicle - on a simulated environment, using ROS and Tensorflow. The vehicle should follow a planned track safely and effectively without violating speed limit and max acceleration. A traffic light detector and classifier was implemented to help the vehicle detect and stop at red light.

---

## Build Instruction
For Free-way Map:

    cd ros
    catkin_make
    source devel/setup.sh
    roslaunch launch/styx.launch

For Testing Site:

    cd ros
    catkin_make
    source devel/setup.sh
    roslaunch launch/site.launch
    rosbag play -l just_traffic_light.bag  # for testing classifier
    rqt-image-view /image_color # to display image from rosbag


---

## Design & Implementation

### 1. CARLA Architecture
Carla is a self-driving Lincoln MKZ developed and owned by Udacity. The vehicle architecture consists of 4 components as described below:  
![alt text][image_0]

- **Sensors**: Hardwares that help Carla gathers data about its surrounding environment. This includes radars, lidars, GPS, ultrasonic sensors, and cameras.
- **Perception**: Softwares that transform sensor data into meaningful understandings about the surrounding environment. This gives Carla its intelligence in perceiving the world.
- **Planning**: Softwares that plan and decide on future trajectory and the corresponding behaviors in different circumstances.
- **Control**: Softwares that directly control the motion of Carla (throttle, brake, steering).

### 2. Design of ROS Nodes
![alt text][image_1]
![alt text][image_2]

Implementations:

- **Waypoint Updater Node**: Generate Final Waypoints for the vehicle to follow. Final Waypoints are a subset of Base Waypoints, which is an unchangeable and pre-loaded global trajectory that takes the vehicle from the starting position to the final position on the each map. It is noteworthy that each Waypoint consists of both Position and Velocity (represented as pose and twist in ROS). Since the vehicle velocity is determined by the vehicle's current state and its surrounding environments, Final Waypoints are needed to help the vehicle adapt its speed according to the current traffic. For example, when there is no obstacle and red line, the vehicle can simply what is pre-determined by Base Waypoints. However, when the vehicle need to stop due to incoming red light, Waypoint Updater will generate a different set of waypoints that allows the vehicle to decelerate and stop at the traffic light. 
- **Drive-by-Wire Node**: Generate a set of Throttle, Brake, and Steering values that directly dictate vehicle's instant motions. In order to generate these values, the node uses a set of PID-controller and Lowpass filter, as well as Yaw-controller in case of steering angle. This node only publishes new commands when the vehicle is in autonomous mode. Switching from Manual to Autonomous will reset the PID-controller to avoid unnecessary error pileup from manual driving. Another important note is that the velocity specified and published by the Waypoint Updater Node should only be considered as target velocity. DBW Node actually adjusts the speed of the vehicle based on Twist_CMD which is published by Pure Pursuit Node (from Autoware, provided by Udacity). In brief, Pure Pursuit Node tries to move the vehicle toward the target waypoint while matching its target velocity. It does so by publishing a very-short-term target velocity for the vehicle to reach so that it will eventually match the target speed at certain point in the future.
- **Traffic Light Detector Node**: detect and classify traffic light so that the vehicle will be able to stop at red light. SSD Inception V2 model from Tensorflow Object Detection Hub was trained and integrated to this node to perform both detection and classification tasks. When an incoming red light is detected, this node will publish the waypoint location of the traffic stopline so that the vehicle will know exactly where to stop at.

### 3. Design of Traffic Light Classifier

- The traffic light classifier uses the pre-trained model ssd_inception_v2_coco_2017_11_17 as the starting point. The model can be downloaded [here][ref_0]
- There are three classes {1:Red, 2:Yellow, 3:Green}
- For simulation training data, I collected and labelled a set of 300 images from the simulation map (100 for each class). I also generated tf-record file using Tensorflow Object Detection API.
- For real training data, I used images generated from Udacity rosbag file of the testing site. Due to time constraints, I ended up using the annotations and tf-record file provided by [vatsl's github][ref_1]
- Both models were trained for 20,000 steps. Since Carla requires Tensorflow 1.3, I had to train these models with Tensorflow 1.14 (to use my native GPU & Cuda 10) and then freeze them with Tensorflow 1.4.

## Demo

![alt text][image_3]




