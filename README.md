# Udacity Self-Driving Car Nanodegree Capstone project


## Team: **Driving Miss Carla**

### Team Members:
* Egar Garcia (egar.garcia@gmail.com)
* Tim Howard (tim.guy.howard@gmail.com)
* Federica Paolicelli (federica.paolicelli@gmail.com)
* Ankith Manjunath (ankith.manjunath@avl.com)
* Nitikesh Bhad (er.nitikeshbhad@gmail.com)

## Background

This is the project repo for the final project of the Udacity Self-Driving Car Nanodegree: Programming a Real Self-Driving Car. For more information about the project, see the project introduction [here](https://classroom.udacity.com/nanodegrees/nd013/parts/6047fe34-d93c-4f50-8336-b70ef10cb4b2/modules/e1a23b06-329a-4684-a717-ad476f0d8dff/lessons/462c933d-9f24-42d3-8bdc-a08a5fc866e4/concepts/5ab4b122-83e6-436d-850f-9f4d26627fd9).

## Implemented Components

#### Waypoint Updater Node (Partial)
Subscribes to ```/base_waypoints``` and ```/current_pose``` and publishes to ```/final_waypoints``` (see file ```src/waypoint_updater/waypoint_updater.py```).

#### DBW Node
Once the ```waypoint updater``` is publishing to ```/final_waypoints```, the ```waypoint_follower``` node publishes messages to the ```/twist_cmd``` topic. The ```dbw_node``` receives these massages and transforms then into throttle, steering and brake commands (see file ```src/twist_controller/dbw_node.py```).

#### Traffic Light Detection

##### Detection
Detects the traffic light and its color from the ```/image_color``` topic, in order to do the color classification a CNN based in ResNet50 is used, the details of the model's building and training can be seen at https://github.com/DrivingMissCarla/Traffic-Light-Classifier (see file ```src/tl_detector/light_classification/tl_classifier.py```).

##### Waypoint publishing
When the traffic light color is identified, then the position (which can be found in ```/vehicle/traffic_lights```) of the lights that require a stop (red or yellow) is published in ```/traffic_waypoint``` (see file ```src/tl_detector/tl_detector.py```).

#### Waypoint Updater (Full)
Receives the position of red (or yellow) lights ahead from the topic ```/traffic_waypoint```, then changes waypoint target velocities before publishing to ```/final_waypoints``` in order to perform a stop (see file ```src/waypoint_updater/waypoint_updater.py```).

## Usage

1. Clone the project repository
```bash
git clone https://github.com/DrivingMissCarla/CarND-Capstone.git
```

2. Install python dependencies (if needed)
```bash
cd CarND-Capstone
pip install -r requirements.txt
```

3. Make
```bash
cd ros
catkin_make
source devel/setup.sh
```

4. Run in the Simulator

    4.1. Run styx
    ```
    roslaunch launch/styx.launch
    ```

    5.2. wait to see the mesage ```Traffic Light Classifier is READY``` in the console, meaning that the light classifier has been loaded

    4.3. Run the simulator

5. Run in Carla

    5.1. Run site
    ```
    roslaunch launch/site.launch
    ```

    5.2. wait to see the message ```Traffic Light Classifier is READY``` in the console, meaning that the light classifier has been loaded


## Submission checklist and requirements

Our team's project is able to drive the vehicle successfully in the simulator.

Please see this video of a successful run in the simulator: [![Capstone Project Run](http://img.youtube.com/vi/Y5bg1G2jTyI/0.jpg)](https://www.youtube.com/watch?v=Y5bg1G2jTyI)

And/or this one with a few runs in the church lot: [![Capstone Project Run](http://img.youtube.com/vi/3rFgo859y40/0.jpg)](https://www.youtube.com/watch?v=3rFgo859y40).

### Launch correctly using the launch files provided in the capstone repo.

No additional launch scripts were created or used other libraries than the provided with the original distribution.

however, some extra files where included which are used by the already existing python scripts and they shouldn't cause problems when running:
* ```src/tl_detector/points_organizer.py``` a helper class to sort 2D points an make more efficient to look for the closest points
* ```src/tl_detector/light_classification/classifier_model.yaml``` the model's description of the CNN (based in ResNet50) used for the traffic light classifier
* ```src/tl_detector/light_classification/classifier_model_weights.h5``` the file containing the weights used for the CNN used for the traffic light classifier

### Smoothly follow waypoints in the simulator

In order to achieve this goal the closest waypoint (in the list of given waypoints) to the current position of the vehicle is searched, then following waypoints given sample length are published (see method ```get_final_waypoints()``` on line 116 of ```src/waypoint_update/waypoint_updater.py```), these waypoints are used by the backend processes to do the necessary interpolation to follow the track.

An additional change was made in ```src/waypoint_follower/src/pure_pursuit_core.cpp``` line 257 to make sure the vehicle is always following the waypoints.

### Respect the target top speed set for the waypoints' ```twist.twist.linear.x``` in ```waypoint_loader.py```

No modifications were made to ```twist.twist.linear.x``` of the retrieved way points, except when decelerating in order to stop when a red (or yellow) light is detected ahead, but in that case the minimum of the already set velocity and the one calculated by decelerating method is chosen (see line 156 of ```src/waypoint_update/waypoint_updater.py```), meaning that the target top speed is respected.

The velocity already been set in the way points in general adheres to to given maximum/target. However, it is worth to mention that at times it might slightly exceed it due to the oscillating nature of the twist controller.

### Stop at traffic lights when needed.

The ```tl_detector``` uses the traffic light classifier (which is stored in the files ```src/tl_detector/light_classification/classifier_model.yaml``` and ```src/tl_detector/light_classification/classifier_model_weights.h5```), to publish the found red and yellow lights (see lines 101 to 118 of ```src/tl_detector/tl_detector.py```), then in combination with a method in reduce the velocity on the ahead waypoints (see lines 130 to 157 of ```src/waypoint_updater/waypoint_updater.py```) provides the mechanism to stop when needed. You can see this mechanism in action in the following video.

[![Capstone Project Run](http://img.youtube.com/vi/Y5bg1G2jTyI/0.jpg)](https://www.youtube.com/watch?v=Y5bg1G2jTyI)

### Stop and restart PID controllers depending on the state of ```/vehicle/dbw_enabled```

The state of ```/vehicle/dbw_enabled``` is captured by the ```dbw_node``` (see lines 63 and 88-89 of ```src/twist_controller/dbw_node.py```), this state is passed to the ```twist_controller``` (see line 82 of ```src/twist_controller/dbw_node.py```), then the twist controller does a reset if ```/vehicle/dbw_enabled``` is not enabled or does it is normal work if it is enabled (see lines 40-66 of ```src/twist_controller/twist_controller.py```).

### Publish throttle, steering, and brake commands at 50hz

In the the ```dbw_node``` is implemented the loop in charge of publishing the throttle, steering and brake commands with a rate of 50hz (see lines 74 and 76-86 of ```src/twist_controller/dbw_node.py```).

## Apendix

### Installation

Please use **one** of the two installation options, either native **or** docker installation.

#### Native Installation

* Be sure that your workstation is running Ubuntu 16.04 Xenial Xerus or Ubuntu 14.04 Trusty Tahir. [Ubuntu downloads can be found here](https://www.ubuntu.com/download/desktop).
* If using a Virtual Machine to install Ubuntu, use the following configuration as minimum:
  * 2 CPU
  * 2 GB system memory
  * 25 GB of free hard drive space

  The Udacity provided virtual machine has ROS and Dataspeed DBW already installed, so you can skip the next two steps if you are using this.

* Follow these instructions to install ROS
  * [ROS Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu) if you have Ubuntu 16.04.
  * [ROS Indigo](http://wiki.ros.org/indigo/Installation/Ubuntu) if you have Ubuntu 14.04.
* [Dataspeed DBW](https://bitbucket.org/DataspeedInc/dbw_mkz_ros)
  * Use this option to install the SDK on a workstation that already has ROS installed: [One Line SDK Install (binary)](https://bitbucket.org/DataspeedInc/dbw_mkz_ros/src/81e63fcc335d7b64139d7482017d6a97b405e250/ROS_SETUP.md?fileviewer=file-view-default)
* Download the [Udacity Simulator](https://github.com/udacity/CarND-Capstone/releases).

#### Docker Installation
[Install Docker](https://docs.docker.com/engine/installation/)

Build the docker container
```bash
docker build . -t capstone
```

Run the docker file
```bash
docker run -p 4567:4567 -v $PWD:/capstone -v /tmp/log:/root/.ros/ --rm -it capstone
```

#### Port Forwarding
To set up port forwarding, please refer to the [instructions from term 2](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/16cf4a78-4fc7-49e1-8621-3450ca938b77)

### Testing

#### Simulator

1. Clone the project repository
```bash
git clone https://github.com/DrivingMissCarla/CarND-Capstone.git
```

2. Install python dependencies
```bash
cd CarND-Capstone
pip install -r requirements.txt
```
3. Make and run styx
```bash
cd ros
catkin_make
source devel/setup.sh
roslaunch launch/styx.launch
```
4. Run the simulator

#### Real world testing
1. Download [training bag](https://s3-us-west-1.amazonaws.com/udacity-selfdrivingcar/traffic_light_bag_file.zip) that was recorded on the Udacity self-driving car.
2. Unzip the file
```bash
unzip traffic_light_bag_file.zip
```
3. Play the bag file
```bash
rosbag play -l traffic_light_bag_file/traffic_light_training.bag
```
4. Launch your project in site mode
```bash
cd CarND-Capstone/ros
roslaunch launch/site.launch
```
5. Confirm that traffic light detection works on real life images
