# Udacity Self-Driving Car Nanodegree Capstone project


## Team: **Driving Miss Carla**

### Team Members:
* Egar Garcia (egar.garcia@gmail.com)
* Tim Howard (tim.guy.howard@gmail.com)
* Federica Paolicelli (federica.paolicelli@gmail.com)
* Ankith Manjunath (ankith.manjunath@avl.com)
* Nitikesh Bhad (er.nitikeshbhad@gmail.com)

## Background

The System Integration project is the Capstone project of the Udacity Self-Driving Car Engineer Nanodegree. We used ROS ([Robot Operating System](http://wiki.ros.org/)) to allow different elements, such as traffic light detection and classification, trajectory planning and control, to communicate with each other. The software we developed will be tested on Carla, the Udacity Self Driving Car, which will drive autonomously on a test track. For more information, see the project introduction [here](https://classroom.udacity.com/nanodegrees/nd013/parts/6047fe34-d93c-4f50-8336-b70ef10cb4b2/modules/e1a23b06-329a-4684-a717-ad476f0d8dff/lessons/462c933d-9f24-42d3-8bdc-a08a5fc866e4/concepts/5ab4b122-83e6-436d-850f-9f4d26627fd9).

## Implemented Components

#### Waypoint Updater Node (Partial)
Subscribes to ```/base_waypoints``` and ```/current_pose``` and publishes to ```/final_waypoints``` (see file ```src/waypoint_updater/waypoint_updater.py```).

#### DBW Node
Once the ```waypoint_updater``` is publishing to ```/final_waypoints```, the ```waypoint_follower``` node publishes messages to the ```/twist_cmd``` topic. The ```dbw_node``` receives these massages and transforms then into throttle, steering and brake commands (see file ```src/twist_controller/dbw_node.py```).

#### Traffic Light Detection

##### Detection
Detects the traffic light and its color from the ```/image_color``` topic. In order to do the color classification a CNN based in SqueezeNet is used. The details of the model's building and training can be seen at https://github.com/DrivingMissCarla/Traffic-Light-Classifier (see file ```src/tl_detector/light_classification/tl_classifier.py```).

##### Waypoint publishing
When the traffic light color is identified, the position (which can be found in ```/vehicle/traffic_lights```) of the light that requires a stop (red or yellow) is published in ```/traffic_waypoint``` (see file ```src/tl_detector/tl_detector.py```).

#### Waypoint Updater (Full)
Receives the position of red (or yellow) lights ahead from the topic ```/traffic_waypoint```, then changes waypoint target velocities before publishing to ```/final_waypoints``` in order to perform a stop (see file ```src/waypoint_updater/waypoint_updater.py```).

## Submission checklist and requirements

Our team's project is able to drive the vehicle successfully in the simulator.

Please see this video of a successful run in the simulator:

[![Capstone Project Run](http://img.youtube.com/vi/ga6i7Juu054/0.jpg)](https://www.youtube.com/watch?v=ga6i7Juu054)

And this one with a few runs in the church lot:

[![Capstone Project Run](http://img.youtube.com/vi/MBh4Rtx8Jcs/0.jpg)](https://www.youtube.com/watch?v=MBh4Rtx8Jcs).

### Launch correctly using the launch files provided in the capstone repo.

No additional launch scripts were created or used other libraries than the provided with the original distribution.

However, some extra files where included which are used by the already existing python scripts and they shouldn't cause problems when running:
* ```src/tl_detector/points_organizer.py``` a helper class to sort 2D points and make more efficient to look for the closest points
* ```src/tl_detector/light_classification/classifier_model.yaml``` the model's description of the CNN (based in ResNet50) used for the traffic light classifier
* ```src/tl_detector/light_classification/classifier_model_weights.h5``` the file containing the weights used for the CNN used for the traffic light classifier

### Smoothly follow waypoints in the simulator

In order to achieve this goal the closest waypoint (in the list of given waypoints) to the current position of the vehicle is searched, then following waypoints given sample length are published (see method ```get_final_waypoints()``` on line 117 of ```src/waypoint_update/waypoint_updater.py```). These waypoints are used by the backend processes to do the necessary interpolation to follow the track.

An additional change was made in ```src/waypoint_follower/src/pure_pursuit_core.cpp``` line 257 to make sure the vehicle is always following the waypoints.

### Respect the target top speed set for the waypoints ```twist.twist.linear.x``` in ```waypoint_loader.py```

No modifications were made to ```twist.twist.linear.x``` of the retrieved way points, except when decelerating in order to stop when a red (or yellow) light is detected ahead. In that case, the minimum between the already set velocity and the one calculated by decelerating method is chosen (see line 167 of ```src/waypoint_update/waypoint_updater.py```), meaning that the target top speed is respected.

Please see the following run targeted to 35 mph (56km/h).

[![Capstone Project Run at 35mph](http://img.youtube.com/vi/ywwam9i1X_E/0.jpg)](https://www.youtube.com/watch?v=ywwam9i1X_E)

Please see the following run targeted to 50 mph (96km/h).

[![Capstone Project Run at 35mph](http://img.youtube.com/vi/kK9VyfS2COQ/0.jpg)](https://www.youtube.com/watch?v=kK9VyfS2COQ)

The velocity already been set in the waypoints in general adheres to the given maximum/target. However, it is worth to mention that at times it might slightly exceed it due to the oscillating nature of the twist controller.

### Stop at traffic lights when needed.

The ```tl_detector``` uses the traffic light classifier (which is stored in the files ```src/tl_detector/light_classification/classifier_model.yaml``` and ```src/tl_detector/light_classification/classifier_model_weights.h5```), to publish the found red, yellow and unknown lights (see lines 102 to 119 of ```src/tl_detector/tl_detector.py```), then in combination with a method to reduce the velocity on the ahead waypoints (see lines 141 to 170 of ```src/waypoint_updater/waypoint_updater.py```) provides the mechanism to stop when needed. You can see this mechanism in action in the following video.

[![Capstone Project Run](http://img.youtube.com/vi/ga6i7Juu054/0.jpg)](https://www.youtube.com/watch?v=ga6i7Juu054)

### Stop and restart PID controllers depending on the state of ```/vehicle/dbw_enabled```

The state of ```/vehicle/dbw_enabled``` is captured by the ```dbw_node``` (see lines 72 and 90-91 of ```src/twist_controller/dbw_node.py```), this state is passed to the ```twist_controller``` (see lines 83-84 of ```src/twist_controller/dbw_node.py```), then the twist controller does a reset if ```/vehicle/dbw_enabled``` is not enabled or does it is normal work if it is enabled (see lines 40-66 of ```src/twist_controller/twist_controller.py```).

### Publish throttle, steering, and brake commands at 50Hz

In the the ```dbw_node``` is implemented the loop in charge of publishing the throttle, steering and brake commands with a rate of 50Hz (see lines 76 and 78-88 of ```src/twist_controller/dbw_node.py```).

## Delivery

### Performance issues during simulator tests

Two branches have been added in the repository to handle latency issues during simulations with Udacity's workspace and VM if you don't have a GPU:

* [sim](https://github.com/DrivingMissCarla/CarND-Capstone/tree/sim): the publish rate of throttle, steering and brake commands is set at 10Hz (instead of 50Hz).

* [noclassif](https://github.com/DrivingMissCarla/CarND-Capstone/tree/noclassif): the light classifier is disabled and the current state of the traffic lights is used instead.

### Run on Carla

The following two videos have been retrieved from the RosBags recorded during the runs on Carla, i.e. the real-life tests.

[![Carla's Run 1](http://img.youtube.com/vi/rmfb-hfoCcI/0.jpg)](https://www.youtube.com/watch?v=rmfb-hfoCcI)

[![Carla's Run 2](http://img.youtube.com/vi/v1xkGzAYWKo/0.jpg)](https://www.youtube.com/watch?v=v1xkGzAYWKo)


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
4. Wait to see the mesage ```Traffic Light Classifier is READY``` in the console, meaning that the light classifier has been loaded

5. Run the simulator
