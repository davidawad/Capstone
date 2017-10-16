# Capstone

This is our Capstone project repo for the final project of the Udacity Self-Driving Car Nanodegree.

### DBW_node

We implement speed and steering using a PID controller with angular velocity as the goal. This is all implemented in  `twist_controller.py`.


```python
# If Positive difference throttle = nonzero, brake = 0
        # Else throttle = 0, brake = nonzero
        if(del_vel > 0) and (current_velocity < target_velocity):
            throttle = self.throttle_limit*(1.0 - (current_velocity/target_velocity))
            brake = 0
        else:
            throttle = 0
            brake = torque
```

### Waypoint Updater

The waypoint_updater finds next waypoints from the closest waypoint based on the current position and the orientation of the vehicle.  It then generates the final_waypoints with the velocity as defined by the launch file.


If the `traffic_waypoint` is published, `waypoint_updater` will generate a smooth deceleration velocity for the `final_waypoints` using a spline. In order to prevent deceleration taking too long, a parameter  `MAX_BRAKE_DISTANCE` is defined. We only start to decelerate if the distance between current location and the stopping waypoint is less than `MAX_BRAKE_DISTANCE`.


### Waypoint Loader

In order to save bandwidth, waypoint_loader only publish `base_waypoints` at rate of 1 hertz, and stop publishing after 5 seconds.


### Traffic Sign Detection

We trained machine learning models to detect traffic lights in images in order to detect when we were coming up against a traffic light so that we could publish this information on a ROS node.

### Traffic Sign Classification


The traffic sign classifier is based on Squeezenets that have seen a lot of success for the object detection problem. The advantage of using Squeezenet over other networks is that it provides AlexNet level accuracy with 50 times fewer parameters. The images have been preprocessed and resized to the size `300x300` for giving it input to the network. For the model to be effective in this scenario the hyperparameters have to be initialized intuitively.

For instance our number of classes is set to `3` (excluding no light: Red, Green and Yellow.) We set our learning rate to be `1e-4` and the number of epochs at `2000`. Lastly we handle the data in batch sizes of `16`.



### Installation

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
* Download the [Udacity Simulator](https://github.com/davidawad/Capstone/releases/tag/v1.2).

### Usage

1. Clone the project repository
```bash
git clone https://github.com/davidawad/Capstone.git
```

2. Install python dependencies
```bash
cd Capstone
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


### Real world testing
1. Download [training bag](https://drive.google.com/file/d/0B2_h37bMVw3iYkdJTlRSUlJIamM/view?usp=sharing) that was recorded on the Udacity self-driving car
2. Unzip the file
```bash
unzip traffic_light_bag_files.zip
```
3. Play the bag file
```bash
rosbag play -l traffic_light_bag_files/loop_with_traffic_light.bag
```
4. Launch your project in site mode
```bash
cd Capstone/ros
roslaunch launch/site.launch
```



### Contributors



| name  | email  | username  |
|---|---|---|
| David Awad    |  contact@davidawad.com     | @davidawad  |
| Da Yang       |  da.yang@outlook.com  | @day  |
| Ashish Rana   |  ashish.rana.me@gmail.com  | @mr.jatt  |
| Michael Tien  |  mtien888@gmail.com   | @mtien888  |
| Naveed Ahmed |   naveedahmed04@gmail.com  | @nahmed |
