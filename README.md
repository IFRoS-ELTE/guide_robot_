# **Guide Robot Using Scout mini Agilex Robot**
___
The objective of this project is to make an interactive guide robot in the university which helps to arrive people to their required place all over the university. It is an interactive robot which autonomously navigates to the required place.

This readme file contains the instructions for running the project either in simulation or in the real robot.

## **Simulation**

The simulation package includes the planning algorithm that we implemented using RRT with Dubins and also it has the fusing between the **odometry**, **IMU**, and **GPS** data to localize the robot. 

### **Dependencies**
___
In order to run the simulation package; some packages should be previously installed:

* [Octomap Package](https://github.com/OctoMap/octomap): In order to provide a  3D volumetric map from sensor data which is the laser scan.
* [Tracking PID Package](https://github.com/nobleo/tracking_pid): Offers a tuneable PID control loop to accurately follow a trajectory which will be used in the implemented planning algorithm.
* [Timed roslaunch Package](https://github.com/MoriKen254/timed_roslaunch): Delay the launch of a roslaunch file.
*  [Rviz Satellite Package](https://github.com/nobleo/rviz_satellite): Provide a plugin in rviz in order to import the google map.
* Robot_Localization Package:
```
sudo apt-get install ros-distro-robot-localization
```

### **Usage for the planning**
___
To run the planning algorithm which is the RRT with Dubins in our environment that includes buildings and walking people in the simulation.

In order to run the simulation with robot spawned in rviz and gazebo run this command:

```
roslaunch guide_robot planning.launch
```

Then in other terminal run the following command to use the RRT with Dubins to give a specific goal to the robot to reach it:

```
rosrun guide_robot global_planner_node.py
```

Then in RVIZ use the **2D Nav Goal** to select a specific goal then the robot will plan a path to reach it.

#### **Results of the planning algorithm**
___
After launching the **planning.launch**, the robot will be launched in the environment.As shown in rviz, the octomap provides the map with the free cells in grey and the obstacles in black.

![Rviz+Gazebo](https://user-images.githubusercontent.com/74091020/216615686-17189048-0b53-4cd0-aa10-79589a89593d.jpg)

Then after running the planning node, the robot will start to create a path in order to reach the goal. As shown in the following video, the path is created a s a marker in green.

https://user-images.githubusercontent.com/74091020/216616903-9d3ae9ae-f779-4373-965d-cdb6119a8f15.mp4

### **Usage for the localization package**
___
In order to test the localization package that fuse the sensor data use the following command:

```
roslaunch guide_robot guide_robot_launch.launch
```

We add some noise to the odom in order to mimic the noisy odometry that can be obtained form real sensors, so run this node to get the noisy odometry:

```
rosrun guide_robot noisy_odom.py
```
Then publish in the terminal some velocity messages to let the robot move in circles:

```
rostopic pub /cmd_vel geometry_msgs/Twist "linear:
 x: 0.5
 y: 0.0
 z: 0.0
angular:
 x: 0.0
 y: 0.0
 z: 0.5"
```
#### **Results of the localization**
___

https://user-images.githubusercontent.com/74091020/216622395-11e4b3ae-b23d-40c1-a2ea-e723e013f8d8.mp4

Now, the robot is fusing all the data from **noisy_odom**, **IMU**, and **GPS**; the output from this package is a **filtered odometry** which give the exact location of the robot. As shown in the video that the red arrows is the noisy odom and the blue one is the filtered odom.

## **Importing Google Map in Rviz for Real Robot or Simulation**

https://user-images.githubusercontent.com/74091020/216624479-e3904a2f-cd8b-4557-9ab9-78a04c7e1733.mp4

To import google map in rviz, we should follow the next steps:

1. Create an empty map which contain **.yaml** and **.pgm** to put the google map.
2. Put the **AerialMapDisplay** plugin in Rviz.
3. Select the gps topic.
4. Create an access token for the google map from [MapBox](https://www.mapbox.com/) and put the object url in Rviz as follows:
   "http://a.tiles.mapbox.com/v4/mapbox.satellite/{z}/{x}/{y}.jpg?access_token=YOUR_ACCESS_Token"
5. Make sure that the robot has an internet access to import the map.

## **Real Robot**

### **Usage**
___

To use the real robot, first initialize the CAN bus to communicate all the devices of the robot together:

```
sudo ip link set can0 up type can bitrate 500000
```
Then, launch the minimal requirements of the robot to get the wheels ready:

```
roslaunch scout_bringup scout_minimal.launch
```
After that broadcast the transformations between the **base_footprint** and the **lidar**:

```
rosrun tf  static_transform_publisher 0 0 0.138 0 0 0 base_footprint velodyne 100
```
Afterwards, the lidar should be launched to receive the data from it:

```
roslaunch scout_bringup open_rslidar.launch
```
Then, the IMU and GPS should be launched also:

```
roslaunch xsens_mti_driver xsens_mti_node.launch
```

To launch the localization package in real robot:

```
roslaunch pomona_localization start_navigation_with_gps_ekf.launch
```
The previous command will launch the robot in a google map by using the information of the gps to get the location of the robot. 
But if there is no internet access; and need to test the localization, we can use the octomap to show the map for the real robot by using this commands:

```
rosrun scout_mini_gazebo laser_scan_to_point_cloud_node.py
```
```
roslaunch scout_mini_gazebo octomap_nav.launch
```
And then in Rviz, Add the **Map** and hide the **AerialMapDisplay** to only show the obtained map from octomap.

In order to navigate the robot either in the google map or by the octomap and giving it a specific goal use:

```
roslaunch scout_bringup move_base.launch
```

#### **Results of the Real Robot**
___

First a small test is done by fusing the odom and IMU data to localize the robot and as shown in the video, the **red arrows** are the noisy odom which is received from the real robot and the **golden arrows** are the filtered odom.

https://user-images.githubusercontent.com/74091020/216640251-cd19e509-eab5-434f-b287-bc9b5d7757e4.mp4


Also, a bag file is recorded around the CLC building after fusing the three sensors together. As shown from the video the noisy odom is in orange and the filtered one is in blue.

https://user-images.githubusercontent.com/74091020/216641769-9051e820-45b3-482f-b21d-799bb3d64779.mp4

After that a small demo is implemented by the real robot with the octomap and move_base in order to give the robot a specific goal to reach "Click on the image below :arrow_down:":

[![Real Robot Demo](https://user-images.githubusercontent.com/74091020/216639160-d7f44be5-b128-4029-865b-dc653d64c08e.png)](https://www.youtube.com/watch?v=TEqhd5YsFOk "Real Robot Demo")

## **Extras**

You can access a small presentation that contains more details about the project: [Rog_Team_Presentation](https://www.canva.com/design/DAFY9Or9-jg/wnfiJCWgsDDyCnZK-uRapA/view)
