# Platform introduction

## Hardware 
The robotic system, AirBot, is equipped with a moving chassis and is connected to the upper robotic arm by a support structure. 
<div align="center">
  <img src="./assets/AIRBOT.png" width="40%">
</div>

* **[Horizon Robotics RDK x3](https://developer.horizon.cc/documents_rdk/)**

* **[Sunny Optical Technology  Mars04D](https://www.sunnyaiot.com/product_detail/Mars04D.html)**

* **[AgileX Robotics RGM2-Ranger mini2.0](https://github.com/agilexrobotics/AgileX-Robotics-all-products-user-manuals)**

## Ros Topic
The following content represents the definitions of topics in ROS. Participants can find the topics they need to subscribe to and view within these definitions

### Subscriber

Subscriber enables your node to read messages from a topic, allowing useful data to flow into the node. In Python, ROS subscribers typically have the following format: sub = rospy. Subscriber(“/topic_name”, message_type, callback_function)。

Among them, "/topic_name" indicates which topic the subscriber should listen to; message_type is the type of message posted on that topic; callback_function is the name of the function that is called for each incoming message. Every time a message is received, it is passed to the callback_function as a parameter. Typically, this feature is defined in your node to perform useful actions on incoming data.

|Topic name|Module| Parameter Type                                                                                                         |Description|
|:-- |:-- |:-------------------------------------------------------------------------------------------------------------|:-- |
|/cmd_vel |Chassis| geometry_msgs/Twist  <br>geometry_msgs/Vector3 linear  <br>geometry_msgs/Vector3 angular                     |    Set chassis speed     <br> 1. linear: Linear velocity in world coordinates，unit m/s  <br>2.angular: Angular velocity in world coordinates，unit rad/s|
|/airbot/base_pose_cmd|Chassis| geometry_msgs/Pose   <br>geometry_msgs/Point position  <br>geometry_msgs/Quaternion orientation              |Navigate chassis to world position  <br>position: position in world coordinates    <br>orientation: orientation in world coordinates, quaternion|
|/airbot_play/joint_cmd|robotic arm| std_msgs/Header  <br>string[ ] name  <br>float64[ ] position  <br>float64[ ] velocity <br> float64[ ] effort | Set the Angle, speed and effort position of the robotic arm joint:  lists of Joint Angle，unit rad velocity: List of joint angular velocities，unit rad effort:   ？Joint limit: unknown|
|/airbot_play/pose_cmd|robotic arm| geometry_msgs/PoseStamped  std_msgs/Header header(Ignored in sim)  geometry_msgs/Pose pose                   | Set the target movement position of the robotic arm end (the pinch point of the two-finger Gripper) (IK control) pose: The position of the Gripper at the end of the robotic arm relative to the base of the robotic arm|
|/airbot_play/gripper/state_cmd|Gripper| std_msgs/Bool                                                                                                |Mechanical Gripper opening and closing control|


### Publisher

Publisher enables your nodes to publish messages to topics, enabling other nodes to receive and read them.

|Topic name|Module|Parameter Type|Description|
|:-- |:-- |:-- |:-- |
|/airbot_play/gripper/current_state |Gripper|std_msgs/String|robotic arm gripper state. <br> open: open  <br> close: closed  <br> moving: Unsteady state|
|/camera/color/image_raw|Camera|sensor_msgs/Image|camera rgb data|
|/camera/color/camera_info|Camera|sensor_msgs/Camera Info - std_msgs/Header header  <br>  uint 32 height  <br> uint 32 width  <br> string distortion_model  <br> float64[] D  <br> float64[9] K  <br> float64[9] R  <br> float64[12] P  <br> uint 32 binning_x  <br> uint 32 binning_y  <br> sensor_msgs/RegionOfInterest roi| camera info data|
|/airbot_play/joint_states|robotic arm|sensor_msgs/JointState| robotic arm joints information|
|/airbot_play/current_pose|robotic arm|geometry_msgs/PoseStamped <br> std_msgs/Header header<br> geometry_msgs/Pose pose|robotic arm+ Second finger Gripper end gesture <br> pose: The position of the Gripper at the end of the robotic arm relative to the base of the robotic arm|
|/airbot/pose|Chassis|geometry_msgs/Pose <br> geometry_msgs/Point position <br> geometry_msgs/Quaternion orientation|The current stance of the chassis in the world system <br> position: Position in the world coordinate system <br>orientation: Orientation in world coordinate system, Quaternion|
|/object/names|Simulator|std_msgs/String|List of task object names|
|/object/poses|Simulator|geometry_msgs/PoseArray<br>std_msgs/Header header <br> geometry_msgs/Pose[] poses|List of current positions of mission objects (world coordinates)|
|/airbot/arm_base_pose|Chassis|geometry_msgs/PoseStamped <br>std_msgs/Header header(Ignored in sim)<br>geometry_msgs/Pose pose|Posture of the robotic arm base relative to the chassis: Posture of the robotic arm base relative to the chassis|
|/IMU_data|Chassis|sensor_msgs/Imu<br>- std_msgs/Header header(Ignored in sim)<br>geometry_msgs/Quaternion orientation<br>- float64[9] orientation_covariance<br>geometry_msgs/Vector3 angular_velocity<br>- float64[9] angular_velocity_covariance<br>- geometry_msgs/Vector3 linear_acceleration<br>- float64[9] linear_acceleration_covariance<br><br>|Orientation: position(rpy), relative to the robot's initial position <br>Angular velocity: Angular velocity at the current moment <br>Linear acceleration: The frame_id corresponding to the linear acceleration at the current moment is imu_link|
|/livox/lidar|Chassis|sensor_msgs/PointCloud2|The point cloud data returned by the radar, the frame_id corresponding to the coordinates of laser_link is livox_frame|


### coordinate relationship of robot joints

When it comes to the joint coordinate relationship of the robot, we are referring to the relative position and angle between the various joints of the robot. These relationships determine the robot's overall posture and movement capabilities.

The robot's joint coordinates are usually represented using the Cartesian coordinate system, with each joint having a specific coordinate system that describes the position and orientation of that joint. The relative positions and angles between joints can be represented by coordinate transformations that describe the transformation from one joint coordinate system to another.


|Topic name| Coordinate                   |Offset(x, y, z)|Description|
|:-- |:-------------------------|:-- |:-- |
|car_base_link | car_base_link            |(0, 0, 0)||
|arm_base| car_base_link            |(0.2975, -0.17309, 0.3488)||
|livox_frame| car_base_link            |(-0.31686, 0.0, -0.26705)||
|imu_link| car_base_link            |(0, 0, 0)||
|camera| custome_end_link(ee_point)|(-0.093, 0.0, 0.07)||
|camera| car_base_link                  |Not static transform||
