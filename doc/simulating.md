#Simulating in Gazebo

After the file has been loaded in the gazebo, now we will configure ROS and Gazebo for our model to be controlled through ROS

## Step 1
Add wheel Trasnmission

To drive the robot around, we specify a `transmission` tag for each of the wheels from within the wheel macro.

You need to add the following just before closing of the `wheel` _macro_ in the `robot.xacro` file

```xml
...
<gazebo reference="base_link_${name}_wheel">
  <mu1 value="200.0"/>
  <mu2 value="100.0"/>
  <kp value="10000000.0" />
  <kd value="1.0" />
  <material>Gazebo/Grey</material>
</gazebo>

<transmission name="base_link_${name}_wheel_trans">
  <type>transmission_interface/SimpleTransmission</type>
  <actuator name="base_link_${name}_wheel_motor">
    <mechanicalReduction>1</mechanicalReduction>
  </actuator>
  <joint name="base_link_${name}">
    <hardwareInterface>hardware_interface/VelocityJointInterface</hardwareInterface>
  </joint>
  </transmission>
...
```

## Step 2
Add Joint state Publisher from Gazebo

First add a file named `joints.yaml` in `config` folder in `mobile_robot_simulation` package, with the following contents,
```xml
type: "joint_state_controller/JointStateController"
publish_rate: 50
```

Then add a file named `state_publisher.launch` in `launch` folder of `mobile_robot_simulation` package, with the following contents,
```xml
<?xml version="1.0"?>
<launch>
  <node pkg="robot_state_publisher" type="robot_state_publisher" name="robot_pub">
    <param name="publish_frequency" type="double" value="30.0" />
  </node>
</launch>
```

Now Update `gazebo.launch` file in `mobile_robot_simulation` package,
adding the following after all the `arg` tags,

```xml
...
<rosparam command="load" file="$(find mobile_robot_simulation)/config/joints.yaml" ns="mobile_robot_joint_state_controller"/>

<node name="mobile_robot_controller_spawner" pkg="controller_manager" type="spawner" args="mobile_robot_joint_state_controller --shutdown-timeout 3"/>
...
```

Finally Update `robot.launch` file in `mobile_robot` package
Update the group with,
```xml
...
<group unless="$(arg sim)">
  <include file="$(find mobile_robot_description)/launch/state_publisher.launch">
    <arg name="use_gui" value="$(arg gui)"/>
  </include>
</group>
<group if="$(arg sim)">
  <include file="$(find mobile_robot_simulation)/launch/state_publisher.launch"/>
  <include file="$(find mobile_robot_simulation)/launch/gazebo.launch"/>
</group>
...
```

## Step 3
Adding Differential Drive

First we would add the config, create a file named `diffdrive.yaml` in `config` folder of `mobile_robot_simulation` package, with the contents

```xml
type: "diff_drive_controller/DiffDriveController"
publish_rate: 50

left_wheel: ['base_link_wh_left_back', 'base_link_wh_left_front']
right_wheel: ['base_link_wh_right_back', 'base_link_wh_right_front']

wheel_separation: 0.44

# Odometry covariances for the encoder output of the robot. These values should
# be tuned to your robot's sample odometry data, but these values are a good place
# to start
pose_covariance_diagonal: [0.001, 0.001, 0.001, 0.001, 0.001, 0.03]
twist_covariance_diagonal: [0.001, 0.001, 0.001, 0.001, 0.001, 0.03]

# Top level frame (link) of the robot description
base_frame_id: base_link

# Velocity and acceleration limits for the robot
linear:
  x:
    has_velocity_limits    : true
    max_velocity           : 0.2   # m/s
    has_acceleration_limits: true
    max_acceleration       : 0.6   # m/s^2
angular:
  z:
    has_velocity_limits    : true
    max_velocity           : 2.0   # rad/s
    has_acceleration_limits: true
    max_acceleration       : 6.0   # rad/s^2
```

Then add another rosparam in `gazebo.launch` file as follows,

```xml
...
<rosparam command="load" file="$(find mobile_robot_simulation)/config/diffdrive.yaml" ns="mobile_robot_diff_drive_controller"/>
...
```

And then update the controller in the same file from
```xml
<node name="mobile_robot_controller_spawner" pkg="controller_manager" type="spawner"
  args="mobile_robot_joint_state_controller --shutdown-timeout 3"/>
```
To
```xml
<node name="mobile_robot_controller_spawner" pkg="controller_manager" type="spawner"
  args="mobile_robot_joint_state_controller mobile_robot_diff_drive_controller --shutdown-timeout 3"/>
```

Now Run in a terminal

```bash
roslaunch mobile_robot robot.launch
```
To better visualize, you can change the `Fixed Frame` from `base_link` to `odom` in RViz on the Left side pane.
