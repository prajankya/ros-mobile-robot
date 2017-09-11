# Converting URDF
Gazebo is a simulation software which uses SDF which is very similar to URDF.
But as gazebo is not a directly inside ros, it has to be linked with ROS.

## Step 1
Add ros_control in gazebo in file `robot.xacro`

Add these line at the bottom of the xacro file, just before closing the `robot` tag
```xml
...
<!-- =================== Gazebo ==================== -->
<gazebo>
   <plugin name="gazebo_ros_control" filename="libgazebo_ros_control.so">
     <robotNamespace>/</robotNamespace>
   </plugin>
 </gazebo>
<plugin name="differential_drive_controller" filename="libgazebo_ros_diff_drive.so">
  <alwaysOn>true</alwaysOn>
  <updateRate>20</updateRate>
  <leftJoint>base_link_wh_left_back</leftJoint>
  <rightJoint>base_link_wh_right_back</rightJoint>
  <wheelSeparation>0.5</wheelSeparation>
  <wheelDiameter>0.1</wheelDiameter>
  <torque>20</torque>
  <commandTopic>cmd_vel</commandTopic>
  <odometryTopic>odom</odometryTopic>
  <odometryFrame>odom</odometryFrame>
  <robotBaseFrame>base_footprint</robotBaseFrame>
</plugin>
...
```

## Step 2
Add Inertia values for all links

Comment all the references(X and Y spheres) using `<!--` and `-->` so as to not define any inertia for those.

Now a macro, just about the wheel macro we had created earlier
```xml
...
<xacro:macro name="default_inertial" params="mass">
  <inertial>
    <mass value="${mass}" />
    <inertia ixx="1.0" ixy="0.0" ixz="0.0" iyy="1.0" iyz="0.0" izz="1.0" />
  </inertial>
</xacro:macro>
...
```

Now in the wheel macro, after the `collision` tag ends, add,
```xml
...
<xacro:default_inertial mass="0.6"/>
...
```

Similarly for base link, after the `collision` tag ends,

```xml
...
<xacro:default_inertial mass="6"/>
...
```

## Step 3
Create Gazebo launch file

Create `gazebo.launch` in `launch` folder in `mobile_robot_simulation` package, with the contents,
```xml
<?xml version="1.0"?>
<launch>
  <arg name="paused" default="false"/>
  <arg name="use_sim_time" default="true"/>
  <arg name="gui" default="true"/>
  <arg name="headless" default="false"/>
  <arg name="debug" default="false"/>

  <!-- push robot_description to factory and spawn robot in gazebo -->
  <node name="urdf_spawner" pkg="gazebo_ros" type="spawn_model"
	        args="-z 1.0 -unpause -urdf -model robot -param robot_description" respawn="false" output="screen" />

  <include file="$(find gazebo_ros)/launch/empty_world.launch">
      <arg name="debug" value="$(arg debug)" />
      <arg name="gui" value="$(arg gui)" />
      <arg name="paused" value="$(arg paused)"/>
      <arg name="use_sim_time" value="$(arg use_sim_time)"/>
      <arg name="headless" value="$(arg headless)"/>
  </include>
</launch>
```

## Step 4
Update the `robot.launch` in `mobile_robot` package

Add this after the two existing _args_
```xml
...
<arg name="sim" default="true" />
...
```

and before closing the launch file, add this group,
```xml
...
<group if="$(arg sim)">
  <include file="$(find mobile_robot_simulation)/launch/gazebo.launch"/>
</group>
...
```

Now we can check the robot in both RViz and Gazebo by running the following in a terminal

```bash
roslaunch mobile_robot robot.launch
```
