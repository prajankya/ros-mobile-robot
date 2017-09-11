# Creating URDF
URDF file is the robot description file in ROS. URDF stands for Unified Robot Description Format.

## Step 1
Create a basic file with chassis of the robot.

Create a file named `robot.urdf` in `mobile_robot_description` package, `urdf` folder
```xml
<?xml version="1.0"?>
<robot name="mobile_robot">
  <link name="base_link">
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <box size="0.6 0.4 0.10"/>
      </geometry>
      <material name="acrylic">
        <color rgba="1 1 1 0.8"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <box size="0.6 0.4 0.10"/>
      </geometry>
    </collision>
  </link>
</robot>
```

Test it by going into the folder `mobile_robot_description` and running

```bash
roslaunch urdf_tutorial display.launch model:=urdf/robot.urdf
```

## Step 2
Converting URDF into XACRO


XACRO is a preprocessor for creating URDF files with few features like variables and some basic logic calculations.

First change the folder name from `URDF` to `xacro`,

Then change the file name from `robot.urdf` to `robot.xacro`


Then change the first two lines of the file,

From
```xml
<?xml version="1.0"?>
<robot name="mobile_robot">
...
```
to
```xml
<robot xmlns:xacro="http://www.ros.org/wiki/xacro"  name="mobile_robot">
...
```

Then create a folder `launch` in the same package `mobile_robot_description` and create a file called `desc.launch`

put the following in the file,
```xml
<?xml version="1.0"?>
<launch>
  <arg name="model" default="$(find mobile_robot_description)/xacro/robot.xacro" />

  <param name="robot_description" command="$(find xacro)/xacro --inorder $(arg model)" />
</launch>
```

Then run this launch file by running
```bash
roslaunch mobile_robot_description urdf.launch
```

Then in other terminal run
```bash
rosrun rviz rviz
```

Then in rviz click on add button in left bottom, adn add `RobotModel`

Now in Left `Display` pane in `Global Options` change the `Fixed frame` from `map` to `base_link`. (you need to type base_link in place of map)

## Step 3
Adding wheels

now we will define a wheel,

in the same `robot.xacro` file, before `base_link`, we will define a _macro_
```xml
...
<xacro:macro name="wheel">
  <visual>
    <origin xyz="0.02 0 0" rpy="0 0 ${pi/2}"/>
    <geometry>
      <box size="0.6 0.4 0.10"/>
    </geometry>
  </visual>
  <inertial>
    <mass value="0.5"/>
    <inertia ixx="0.0" ixy="0.0" ixz="0.0" iyy="0.000001" iyz="0.0" izz="0.0"/>
  </inertial>
  <collision>
    <origin xyz="0.027 0 0.0" rpy="0 ${pi/2} 0"/>
    <geometry>
       <cylinder length="0.045" radius="0.08"/>
    </geometry>
  </collision>
</xacro:macro>
...
```

Now add these lines below the `base_link` link in `robot.xacro`

```xml
<xacro:wheel name="wh_left_back" x="1" y="-1"/>
<xacro:wheel name="wh_right_back" x="-1" y="-1"/>
<xacro:wheel name="wh_left_front" x="1" y="1"/>
<xacro:wheel name="wh_right_front" x="-1" y="1"/>
```

Also save the config of current settings in rviz with `RobotModel` in the folder `config` in `mobile_robot_description` with the name `rviz_config.rviz`



Next, create a file named `state_publisher.launch` in the launch folder of `mobile_robot_description` with the following contents

```xml
<?xml version="1.0"?>
<launch>
  <arg name="use_gui" default="false"/>
  <param name="use_gui" value="$(arg use_gui)"/>

  <node pkg="robot_state_publisher" type="robot_state_publisher" name="robot_pub" />
  <node pkg="joint_state_publisher" type="joint_state_publisher" name="joint_pub" />
</launch>
```

We would create few more launch files,

`rviz.launch` in the launch folder of `mobile_robot_description` with the following contents

```xml
<?xml version="1.0"?>
<launch>
  <arg name="config" default="rviz_config" />

  <node name="rviz" pkg="rviz" type="rviz" required="true" args="-d $(find mobile_robot_description)/config/$(arg config).rviz" />
</launch>
```

And finally a main launch file, `robot.launch` in `mobile_robot` package, in `launch` folder with the following contents

```xml
<?xml version="1.0"?>
<launch>
  <arg name="viz" default="true"/>
  <arg name="gui" default="false"/>

  <include file="$(find mobile_robot_description)/launch/desc.launch"/>
  <include file="$(find mobile_robot_description)/launch/state_publisher.launch">
    <arg name="use_gui" value="$(arg gui)"/>
  </include>

  <group if="$(arg viz)">
    <include file="$(find mobile_robot_description)/launch/rviz.launch"/>
  </group>
</launch>
```
