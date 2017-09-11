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

now we will define a wheel,

in the same file, before `base_link`, we will define a _macro_
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
