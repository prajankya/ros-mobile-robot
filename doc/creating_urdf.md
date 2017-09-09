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
