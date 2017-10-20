# Adding a STL file for 3D sensor

## Step 1
Download the STL

go to
> [https://github.com/prajankya/ros-mobile-robot/blob/master/mobile_robot_description/meshes/kinect.stl](https://github.com/prajankya/ros-mobile-robot/blob/master/mobile_robot_description/meshes/kinect.stl)

Download the stl using the download button on top right corner of the preview window and put it in a folder named `meshes` in the package `mobile_robot_description`.

## Step 2
Update the `robot.xacro` file for adding the stl

Add the following lines in the `robot.xacro` file after the `<xacro:wheels ..>` tag ends and before the comment of starting of `gazebo` section.

```xml
...

  <!-- =================== Kinect ==================== -->
  <link name="kinect_link">
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <mesh filename="package://mobile_robot_description/meshes/kinect.stl" scale="0.01 0.01 0.01"/>
      </geometry>
      <material name="kinect">
        <color rgba="1 0.9 0.9 1"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <box size="0.28 0.07 0.08"/>
      </geometry>
    </collision>
    <xacro:default_inertial mass="0.2"/>
  </link>

  <joint name="base_link_kinect_link" type="fixed">
    <origin xyz="0.265 0 0.087" rpy="0 0 ${pi/2}"/>
    <parent link="base_link"/>
    <child link="kinect_link"/>
  </joint>
...
```
__Note:__  The `scale` attribute in `<mesh .. />` tag, as the model was made in centimeters, we scale it to meters, as ROS uses SI units.

Now run using

```bash
roslaunch mobile_robot robot.launch
```

And in another terminal

```bash
roslaunch mobile_robot teleop.launch
```
