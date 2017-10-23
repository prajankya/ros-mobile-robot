# Adding sensor plugin in Gazebo

## Step 1
Update xacro file

Now we will add a link and the gazebo plugin for 3D depth sensor. Add the following just after the joint of `kinect baselink` and before the comment of `<!-- gazebo -->`

```xml
...
<!-- =================== Gazebo - Kinect ==================== -->
<link name="gz_kinect_link">
  <visual>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <geometry>
      <sphere radius="0.05"/>
    </geometry>
    <material name="kinect">
      <color rgba="1 0.9 0.9 1"/>
    </material>
  </visual>
  <collision>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <geometry>
      <sphere radius="0.05"/>
    </geometry>
  </collision>
  <xacro:default_inertial mass="0.2"/>
</link>

<joint name="kinect_link_gz_kinect_link" type="fixed">
  <origin xyz="0 0 0" rpy="0 0 -${pi/2}"/>
  <parent link="kinect_link"/>
  <child link="gz_kinect_link"/>
</joint>

<gazebo reference="gz_kinect_link">
  <sensor type="depth" name="kinect">
    <pose>0 0 0 0 0 0</pose>
    <always_on>1</always_on>
    <visualize>true</visualize>

    <camera>
      <horizontal_fov>1.047</horizontal_fov>
      <image>
        <width>640</width>
        <height>480</height>
        <format>R8G8B8</format>
      </image>
      <depth_camera></depth_camera>
      <clip>
        <near>0.1</near>
        <far>100</far>
      </clip>

      <noise>
        <type>gaussian</type>
        <mean>0.0</mean>
        <stddev>0.7</stddev>
      </noise>
    </camera>

    <plugin name="camera_controller" filename="libgazebo_ros_openni_kinect.so">
      <!-- <baseline>0.2</baseline> -->
      <alwaysOn>true</alwaysOn>

      <updateRate>10.0</updateRate>
      <cameraName>kinect_sensor</cameraName>
      <frameName>gz_kinect_link</frameName>

      <imageTopicName>/mobile_robot/rgb/image_raw</imageTopicName>
      <depthImageTopicName>/mobile_robot/depth/image_raw</depthImageTopicName>
      <pointCloudTopicName>/mobile_robot/depth/points</pointCloudTopicName>

      <cameraInfoTopicName>/mobile_robot/rgb/camera_info</cameraInfoTopicName>
      <depthImageInfoTopicName>/mobile_robot/depth/camera_info</depthImageInfoTopicName>

      <pointCloudCutoff>0.4</pointCloudCutoff>
      <hackBaseline>0.07</hackBaseline>
      <distortionK1>0.001</distortionK1>
      <distortionK2>0.001</distortionK2>
      <distortionK3>0.001</distortionK3>
      <distortionT1>0.001</distortionT1>
      <distortionT2>0.001</distortionT2>
      <CxPrime>0</CxPrime>
      <Cx>0</Cx>
      <Cy>0</Cy>
      <focalLength>0</focalLength>
    </plugin>
  </sensor>
</gazebo>
...
```

Another small change would be to increase the mass of `base_link` as it is not staying well on ground,
```xml
...
<xacro:default_inertial mass="20"/>
...
```

## Step 2
Update DiffDriveController

Now update the file `mobile_robot_simulation/config/diffdrive.yaml`,
__Change__ the `max_velocity` and `max_acceleration` parameters as follows
```yaml
...
linear:
  x:
    has_velocity_limits    : true
    max_velocity           : 5.0   # m/s
    has_acceleration_limits: true
    max_acceleration       : 1.0   # m/s^2
angular:
  z:
    has_velocity_limits    : true
    max_velocity           : 8.0   # rad/s
    has_acceleration_limits: true
    max_acceleration       : 2.0   # rad/s^2
...
```

## Step 3


Lastly add the following in `mobile_robot_simulation/launch/state_publisher.launch` with the `<launch />` tag

```xml
...
<node pkg="tf" type="static_transform_publisher" name="kinect_tf_broadcaster" args="0 0 0 0 3.1459 1.57 /kinect_link /gz_kinect_link 4"/>
...
```



Now you can check everything by running,
```bash
roslaunch mobile_robot robot.launch
```

And in another terminal

```bash
roslaunch mobile_robot teleop.launch
```

Put some obstacles before the bot in gazebo and then control the robot using arrow keys in the teleop terminal.
