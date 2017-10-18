# Making the Robot Generic

Now we would make our robot to work for any future projects.

# Creating a teleop file

Create a file named `mobile_robot/launch/teleop.launch` with the following contents.

```xml
<?xml version="1.0" encoding="UTF-8"?>
<launch>
  <arg name="gui" default="false"/>

  <group if="$(arg gui)">
    <node pkg="rqt_robot_steering" type="rqt_robot_steering" name="steering" ns="mobile_robot"/>
  </group>

  <group unless="$(arg gui)">
    <node pkg="turtlesim" type="turtle_teleop_key" name="turtle_steering" ns="mobile_robot" output="screen">
      <remap from="/mobile_robot/turtle1/cmd_vel" to="/mobile_robot/cmd_vel"/>
    </node>
  </group>
</launch>
```

Now edit the file `mobile_robot_simulation/launch/state_publisher.launch` replace all the contents with the following,

```xml
<?xml version="1.0"?>
<launch>
  <node pkg="robot_state_publisher" type="robot_state_publisher" name="robot_pub" ns="mobile_robot">
    <remap from="/mobile_robot/joint_states" to="/joint_states"/>
    <param name="publish_frequency" type="double" value="30.0"/>
  </node>
</launch>
```

Now edit the `robot.xacro` file and __edit__ the plugin we added for `differential_drive_controller` and edit the following tags so that the values are same as follows
```xml
...
<commandTopic>/cmd_vel</commandTopic>
<odometryTopic>/mobile_robot/odom</odometryTopic>
<odometryFrame>odom</odometryFrame>
<robotBaseFrame>base_link</robotBaseFrame>
...
```

And finally update `mobile_robot_simulation/launch/gazebo.launch` now __update__ as following

```xml
...
<rosparam command="load" file="$(find mobile_robot_simulation)/config/diffdrive.yaml" ns="mobile_robot"/>

<node name="mobile_robot_controller_spawner" pkg="controller_manager" type="spawner" args="mobile_robot_joint_state_controller mobile_robot --shutdown-timeout 3"/>
...
```
__Notice__ the `ns="mobile_robot"` part.

Now we can run with the following in one terminal

```bash
roslaunch mobile_robot robot.launch
```

and in another terminal
```bash
roslaunch mobile_robot teleop.launch
```

Now you can controller the robot from this terminal using arrow keys



> /mobile_robot/cmd_vel
