# Create basic ROS packages

We need to create following packages in the catkin workspace directory
 - mobile_robot
 - mobile_robot_base
 - mobile_robot_description
 - mobile_robot_simulation
 - mobile_robot_work

We will start assuming you dont have any project yet in ROS
## Step 1
Go to your catkin workspace
```bash
mkdir catkin_ws
cd ~/catkin_ws/
```
> *Note :* `cd` command is for changing the directory(change/goto a folder). `mkdir` command is for creating a new folder/directory, i.e. `catkin_ws`.

## Step 2
Initialize the workspace

```bash
mkdir src
cd src
catkin_init_workspace
```
