# Task 2: Getting Started with PX4

## Task 2.1

## Problem Statement
- You need to make the drone in Gazebo to follow 4 waypoints in the mission mode of px4
- Create a rosnode named waypoint_Mission in a python script, which will set the waypoints to be sent to px4
- You need to call the rosservice /mavros/cmd/arming to arm and /mavros/set_mode to set mode of the drone to mission mode
- Then you have to call the rosservice /mavros/mission/push and /mavros/mission/pull to Request parameter from device (or internal cache) and send parameters from ROS to FCU respectively.
- The waypoints are as follows:
    - Takeoff at the home position to 10 meters
    - Go to 19.134641, 72.911706, 10
    - Go to 19.134617, 72.911886, 10
    - Go to 19.134434, 72.911817, 10
    - Go to 19.134423, 72.911763, 10
    - Land at the last coordinate

## Resources

### Install QGroundControl
- On the command prompt enter:
```bash
sudo usermod -a -G dialout $USER
sudo apt-get remove modemmanager -y
sudo apt install gstreamer1.0-plugins-bad gstreamer1.0-libav gstreamer1.0-gl -y
```
- Logout and login again to enable the change to user permissions.
- Download [QGroundControl.AppImage](https://s3-us-west-2.amazonaws.com/qgroundcontrol/latest/QGroundControl.AppImage)
```bash
wget https://s3-us-west-2.amazonaws.com/qgroundcontrol/latest/QGroundControl.AppImage
```
- Install (and run) using the terminal commands:

```bash
chmod +x QGroundControl.AppImage
./QGroundControl.AppImage
```

## Procedure
- Complete the boiler plate script named waypoint_mission.py provided to you in the scripts folder
- Launch the Gazebo world by typing the following command in a terminal
```bash
roslaunch task_2 task2_1.launch
```
- Once the simulation window launches, you should see a drone in the gazebo environment.
- Run your python script in a separate terminal to start sending the waypoints and navigate the drone.
- As soon as you start the launch file, your python scripts should start running and the drone should arm, changes its mode to mission and then fly to the given coordinates and then land at the last coordinate.
- You can either run your python scripts manually or you can add the rosnode in the *task2_1.launch* file by adding the following lines before the ```<group>``` tag
```xml
<node name="waypoint_mission" type="waypoint_mission.py" pkg="task_2" />
```
To record ROS messages enable recording via rosbag utility:
```bash
roslaunch task_2 task2_1.launch record:="true" rec_name:="waypoint_mission.bag"
```
## Result
https://user-images.githubusercontent.com/72087882/142737636-8195dc1c-8dc4-473a-9b19-a8226289157e.mp4
