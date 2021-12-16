# Task 3: Pick and Place

## Task 3.1

## Problem Statement
- The gazebo world consist of a drone and a strawberry box.
- You have to put the drone in Offboard mode and publish the positions of the drone as setpoints.
- The location of the box is 3m 0m 0m, the drone needs to do the following:
    - Takeoff and the initial position to the height of 3m
    - Go to the coordinate 3m 0m 3m
    - Land on the box and pick the box
    - Takeoff at the height of 3m and go to 3m 3m 3m
    - Land at 3m 3m 0m and drop the box
    - Takeoff again to height of 3m
    - Head back to the start position ie 0m 0m 3m
    - Finally land the drone at 0m 0m 0m and then disarm
- After landing on the box, you need to check that the drone is above the box in the allowable range to pick the box. To do that, you need to subscribe to the rostopic _/gripper_check_. If the value is true, you can now pick the box and if the value is false,the drone is not correctly placed and you need to correct the position.
- If the result of _/gripper_check_ is true, to pick the box, you need to call the rosservice _/activate_gripper_ and pass the value true to attach the box. If the box is attached, you will get a result from the rosservice as true. If the result is false, which means the drone is not correctly placed over the box.
- To detach the box, you need to use the same rosservice _/activate_gripper_ and pass the value false. This will detach the box from the drone.

## Procedure
- Write a python script named pick_n_place.py in the scripts folder and create a rosnode named pick_n_place in this python script.
- Launch the Gazebo world by typing the following command in a terminal
```bash
roslaunch task_3 task3_1.launch
```
- Once the simulation window launches, you should see a drone and a box in the gazebo environment.
- Run your python script in a separate terminal to start sending the setpoints and navigate the drone.

## Result
