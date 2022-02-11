# prl_ur10e


Copy the project on the catkin workspace
```
$ cd catkin_ws
$ cd src
$ git clone
```

Install depenancies
``` 
$ rosdep update
$ cd ..
$ rosdep update rosdep install --rosdistro noetic --ignore-src --from-paths src
$ catkin_make
```

(1) Gazebo simulation + Control + Camera / Visual Processing
```
$ roslaunch armmp view_gazebo.launch
$ rosrun armmp gazebo_teleop.py
$ rqt_image_view
$ roslaunch find_object_2d find_object_2d.launch
```
 
 ![alt text](https://github.com/yw14218/prl_ur10e/blob/master/camera/Screenshot%20from%202022-01-17%2000-30-22.png)
 ![alt text](https://github.com/yw14218/prl_ur10e/blob/master/camera/.png)
 
(2) Integrated Gazebo + MoveIt Simulation
```
$ roslaunch armmp_moveit_config robot.launch
```
 
 ![alt text](https://github.com/yw14218/prl_ur10e/blob/master/camera/Screenshot%20from%202022-01-17%2000-36-33.png)
