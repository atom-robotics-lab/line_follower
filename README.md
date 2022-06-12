# Line_follower

This is line follower robot based on OpenCV(computer vision).  
# Introduction
In this project, we use OpenCV to capture video from the Image sensor on the bot and then process the image by __masking__ the path and drawing __Contour & Momments__ of the masked image. Now in bot_control class we use __P(proportional control)__ to correct the deviation of the bot from the path and publish them into the __/cmd_vel__ topic which helps the bot to take a turn.

<img src = "https://github.com/atom-robotics-lab/line_follower/blob/main/Assets/work_flow.png" >

#

# Installation

## Pre-Requisites:
- ROS noetic : Refer to the [official documentation](http://wiki.ros.org/noetic/Installation/Ubuntu) for installation of ROS noetic.
               
- Catkin workspace : A catkin workspace is a folder where you modify, build, and. install catkin packages. Take a look at the [official documentation](http://wiki.ros.org/catkin/Tutorials/create_a_workspace) for instructions regarding creation of a catkin workspace


## Installation of Virtualenvwrapper, OpenCV, and CV_bridge

Your can refer to [A.T.O.M's wiki](https://atom-robotics-lab.github.io/wiki/setup/virtualenv.html) for installation of the above mentioned packages and libraries.


## Clone the Line Follower package
Now go ahead and clone this repository inside the "src" folder of the catkin workspace you just created by executing the command given below in your terminal.
```bash
git clone git@github.com:atom-robotics-lab/line_follower
```


## Clone the MR-Robot package
This package provide us the bot which we are gonig to use.
Go inside the "src" folder of the catkin workspace and executing the command given below in your terminal.
```bash
git clone git@github.com:atom-robotics-lab/MR-Robot.git
```
### Note:

Now out robot does not have camera in this package so we have to chang branch from main to with_camera.   
Now then go inside MR-Robot package you just created by executing the above command then executing the command given below in your terminal.
```bash
git checkout with_camera 
```
__Hurray!!! All installation is done__

# Make the package
We'll need to "make" everything in our catkin workspace so that the ROS environment knows about our new package.  (This will also compile any necessary code in the package). Execute the given commands in your terminal.

```bash
cd ~/catkin_ws
catkin_make
```

# Launch

__launching world__

```bash
roslaunch line_follower bot_world.launch
```
The above command when executed in the terminal will launch the gazebo simulation and will also start ROS Master.

<img src = "https://github.com/atom-robotics-lab/line_follower/blob/main/Assets/launch.png" >


__Run script__

```bash
rosrun line_follower line2.py
```

The given command will run the controller script which controls the robot's movements.

<img src = "https://github.com/atom-robotics-lab/line_follower/blob/main/Assets/line_follower.gif" >




