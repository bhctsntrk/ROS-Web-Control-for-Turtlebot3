#Turtle Bot Web Control 
======================

#### Installation
Firstly you must install turtlebot3, rosbridge server and web video server.
```bash
sudo apt-get install "ros-kinetic-turtlebot3*"
sudo apt-get install ros-kinetic-rosbridge-server
sudo apt-get install ros-kinetic-web-video-server
```
then clone this repo
```bash
git clone https://github.com/bhctsntrk/TurtleBotWebControl
```

#### Usage with turtlebot3 simulation

```bash
export TURTLEBOT3_MODEL=waffle
roslaunch turtlebot3_gazebo turtlebot3_world.launch
roslaunch rosbridge_server rosbridge_websocket.launch
roslaunch turtlebot3_navigation turtlebot3_navigation.launch
roslaunch turtlebot3_slam turtlebot3_slam.launch
rosrun web_video_server web_video_server
rosrun robot_pose_publisher robot_pose_publisher
```
then start index.html with your browser and start to use
I use a lot of javascript code from robot web tools
http://robotwebtools.org/tools.html







