# DiffBot
DiffBot is a differential drive robot with 12V dc motors + encoders built with Arduino UNO and Raspberry 3

-Motor driver:
You can use any motor driver such as L298N but you need to edit some parameters to make it work. I used a Cytron 10A 5-30V Dual Channel DC Motor Driver and Arduino code is made with this particular driver. I will later add L298N code if my schedule allows.

-Arduino UNO
-Raspberry 3

-Battery pack 
I made a battery pack consisting of x4 18650 and DC-DC stepdown converter to reduce voltage to 12,2 volts. This should provide plenty of power to use your robot without worrying about power consumption. I provided additional 0,2 volts to make up for lost power due to wiring and equalizing the dc motor voltages just in case.

-Chassis
I designed a chassis for my liking, I'll share my design once it is ready and as my schedule allows (or upon request)

-Wiring
I'll share as my schedule allows (or upon request)

-ROS
Easiest way to deal with the ROS part is to just head over ubiquityrobotics download page and flash your card with the image provided here:
https://downloads.ubiquityrobotics.com/pi.html

-Rosserial installation:
I do not recommend cloning from rosserial's github. If you want to go that path, just download the indigo branch (I couldn't find kinetic branch) or else catkin_make will fail. 

```
git clone -b indigo-devel https://github.com/ros-drivers/rosserial.git
```

I strongly suggest you install rosserial via

```
sudo apt install ros-kinetic-rosserial-arduino
sudo apt install ros-kinetic-rosserial-python
```

-Start rosserial using:
```
rosrun rosserial_python serial_node.py _port:=/dev/ttyACM0 _baud:=115200
```

-Test published topics in second terminal:

```
rostopic echo /left_ticks
rostopic echo /right_ticks
```

-Velocity control via:

```
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```

For ethical reasons I'd like to mention that I borrowed some Arduino code from Yoraish:
https://github.com/yoraish/lidar_bot

Problem with yoraish robot is that it uses only Lidar for odometry and navigation which is not reliable for many roboticist standards and dc motors used are without encoder.

ToDo's:

1-I will upload full ROS code to here once I tested everything.
2-Add Lidar
3-Add IMU module
4-Maybe add Kinect V2
