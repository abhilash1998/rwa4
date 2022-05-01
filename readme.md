## License
```
MIT License

Copyright (c) 2021 Abhilash Mane, Atharva Paralikar, Rajan Pande, Robert Vandemark, Sameep Pote

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
```

## Overview

## Architecture

![Architecture Diagram](https://github.com/Sameep2808/group_3_rwa1/blob/main/docs/figures/Architecture-Diagram.png)

## Sequence Diagram

![Sequence Diagram-Kitting Procedure](https://github.com/Sameep2808/group_3_rwa1/blob/main/docs/figures/Sequence-Kitting.png)
![Sequence Diagram-Assembly Procedure](https://github.com/Sameep2808/group_3_rwa1/blob/main/docs/figures/Sequence-Assembly.png)

## Class UML Diagram

![Class UML Diagram](https://github.com/Sameep2808/group_3_rwa1/blob/main/docs/figures/UML_Class-Diagram.png)

## Dependencies and Technologies used

- Programming language : C++
- Build system : catkin
- Operating System : Ubuntu 18.04
- Packages : nist_gear, roscpp, rospy, sensor_msgs, std_msgs, std_srvs, trajectory_msgs

## Steps to Build package
1. Navigate to ARIAC Workspace
```
cd ~/ariac_ws/src 
```
2. Copy the repository in src folder of catkin workspace
```
git clone --recursive https://github.com/Sameep2808/group_3_rwa1.git
cd ..
catkin build
```
## Steps to Run package
1. Make sure you have sourced setup file
```

source ~/ariac_ws/devel/setup.bash
```

2. To run the entire package 
```
roslaunch group_3_rwa1 main.launch
```

## To run just the sensor subscribing node
1. Launch the sample Environment
```
roslaunch nist_gear sample_environment.launch
```
2. Run the sensor node
```
rosrun group_3_rwa1 sensors_controller
```
3. Start the Competition
```
rosservice call /ariac/start_competition
```

## To run just the AGV node
1. Launch the sample Environment
```
roslaunch nist_gear sample_environment.launch
```
2. Run the sensor node
```
rosrun group_3_rwa1 agvs_controller
```
3. Start the Competition
```
rosservice call /ariac/start_competition
```



## Making Doxygen documentation

This generates a index.html page in the build/coverage sub-directory that can be viewed locally in a web browser.
How to Generate Doxygen Documentation

To install doxygen run the following command:
```
sudo apt-get install doxygen

Now from the cloned directory run:

doxygen doxygen
```
Generated doxygen files are in html format and you can find them in ./docs folder. With the following command
```
cd docs
cd html
google-chrome index.html
```

