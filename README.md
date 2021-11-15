# Assignment 1 of the Experimental Robotics course (MSc Robotics Engineering, Unige)

The following repository contains a ROS package for a toy simulation of Clauedo game in which a robot explore the environment for hints to find the killer. The environment in this project is an appartment with three rooms in which robot enter one by one to find hints. Based on the discovered hints robot deduces a hypotheses regarding the killer. The deduced hypotheses by robot has to be consistent. A hypotheses will be consistent if it is based on three different types of hints.

1. who: Robot can find a name of a person as a hint which can be a killer e.g: Prof. Plum.
2. what: Robot can find a name of a weapon as a hint which killer might have used e.g: Dagger.
3. where: Robot can find a name of random place where the crime might have been committed e.g: Hall.

Statement of a consistent hypothesis would be something like this: “Prof. Plum with the Dagger in the Hall”. Incase the deduced hypotheses is wrong then the robot will visit the three rooms again for new hints until it forms a consistent hypotheses. 

To deduced an hypothesis robot use the ARMOR package service which is developed by researchers at University of Genova. ARMOR is versatile management system that can handle single or multiple-ontology archetectures under ROS. Please find more details regarding ARMOR from here: https://github.com/EmaroLab/armor 

## Project Installation:

This project requires ROS with ARMOR package to be install in the system. Please make sure you have alrady install it before following the instructions. For installing ARMOR package you go to this link: https://github.com/EmaroLab/armor 

1. Code available in **Main** branch is a ROS package which should be place in ROS workspace {ros1_ws}/src after downloading.
2. To successfully deploy and build the package run the following command.
```
catkin_make
cd devel/
source setup.bash
```
3. In order to use the python modules contained in armor_py_api package run the following command to add the path of the armor python modules to your PYTHONPATH environmental variable.
``` 
export PYTHONPATH=$PYTHONPATH:/root/ros_ws/src/armor/armor_py_api/scripts/armor_api/
```

## Running the Project Simulation:


## Project Architecture:

Part1 of the project is consist of four main nodes. 

1. position_service 
2. state_machine_action 
3. go_to_point_action.py
4. user_interface_action.py

'user_interface_action' node communicates with the user and as per the provided commands, instruct the system to behave accordingly. If the user press 1 in the terminal, it request '/user interface' service which is host by 'state_machine_action.py' node. Letting the node know that user have request for mobile robot to move. Therefore, upon receiving the service request from 'user interface' node it request another service '/position server' which is hosted by 'position service' node to randomly generate goal coordinates for the robot to follow. Once it receives the goal coordinates in response to the earlier request to '/position service' service, it pass these goal coordinates to an action service "/go_to_point" which is host by 'go_to_point_action.py' node. Once ‘go_to_point_action.py’ node receives the goal coordinate it start computing required linear and angular velocity values for robot to reach that point and meanwhile start publishing it on ‘cmd_vel’ topic which subscribed by Gazeebo. Since ‘/go_to_point’ is an action service therefore the user has the option to request cancelling the goal at any point during the execution. And for this purpose the ‘user interface’ node ask the user to press 0 in order to cancel the goal. 

In order to run this part please make sure you are in /root folder where you have already downloaded **rt2_assignment_1a.sh** bash file. Open the terminal and run the following command.

```
./rt2_assignment_1b.sh
```
After doing this you will see three terminal windows start appearing on the screen, including a Gazebo simulation with a mobile robot in it. Wait for the system load all the files. Find the terminal window with title **user_interface** which will be asking to press 1. Upon pressing 1, the mobile robot will start moving towards the randomly generated goal target. And if during the execution of this goal if you press 0 the robot will stop immediately. 

6. Once program is loaded it will be asking user to press 1. Upon pressing 1, the mobile robot will start moving towards the randomly generated goal target. And if during the execution of this goal if you press 0 the robot will not stop immediately. It will complete the last assigned target first and then stop.


Contant Info: 
Author: Shozab Abidi
Email: hasanshozab10@gmail.com
