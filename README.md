# Assignment 1 of the Experimental Robotics course (MSc Robotics Engineering, Unige)

The following repository contains a ROS package for a toy simulation of Clauedo game in which a robot explore the environment for hints to find the killer. The environment in this project is an appartment with three rooms in which robot enter one by one to find hints. Based on the discovered hints robot deduces a hypotheses regarding the killer. The deduced hypotheses by robot has to be consistent. A hypotheses will be consistent if it is based on three different types of hints.

1. Person: Robot can find a name of a person as a hint which can be a killer's name e.g: Prof. Plum.
2. Weapon: Robot can find a name of a weapon as a hint which killer might have used e.g: Dagger.
3. Place: Robot can find a name of random place where the crime might been committed e.g: Hall.

Incase the deduced hypotheses is wrong then the robot visits the three rooms again for new hints until it forms a consistent hypotheses.

                           R2(2,0)
                           |
                           |
                           |
                           |   P(1,1)      
(-2,0)R2 ----------------- O -------------------- R1(2,0)
                           | (0,0)
                           | 
                           | 
                           | 
## Project Installation:

This project requires both ROS1 and ROS2 to be install in the system. Please make sure you have both of them installed before following the instructions. There are three branches in this respository and each one of them should be install in the following fashion.

1. Code available in **Main** branch is a ROS1 package should be install in ROS1 workspace {ros1_ws}/src and to deploy successfully run the following command.
```
catkin_make
cd devel/
source setup.bash
```
3. Code available in **ros2** branch is a ROS2 package should be install in ROS2 workspace {ros2_ws}/src and to deploy successfully run the following command.
```
colcon build --packages-select rt2assignment_package
cd install/
source local_setup.bash
```
5. Code available in **sourcefiles** branch is a set of source files that will be required to run the project. Please install them in your /root directory. Once the branch is cloned make all these files executable by running the following command for each of the bash files.
```
chmod +x <file_name>
```
5. This project requires both Gazebo and Vrep simulation environments, if they are not already installed in your system please install them by following given instructions: 

  **For Gazebo :** 
  
  * Please follow the instructions available on the respository.http://gazebosim.org/tutorials?tut=install_ubuntu
  
  **For Vrep/Coppeliasim :**
  
  * Download the PRO-EDU version from: http://www.coppeliarobotics.com/downloads.html
  * Vrep should be already integrated with ROS. You just need to launch the ROS master before running the V-REP (CoppeliaSim) software.
  * If there is any problem in building the plugin, you will need to recompile it by yourself: you can download it from here: CoppeliaRobotics/simExtROS (github.com).
  * Please first install xsltproc [(sudo) apt-get install xsltproc] and xmlschema [pip3 install xmlschema]
  * If specific messages/services/etc need to be supported, we have to make sure to edit files located in simExtROS/meta/, prior to recompilation. So please check if the following to message type is already included or not in the simExtROS/meta/message.txt 'geometry_msgs/Twist' 'nav_msgs/Odometry'
  * In order to build the packages, navigate to the catkin_ws folder and type:
    ```
    export COPPELIASIM_ROOT_DIR=~/path/to/coppeliaSim/folder
    ```
   
    ```
    catkin_make --cmake-args -DCMAKE_BUILD_TYPE=Release
    ```
    The packages should be generated and compiled to a library now. Copy the **devel/lib/libsimExtROS.so** file in the CoppeliaSim installation folder. The plugin is now ready     to be use.
    
  *  Now to launch Vep/CoppeliaSim you can run the following command. Make sure you are in the Vep/CoppeliaSim installation folder.

     ```
     ./coppeliasim.sh 
     ```
 5. Once this is done. Next dependency that need to install is ROS1/ROS2 bridge package. Please install the package available on this repository: https://github.com/ros2/ros1_bridge 
 6. Once setup 5 is done we need to modify our .bashrc file, to have the possibility of using both ROS1 and ROS2 frameworks at the same time (in different
terminal). For that please comment all bash files lines. 

 ```
 #source /opt/ros/noetic/setup.bash
 #source /root/my_ros/devel/setup.bash
 #source /opt/ros/foxy/setup.bash
 #source /root/my_ros2/install/local_setup.bash
 ```
 7. If you are successfully able to complete all the previous steps then all the dependencies are completed in order to run the project. 

The project in total have three parts, which can individually executed by following given instructions. 

## Part 1: Running Mobile Robot Simulation in Gazeboo using action server.

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

## Part 2: Running Mobile Robot Simulation in Gazeboo using ROS1/ROS2 Brigde

From users prespective part 2 of this project is very similar to part 1. One of the two differences are mainly architecture based. First main difference is that half of this part is build in ROS1 and the other half is build in ROS2. So we are using ROS1/ROS2 brigde to make these two sub-parts communicate with each other. The second difference is that in this part we are using a simple server to provide randomly generated goal coordinates to 'go_to_point' node instead of using action server like in part 1. Therefore unlike in part 1 robot will not immediately stop when we request to canncel the goal but after finishing the current goal. 

In ROS2 sub-part we have developed 'state_machine' and 'position_service' node as components, and by using the bridge, they are interface with the ROS1 nodes which is 'user_interface.py' and 'go_to_point.py' and also with the simulation in Gazebo.

In order to run this part please make sure you are in /root folder where you have already downloaded **rt2_assignment_1b.sh** bash file. Open the terminal and run the following command.

```
./rt2_assignment_1a.sh
```
After doing this you will see four terminal windows start appearing on the screen, including a Gazeebo simulation with a mobile robot in it. Wait for the system load all the files. Find the terminal window with title **user_interface** which will be asking to press 1. Upon pressing 1, the mobile robot will start moving towards the randomly generated goal target. And if during the execution of this goal if you press 0 the robot will not stop immediately since we are not using action server in this part. The robot will complete the last assigned target first before stopping.

## Part 3: Running Mobile Robot Simulation in Vrep/CoppeliaSim simulation

Part 3 of the project is also very similar with the part 1. The only difference is that we are using VRep/CoppeliaSim simulation environement instead of Gazebo. To create a communication channel between the vrep scene and 'go_to_point_vrep.py' node a subscriber and a publisher has been develop. The publisher publishes the robot current odometry data in '/odom' topic which 'go_to_point_vrep.py' node subscribes and use it as a feedback to check how far the robot is from the goal and based on that compute new velocity commands and publishes it in the '\cmd_vel' topic.


To run this part please following steps. 


1. Now make sure you are in /root folder where **'ros.sh'** is downloaded. open the terminal and run the following commands.
 
```
source ros.sh
```
2. Now go to the ROS1 workspace and run the master by using following command:
```
roscore &
```
3. Then go to the Vrep/CoppeliaSim installation folder and to launch Vrep/CoppeliaSim run the following command.

 ```
 ./coppeliasim.sh 
 ```
3. Wait for vrep simulation to get started. Then load the **'pioneerROS.ttt'** scene in the environment by clicking the **file/open scene..** button in the main menu. 
4. Once the scene is loaded start the simulation by clicking on the play button in the top center. Unlike Gazebo the simulation does not automatically start in the vrep simulator. 
5. To run the program open another terminal window, go to the root folder again and run the following commands.

```
source ros.sh
roslaunch rt2_assignment1 sim_vrep.launch
```
6. Once program is loaded it will be asking user to press 1. Upon pressing 1, the mobile robot will start moving towards the randomly generated goal target. And if during the execution of this goal if you press 0 the robot will not stop immediately. It will complete the last assigned target first and then stop.

 


