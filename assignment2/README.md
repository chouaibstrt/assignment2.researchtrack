# Research Track Assignment 2

### Objectives
Create a new package with three nodes
- (a) A node that implements an action client, allowing the user to set a target (x, y) or to cancel it. The node
also publishes the robot position and velocity as a custom message (x,y, vel_x, vel_z), by relying on the values
published on the topic /odom. Please consider that, if you cannot implement everything in the same node, you
can also develop two different nodes, one implementing the user interface and one implementing the publisher
of the custom message.
- (b) A service node that, when called, prints the number of goals reached and cancelled;
- (c) A node that subscribes to the robot’s position and velocity (using the custom message) and prints the
distance of the robot from the target and the robot’s average speed. Use a parameter to set how fast the
node publishes the information.
- Also create a launch file to start the whole simulation. Set the value for the frequency with which node (c) publishes
the information




  


## organization


This package have following files structure. 
- **src:** Folder to place the all the nodes of the project. There are total of four nodes in the src files.

  - *sub-files:* Nodes of the project.

- **launch:** Folder with the launch files for the robots.

- **srv:** Folder with the service files for the robots. 
  
- **msg:** Folder with the msg files for the robot.

- **Other files:** Folders and other files are also important such as CMakelist and package files.

    - ***include:*** Folder with the common module files, similar to the Libs.


### Running

To run this package, first run userinterface file then other launch files. 

``` 
roslaunch assignment_2_2022 assignment1.launch
```
- **Start user interface**
``` 
rosrun control client_node
```
Once node is started, follow the instruction on the terminal. 


- **Launch assign_solution.launch**
``` 
roslaunch control assign_solution.launch
```


## Pseudocode 


- **client_node**
- Action client is created which calls Action server "Planning".

- Ininite Loop

  - Press one then goal location from the user is taken

  - Press zero then Goal is cancelled
 
- Loop finished

- **client_sub_node**
- creating a publisher 'pub' and "Cbkodom is the callback function

  

  - define custum messages

- Subcriber is created called sub which subcribes "/odom"
