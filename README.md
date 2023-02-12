# RT_Assignment_02

## Assignment Objective

In this assignment, a ROS package was developed to control a **Robot's** movement and gather information about its position and speed. The package includes the following features:

- An action client node that allows the user to set a target (x,y) position for the robot or cancel the current goal.
- A publisher node that publishes the robot's current position and velocity as a custom message  using the /odom topic.
- A service node that prints the number of goals reached and canceled by the robot. 
- A subscriber node that listens to the robot's position and velocity and prints the distance of the robot from the target and the robot's average speed. This node also uses a parameter to set the frequency at which it publishes the information.
- A launch file to start the simulation and set the frequency for the subscriber node.<br/>


![Screenshot from 2023-02-11 00-26-59](https://user-images.githubusercontent.com/123844091/218287220-838028bb-0fd9-4cdc-80c0-b2016882b2bf.png)



## Organization of Robot_package
This package have following files structure. Name of package is 'robot_r' and and inside the package, following is the structure of files.

- **src:** Folder to place the all the nodes of the project. There are total of **Four nodes** in the src files.
  - *sub-files:* Nodes of the project.
- **launch:** Folder with the launch files for the robots **assign_02.
- **srv:** Folder with the service files for the robots.   
- **msg:** Folder with the message files for the robot.

The following graph demonstrate the connection between nodes:
![Screenshot from 2023-02-11 22-16-29](https://user-images.githubusercontent.com/123844091/218287163-4325aefb-dded-41e3-b9c8-c0e5145bcd3d.png)



## Pacakge Directory
For running this file first install the ROS workspace and other required files. To install this package, just clone it inside your ROS workspace, running:

```
git clone https://github.com/notMuizz/RT_Assign_02.git
```
### Running

To run this package, open the terminal and first run userinterface file then other launch files. 

``` 
roslaunch robot_r  assign_02.launch
```
- **Start user interface**
``` 
rosrun robot_r client_node
```
Once node is started, follow the instruction on the terminal. Please find the screenshot for more details. 
![Screenshot from 2023-02-11 22-12-11](https://user-images.githubusercontent.com/123844091/218287266-8d4df396-17d9-4eb1-bdb9-6ab2f1ddceae.png)

![Screenshot from 2023-02-11 22-14-10](https://user-images.githubusercontent.com/123844091/218287292-844bfe16-c6b7-4942-9f03-ae32dde444d9.png)


## Pseudocode 
This sudo code is for only two nodes of package, **client_nod** and **client_sub_node**.

- **client_node**
- Action client is created which calls Action server "Planning".
- Ininite Loop
  - Press one then goal location from the user is taken
  - Press zero then Goal is cancelled
  - (For proper functionality of code please cancel goal after setting goal)
- Loop finished
- **client_sub_node**
- Publisher is created called pub with call back function "Cbkodom"
  - Inside Call back function 
  - Custom msg was defined    
- Subcriber is created called sub which subcribes "/odom"
