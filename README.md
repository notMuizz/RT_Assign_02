# RT_Assignment_02

## Assignment Objective

In this assignment, a ROS package was developed to control a **Robot's** movement and gather information about its position and speed. The package includes the following features:

- An action client node that allows the user to set a target (x,y) position for the robot or cancel the current goal.
- A publisher node that publishes the robot's current position and velocity as a custom message  using the /odom topic.
- A service node that prints the number of goals reached and canceled by the robot. 
- A subscriber node that listens to the robot's position and velocity and prints the distance of the robot from the target and the robot's average speed. This node also uses a parameter to set the frequency at which it publishes the information.
- A launch file to start the simulation and set the frequency for the subscriber node.<br/>




![Screenshot from 2023-01-11 04-45-58](https://user-images.githubusercontent.com/48551115/211718216-c5359878-052d-4491-8543-d0c1db8dd183.png)

## Organization of Robot_package
This package have following files structure. Name of package is 'robot_r' and and inside the package, following is the structure of files.

- **src:** Folder to place the all the nodes of the project. There are total of **Four nodes** in the src files.
  - *sub-files:* Nodes of the project.
- **launch:** Folder with the launch files for the robots **assign_02.
- **srv:** Folder with the service files for the robots.   
- **msg:** Folder with the message files for the robot.

The following graph demonstrate the connection between nodes:


## Pacakge Directory
For running this file first install the ROS workspace and other required files. To install this package, just clone it inside your ROS workspace, running:

```
git clone https://github.com/masoodad/ws_researchTrack.git
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

![Screenshot from 2023-01-11 03-08-03](https://user-images.githubusercontent.com/48551115/211718427-559d5ab7-240f-49f9-9977-3a9eb7bf3587.png)


![Screenshot from 2023-01-11 05-53-26](https://user-images.githubusercontent.com/48551115/211720964-47cd5881-b9a6-4d51-a27e-9f64d80a6176.png)
![Screenshot from 2023-01-11 05-53-43](https://user-images.githubusercontent.com/48551115/211720972-12f26d68-37e5-4a55-a45d-2e72735148fc.png)
![Screenshot from 2023-01-11 05-54-52](https://user-images.githubusercontent.com/48551115/211720980-436af9ef-2e76-4620-9125-69b905f72d4d.png)


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
