# 5-DoF Robot Arm for the Operation Board Game

Teammates' Github: kgauld1, harnett1, sravaniboggaram 

## Game Design
The project is based off of the game Operation. In this game, there is a board with cutouts of various shapes, each with matching game pieces that fit into the corresponding cutouts. In the classic version of the game, the goal is to remove all of the pieces from their cutouts without the player or the pieces touching the cutout edges.

Our game is a variation on Operation in which the player controls a robotic arm to remove or insert the pieces from the game board. In leveraging a robotic arm, we are also able to autonomously insert and remove pieces from the board. As such, the goal is to make a fully functional game in which the robot can autonomously control the board state, as well as respond to manual commands from a user operating a control box.

## Hardware Design
We opted to construct an arm with 5 degrees of freedom and an electromagnetic end effector. There is a pan DOF at both the base and end effector, with 3 tilt DOFs chained together between them. As it has the highest torque requirement, we chose an X8-16 Nm motor for the base tilt joint. All other motors were X5-9 or X5-4 motors, as they required lower torque ranges. For the end effector, we chose a 5V electromagnet with a 2.5kg mass rating. The electromagnet was directly mounted to the bottom of the last panning motor to give us direct rotational freedom in its positioning. The use of an electromagnet limited us to making the game pieces out of thin 30 gauge steel sheet metal, which can be easily picked up and dropped by the electromagnet.

![image](https://user-images.githubusercontent.com/67039263/229213784-ab48e4ef-f839-447b-bd0b-535c2e5aee2d.png)
The image above displays the constructed robot arm.

