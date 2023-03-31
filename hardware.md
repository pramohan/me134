## Hardware Design
We opted to construct an arm with 5 degrees of freedom and an electromagnetic end effector. There is a pan DOF at both the base and end effector, with 3 tilt DOFs chained together between them. As it has the highest torque requirement, we chose an X8-16 Nm motor for the base tilt joint. All other motors were X5-9 or X5-4 motors, as they required lower torque ranges. For the end effector, we chose a 5V electromagnet with a 2.5kg mass rating. The electromagnet was directly mounted to the bottom of the last panning motor to give us direct rotational freedom in its positioning. The use of an electromagnet limited us to making the game pieces out of thin 30 gauge steel sheet metal, which can be easily picked up and dropped by the electromagnet. The image below displays the constructed robot arm.
![image](https://user-images.githubusercontent.com/67039263/229213784-ab48e4ef-f839-447b-bd0b-535c2e5aee2d.png)

We designed the game board by laser cutting a 2ft x 2ft acrylic sheet. The board, pieces, and holes were distinct colors from both each other and the table. This enabled us to identify and classify gameboard components by color. The board is shown below.
![image](https://user-images.githubusercontent.com/67039263/229214478-f7bfac74-3563-4e37-9d3f-86cff03f7802.png)

For detecting contact between the game board and the piece during gameplay we chose to implement capacitive touch sensing which required copper tape, an ESP32 Wroom, and roughly 0.75m of 30 AWG wire. The ESP32 was chosen since we already had a board and it natively has capacitive touch pins to make measurements easy to read with one command. While this has not been integrated in the current system–given our priorities in getting more complex systems such as camera detection working–its isolated tests were successful.

For the manual control, there are two modes of operation. The first mode leverages 5 potentiometers in directly controlling the angles of each of the arm joints. The second mode uses a joystick to control the cartesian velocity of the tip of the robot. There is a toggle switch to switch between the two modes, as well as a toggle switch to change the z direction of the joystick (which is controlled by pressing down on the joystick). There are also 3 LEDs to indicate the status of the internal microcontroller as well as which mode is active at the time of operation. The control box is enclosed within a laser-cut acrylic box measuring 10x15x30 cm.
![image](https://user-images.githubusercontent.com/67039263/229214838-71733f45-5e82-41ec-9e93-c2ff21b9f35c.png)

The final hardware diagram is displayed below:
![image](https://user-images.githubusercontent.com/67039263/229214906-fbc07d53-b684-40cb-9e53-1de08e48b6b0.png)

[Go back](https://github.com/pramohan/me134)
