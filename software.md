[Go back](https://github.com/pramohan/me134)
# Software Setup

## Detectors
A Logitech web camera mounted above the table gave an aerial view of the game board.
 
The camera has multiple uses, one for reporting points of interest in the table frame, and another for detecting color contour boundaries. When the program is run, the 
camera waits and watches for 4 specific ARUCO tags before continuing. It then correlates the pixel position of the tags with their known table frame coordinates in order 
to compute a matrix that can compute the transform from camera coordinates to table coordinates. This process is run each time at startup, so it is robust to changes in 
table or camera location - as long as the ARUCOs are located at the same table coordinates, the position of the tags in the camera does not matter. Measuring the 
location of the tags with sub-centimeter accuracy in the table frame allowed the camera to return table frame coordinates of points of interest with sub-centimeter 
accuracy as well. We programmed our executable to wait until this camera setup is complete, so the robot will not move until all four ARUCOs are visible. An image of ARUCO detection is displayed below.

![image](https://user-images.githubusercontent.com/67039263/229218712-904c3884-c8e6-4bee-9ef0-86c78cd36927.png)

The other use of the camera is as a color contour detector. The pieces for our game were dark pink, and the board was placed on top of green felt, so that the holes could be detected by looking for green color. Finally, the board itself was blue. We tuned the HSV intervals of the detector until it could detect the boundaries of the pieces and holes with very little noise. This contour also allowed us to obtain its centroid, and thus the center of the piece. 

We attempted to obtain orientation of the pieces by using PCA; however, this only successfully worked for the bone-shaped piece. This method returned noisy and jittery angles for the other pieces. To get around this issue, we developed another method of obtaining orientation, which involved getting the line segment from the centroid to the point on the contour farthest from the centroid, and finding the angle of the segment. We were able to differentiate the bone from the other pieces using the ratio of the sides of the minimum area bounding rectangle, which was further from 1 because of its elongated shape. Thus, we could use PCA on the bone-shaped piece, and our custom orientation method on the other pieces. 

![image](https://user-images.githubusercontent.com/67039263/229219051-800eaffd-da2d-4147-9633-ab208f152795.png)

Additionally, we could use the cv2.matchShapes function to measure similarity between piece and hole contours. This allowed us to match pieces to their respective holes during autonomous board setup. In the picture below, we can see dissimilarity coefficients between the red star and each of the holes. This means that the smaller the number is, the better match it is to the star. As expected, the star shaped hole contains the smallest dissimilarity coefficient. 

![image](https://user-images.githubusercontent.com/67039263/229219438-ded799be-8974-4762-8ec8-eda986423412.png)

The blue color of the board was only used during the automated removal of pieces from the game board.  We fit a rectangle to the blue contour, and only remove pieces if their centroid falls within the bounds of the rectangle. 

## Micro-ROS
Our controller was implemented using micro-ROS. The micro-ROS framework allows us to create a node on an Arduino Nano RP2040 Connect which transmits state information over serial communication. To establish this connection, we built a micro-ROS agent on the NUC, which read in serial data from the node on the RP2040. This worked fine, but after some time began causing a data stream conflict with the Ethernet connection to the Hebi motors. Due to this conflict, we were forced to find an alternative way to connect the controller to the main ROS node on the NUC. To this effect, we flashed a Raspberry Pi 4 with Ubuntu Jammy 22.04 running ROS 2 Humble and created a WiFi link to the NUC over the robotnet network. By establishing an SSH connection between the NUC and Raspberry Pi, we were able to wirelessly transmit publishers and subscribers between the two computers.

All development on the Micro-ROS node was done in C++, using the rclcpp library to communicate with ROS and the Arduino library to read GPIO data. Code was flashed to the microcontroller using PlatformIO in VSCode, which allowed for flashing the microcontroller with the micro-ROS firmware.
We ran into two major issues with the RP2040 Connect. The first of these issues was due to the lack of available analog pins. The RP2040 Connect has 8 analog pins onboard, but only 4 of these pins are accessible for use, as the micro-ROS firmware locks the last 4 analog pins for wireless transport modes, which we did not use. Since we needed to use 5 potentiometers for joint control and 2 for the joystick, we needed to expand the number of analog ports available on the RP2040. As such, we added an MCP3008 IC to the system, which adds 8 analog inputs through an Analog-Digital Converter (ADC) communicating to the RP4040 through the Serial Peripheral Interface (SPI). 

Our second major issue was due to the limitations of micro-ROS. The micro-ROS firmware defines the controller as “very low mem”, allowing for no more than 2 publishers and 1 subscriber. Ideally, we would have had multiple binary publishers for each button, so this forced us to create 2 Int64 subscribers which contained all of the data for the various components in its digits. The first of these subscribers was for the potentiometer values, with 15 digits, with each potentiometer value ranging from 0 to 999. The second publisher published the joystick as the first 7 digits with 3 for each direction and 1 for the up/down button, followed by 3 digits for the button/toggle switch states and one digit for the state of the capacitive touch sensor.

## Manual Control
Our primary means of motion integration was done through the implementation of splines taking the robot through either task or joint space. We begin by integrating the manual control. For manual control, we subscribe to the topics from the micro-ROS node over the network. For each of these control modes, there is a unique starting position. For the joint control, we want to set the joints to match the initial potentiometer position, while for joystick control we want to set the joints to a safe position specific to the joystick operation. After setting these segments, we update the position directly with filtered input values. 

For potentiometer control, we implement a first-order filter on the potentiometer rotation values. This creates a smoothed update sequence, so we are able to directly write the filtered potentiometer positions to the HEBIs after the startup splines are complete.

For joystick control, we have constant velocity control in the direction of the joystick. As the velocity is sufficiently slow, we are able to numerically integrate, adding a set distance at every time step of the runtime. The nominal velocity control adds 1 mm per time step, giving a velocity of 10cm/s. By maxing out the control range in either direction, the velocity can increase up to a maximum velocity of 20 cm/s in either direction. As this motion is fairly slow, the robot is able to move smoothly while controlled. To implement the actual control, we have a saved desired joystick position variable which updates with the velocity command. At each time step, the ikin of this desired position is computed, and the resultant joint states are sent to the motor controller.

Now, we must switch between the two manual control modes based on the state of the select toggle switch on the controller box. At the beginning of the update loop for each mode, there is a check for a mode mismatch. If the software operation mode does not match the controller toggle switch, the mode is switched by executing the initial spline case for the selected mode.

## Autonomous Control
We also had 2 separate modes of autonomous control. For these modes, we subscribe to the detector topics to find the position of contours and their matches. Both of these methods start with the same spline, sending the robot off of the board to make sure the entire board can be seen by the camera. 

First, we consider autonomous removal from the board. For this, we only care about the subscriber giving the list of pieces currently on the board. We send the robot to the safe position at the start, then check the board. If there are any pieces on the board, it picks one and updates the list of splines to take it off the board. The removal motion goes to a point directly above the dropoff location, then moves straight down to drop the piece. This ensures the robot does not move into the board while moving to the dropoff location.
 
Autonomous insertion has a similar movement path to the autonomous removal, but instead of 3D splines in xyz cartesian space, we must compute splines in 5D xyz tilt pan space, as we are controlling the final pan angle of the tip to ensure the piece has the right orientation when being dropped in. 

For overlapping chunks, we have a list of positions which do not match holes on the board. If there are no more pieces which match holes on the board, we go to this list and move the first centroid to one of 4 safe positions. After moving this centroid, it rechecks the board and continues. This allows for overlapping pieces to be moved then matched to hole shape to place onto the board.

Combining manual and autonomous motion, we have a flag in the code which determines the mode and any transitions. Currently, the flag is set such that the robot completes one of the two autonomous motions then switches to user input, but the modes are built with modularity in mind, so these transitions can be easily switched.

The complete diagram of the software is shown below:
![image](https://user-images.githubusercontent.com/67039263/229221027-a9ded61a-69f4-4661-b06a-d841c87515ce.png)

[Go back](https://github.com/pramohan/me134)
