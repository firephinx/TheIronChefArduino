# TheIronChefArduino

## Setup Instructions:
1. Follow instructions here to install rosserial onto the Arduino: http://wiki.ros.org/rosserial_arduino/Tutorials
2. Use the Arduino IDE to upload the arduino code in TheIronChef.ino to the Arduino Mega.

## Starting ROS Serial
1. Start a new roscore on the NUC. 
2. Type `startrosserial` in any terminal window.

## ROS Serial Options
1. To turn on the electromagnet, type `magneton` in any terminal.
2. To turn off the electromagnet, type `magnetoff` in any terminal.
3. To home the robot, type `home` in any terminal.
4. To stop everything, type `stop` in any terminal.
5. To start again after stopping everything, type `start` in any terminal.
6. To reset the robot, type `reset` in any terminal.
7. To keep the robot at its current location, type `stay` in any terminal. 