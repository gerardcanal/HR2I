HR2I (HUMAN ROBOT ROBOT INTERACTION)
====
Repository for the code of the Master Thesis called "Human Multi-Robot Interaction based on Gesture Recognition".
The author of the thesis is Gerard Canal, and the supervisors are Cecilio Angulo and Sergio Escalera.
 
===

A brief description of the project is:

Interaction with a robot in a gesture-based manner. It involves gesture detection and recognition with a depth camera posed on a mobile platform. A NAO robot may be used to perform object manipulation. Example case: a person pointing an object which the robot has to find and maybe grasp.

===

#### Repository organization
The repository has two branches:
 - ros: With the ROS code and packages, to be execuged in a Linux (Ubuntu) operating system
 - Kinect: Contains the MS Visual Studio 2013 package with the code for the Kinect v2 sensor, which runs on a Windows 8.1 machine. The main project, HR2I\_Kinect2 contains also the interface with ROS via the rosserial\_windows package
