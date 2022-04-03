# **Kalman-Copter**: a helicopter control system using simple Arduino parts
Based on a modern helicopter control system, the **kalman-copter** simplifies it to the bare necessities, utilizing an "altitude sensor" and "yoke" in the form of an ultrasonic sensor and joystick.

<p align="center">
  <img src="./media/enclosed_system_front.png">
</p>

<div align="center">Figure 1. Assembled Kalman-Copter</div><br>

## **Program Specifications**
The objective of the project was to use our knowledge of microcontrollers (specfically the Arduino UNO) and create a system that mimics a real-world aerospace application using some sensors, actuators, and filters.

The Kalman-Copter utilizes 2 sensors as inputs and a custom kalman filter to drive the 3 outputs. A complete part list can be found further in the README.

## **Kalman-Copter In Action**

<div align = "center">Figure 2. Kalman-Copter reacts to the user input in real time. The pitch and roll angles are the red and green lines (respectively) and the altitude is measured with the blue line. Each input has a separate kalman filter to reduce noise across the system. </div> <br>