
https://github.com/yousefh112/Color-Classifier/assets/95450251/d1d9d921-cd32-4401-b554-0de96c9c48ef

# Color Classifier 🤖

Welcome to the Color Classifier project repository! This project, powered by ROS (Robot Operating System) and OpenCV, is designed to detect and classify colors, triggering corresponding actions by a robotic arm.

## Project Overview

The Color Classifier project consists of two main components:

1. **Color Classifier Node**: This node analyzes images to detect colors and publishes the results to the `/color` topic. If the detected color is green, it sends a signal of 1; otherwise, it sends 0.

2. **Robot Arm Operation Node**: Subscribing to the `/color` topic, this node triggers the robot arm to perform a pick operation when it receives a signal of 1 (indicating the presence of green).

## Installation

To get started with the Color Classifier project, follow these steps:

1. Clone this repository to your local machine:

   ```bash
   git clone https://github.com/yousefh112/Color-Classifier-Using-ROS.git
   ```

2. Navigate to the project directory:

   ```bash
   cd Color-Classifier-Using-ROS
   ```

3. Ensure you have ROS and OpenCV installed on your system.

4. Build the ROS package:

   ```bash
   catkin_make
   ```

## Usage

1. Launch the Color Classifier nodes:

   ```bash
   roslaunch color_classifier color_classifier.launch
   ```

2. Watch the robot arm spring into action as it picks objects of the specified color!

## Video Demonstration

Check out our exciting video demonstration to see the Color Classifier project in action:



Uploading Screencast from 24-04-24 02:29:24.mp4…




## Contributors

- Yousef Hesham ([@yousefh112](https://github.com/yousefh112))
- Abdelrahman Helal ([@abdelrahmanhelal](https://github.com/Helal20002018))
