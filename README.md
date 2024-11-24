# Turtlebot3-Navigation-with-Hand-Gesture-Control-Obstacle-Avoidance
This project integrates LIDAR-based obstacle avoidance with hand gesture navigation for the TurtleBot3 using ROS2 Noetic. 
The system processes LIDAR data to detect obstacles and determine safe movement directions. 
Simultaneously, it employs MediaPipe to recognize hand gestures via a PC camera, translating them into navigation commands. 
This dual approach enables the TurtleBot3 to navigate autonomously while responding to user gestures.

# The code for a gesture-controlled TurtleBot application includes:

- HandGestureNode.py (Gesture Recognition): Detects gestures using MediaPipe and publishes commands to a ROS topic.
- TurtleBotControlNode.py (TurtleBot Command Node): Subscribes to the gesture commands and executes appropriate motion commands for the TurtleBot.
- main.py (Hand Gesture Detection): Tests the gesture detection logic in isolation.


![Hand Gestures & corresponding Commands](https://github.com/user-attachments/assets/5d7c8518-206d-4326-92a9-2bbc5115e77f)
