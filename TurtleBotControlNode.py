import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from math import radians

class TurtleBotControlNode:
    def __init__(self):
        rospy.init_node('turtlebot_control_node', anonymous=True)
        self.command_sub = rospy.Subscriber('/gesture_command', String, self.command_callback)
        self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.last_command = None
        self.command_time = rospy.Time.now()

    def move_forward(self, linear_velocity, duration):
        twist = Twist()
        twist.linear.x = linear_velocity
        self.vel_pub.publish(twist)
        rospy.sleep(duration)

    def move_backward(self, linear_velocity, duration):
        twist = Twist()
        twist.linear.x = -linear_velocity
        self.vel_pub.publish(twist)
        rospy.sleep(duration)

    def turn_right(self, radian_angle, duration):
        twist = Twist()
        twist.angular.z = -radians(radian_angle)
        self.vel_pub.publish(twist)
        rospy.sleep(duration)

    def turn_left(self, radian_angle, duration):
        twist = Twist()
        twist.angular.z = radians(radian_angle)
        self.vel_pub.publish(twist)
        rospy.sleep(duration)

    def stop(self):
        twist = Twist()
        self.vel_pub.publish(twist)

    def command_callback(self, data):
        command = data.data
        current_time = rospy.Time.now()

        # Check if the command is the same as the last and if enough time has passed
        if (current_time - self.command_time).to_sec() > 5:
            self.last_command = command
            self.command_time = current_time

            if command == 'Move Forward':
                print("Moving forward")
                self.move_forward(0.5, 5)
            elif command == 'Move Backward':
                print("Moving backward")
                self.move_backward(0.5, 5)
            elif command == 'Turn Right':
                print("Turning Right")
                self.turn_right(45, 5)
            elif command == 'Turn Left':
                print("Turning Left")
                self.turn_left(45, 5)
            elif command == 'Stop':
                print("Stopping")
                self.stop()
            elif command == 'no_gesture':
                pass
            else:
                rospy.logwarn(f"Unknown command: {command}")

if __name__ == '__main__':
    turtlebot_control_node = TurtleBotControlNode()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down TurtleBot Control Node")
