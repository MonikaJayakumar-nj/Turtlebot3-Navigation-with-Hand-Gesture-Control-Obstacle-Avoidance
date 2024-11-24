import rospy
import cv2
import numpy as np
import mediapipe as mp
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError

mp_drawing = mp.solutions.drawing_utils
mp_hands = mp.solutions.hands


class GestureRecognitionNode:

    def __init__(self):
        rospy.init_node('gesture_recognition_node', anonymous=True)
        #self.image_sub = rospy.Subscriber('/camera/image', Image, self.image_callback)
        self.gesture_pub = rospy.Publisher('/gesture_command', String, queue_size=1)
        self.bridge = CvBridge()
        self.hands = mp_hands.Hands(min_detection_confidence=0.5, min_tracking_confidence=0.5)
        self.last_command_time = rospy.Time.now()
        self.command_interval = rospy.Duration(5)

    '''
    def image_callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, 'bgr8')
            gesture = self.detect_gesture(cv_image)
            # Perform hand gesture recognition using OpenCV contour detection
            # Example: If fingers are detected, publish a message like "move_forward"
            if gesture:
                self.gesture_pub.publish(gesture)
        except CvBridgeError as e:
            print(e)
    '''
    #Function to detect 'ThumbsUp'
    def is_finger_up(self, tip_y, other_fingers_y):
        return all(tip_y < y for y in other_fingers_y)

    # Get landmark position for fingertips and base joints
    def get_finger_positions(self, hand_landmarks):
        thumb_tip_y = hand_landmarks.landmark[mp_hands.HandLandmark.THUMB_TIP].y
        index_finger_tip_y = hand_landmarks.landmark[mp_hands.HandLandmark.INDEX_FINGER_TIP].y
        middle_finger_tip_y = hand_landmarks.landmark[mp_hands.HandLandmark.MIDDLE_FINGER_TIP].y
        ring_finger_tip_y = hand_landmarks.landmark[mp_hands.HandLandmark.RING_FINGER_TIP].y
        little_finger_tip_y = hand_landmarks.landmark[mp_hands.HandLandmark.PINKY_TIP].y

        return thumb_tip_y, index_finger_tip_y, middle_finger_tip_y, ring_finger_tip_y, little_finger_tip_y

    # Function to detect 'Thumbs Up'
    def detect_gesture(self, frame, ):
        if rospy.Time.now() - self.last_command_time < self.command_interval:   # 3 seconds gap
            return None




        image_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        results = self.hands.process(image_rgb)

        if results.multi_hand_landmarks:
            for hand_landmarks in results.multi_hand_landmarks:
                thumb_tip_y, index_finger_tip_y, middle_finger_tip_y, ring_finger_tip_y, little_finger_tip_y = self.get_finger_positions(
                    hand_landmarks)

                thumbs_up = self.is_finger_up(thumb_tip_y,
                                             [index_finger_tip_y, middle_finger_tip_y, ring_finger_tip_y, little_finger_tip_y])
                index_up = self.is_finger_up(index_finger_tip_y,
                                            [thumb_tip_y, middle_finger_tip_y, ring_finger_tip_y, little_finger_tip_y])
                middle_up = self.is_finger_up(middle_finger_tip_y,
                                             [thumb_tip_y, index_finger_tip_y, ring_finger_tip_y, little_finger_tip_y])
                ring_up = self.is_finger_up(ring_finger_tip_y,
                                           [thumb_tip_y, index_finger_tip_y, middle_finger_tip_y, little_finger_tip_y])
                little_up = self.is_finger_up(little_finger_tip_y,
                                             [thumb_tip_y, index_finger_tip_y, middle_finger_tip_y, ring_finger_tip_y])

                # Basic check: Determine which gesture it is
                if thumbs_up and not any([index_up, middle_up, ring_up, little_up]):
                    return "Move Forward"
                elif index_up and not any([thumbs_up, middle_up, ring_up, little_up]):
                    return "Move Backward"
                elif middle_up and not any([thumbs_up, index_up, ring_up, little_up]):
                    return "Turn Right"
                elif ring_up and not any([thumbs_up, middle_up, index_up, little_up]):
                    return "Turn Left"
                elif little_up and not any([thumbs_up, middle_up, ring_up, index_up]):
                    return "Stop"
        return "no_gesture"

if __name__ == '__main__':
    gesture_recognition_node = GestureRecognitionNode()
    cap = cv2.VideoCapture(0)
    try:
        while not rospy.is_shutdown():
            success, image = cap.read()
            if not success:
                print("Ignoring empty camera frame.")
                continue

            # Process the image for gesture recognition
            gesture = gesture_recognition_node.detect_gesture(image)
            if gesture and gesture != "no_gesture":
                print(gesture)
                gesture_recognition_node.gesture_pub.publish(gesture)
                gesture_recognition_node.last_command_time = rospy.Time.now()

            cv2.imshow('Gesture Recognition', image)
            if cv2.waitKey(200) & 0xFF == 27:
                break
    except KeyboardInterrupt:
        print("Shutting down Gesture Recognition Node")
    finally:
        cap.release()
        cv2.destroyAllWindows()
