import cv2
import mediapipe as mp
mp_drawing = mp.solutions.drawing_utils
mp_hands = mp.solutions.hands


def is_finger_up(tip_y, other_fingers_y):
  return all(tip_y < y for y in other_fingers_y)

#Get landmark position for fingertips and base joints
def get_finger_positions(hand_landmarks, mp_hands):
    thumb_tip_y = hand_landmarks.landmark[mp_hands.HandLandmark.THUMB_TIP].y
    index_finger_tip_y = hand_landmarks.landmark[mp_hands.HandLandmark.INDEX_FINGER_TIP].y
    middle_finger_tip_y = hand_landmarks.landmark[mp_hands.HandLandmark.MIDDLE_FINGER_TIP].y
    ring_finger_tip_y = hand_landmarks.landmark[mp_hands.HandLandmark.RING_FINGER_TIP].y
    little_finger_tip_y = hand_landmarks.landmark[mp_hands.HandLandmark.PINKY_TIP].y

    return thumb_tip_y, index_finger_tip_y, middle_finger_tip_y, ring_finger_tip_y, little_finger_tip_y


# Function to detect 'Thumbs Up'
def detect_gesture(hand_landmarks, mp_hands):
  thumb_tip_y, index_finger_tip_y, middle_finger_tip_y, ring_finger_tip_y, little_finger_tip_y = get_finger_positions(hand_landmarks, mp_hands)

  thumbs_up = is_finger_up (thumb_tip_y, [index_finger_tip_y, middle_finger_tip_y, ring_finger_tip_y, little_finger_tip_y])
  index_up = is_finger_up (index_finger_tip_y, [thumb_tip_y, middle_finger_tip_y, ring_finger_tip_y, little_finger_tip_y])
  middle_up = is_finger_up (middle_finger_tip_y, [thumb_tip_y, index_finger_tip_y, ring_finger_tip_y, little_finger_tip_y])
  ring_up = is_finger_up (ring_finger_tip_y, [thumb_tip_y, index_finger_tip_y, middle_finger_tip_y, little_finger_tip_y])
  little_up = is_finger_up (little_finger_tip_y, [thumb_tip_y, index_finger_tip_y, middle_finger_tip_y, ring_finger_tip_y])


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
  return None

cap = cv2.VideoCapture(0)
with mp_hands.Hands(
    min_detection_confidence=0.5,
    min_tracking_confidence=0.5) as hands:
  while cap.isOpened():
    success, image = cap.read()

    if not success:
      print("Ignoring empty camera frame.")
      # If loading a video, use 'break' instead of 'continue'.
      continue
    image = cv2.cvtColor(cv2.flip(image, 1), cv2.COLOR_BGR2RGB)

    image.flags.writeable = False
    results = hands.process(image)

    # Draw the hand annotations on the image.
    image.flags.writeable = True
    image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
    if results.multi_hand_landmarks:
      for hand_landmarks in results.multi_hand_landmarks:
        mp_drawing.draw_landmarks(
            image, hand_landmarks, mp_hands.HAND_CONNECTIONS)

        gesture = detect_gesture(hand_landmarks, mp_hands)
        if gesture:
          print(gesture)
          cv2.putText(image, gesture, (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2, cv2.LINE_AA)

    cv2.imshow('MediaPipe Hands', image)
    if cv2.waitKey(5) & 0xFF == 27:
      break

cap.release()
cv2.destoryAllWindows()

