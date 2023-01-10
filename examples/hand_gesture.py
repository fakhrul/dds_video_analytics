#Required things
#1.Mediapipe
#2.OpenCV
import cv2
import mediapipe as mp
mp_drawing = mp.solutions.drawing_utils
mp_drawing_styles = mp.solutions.drawing_styles
mp_hands = mp.solutions.hands

# reference 
# https://google.github.io/mediapipe/


cap = cv2.VideoCapture(0)

width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
fps = cap.get(cv2.CAP_PROP_FPS)
vwriter = cv2.VideoWriter('hand-out.avi', cv2.VideoWriter_fourcc(*'MJPG'),fps, (width, height))

hands=mp_hands.Hands()

while True:
    success, image = cap.read()
    # Flip the image horizontally for a later selfie-view display, and convert
    # the BGR image to RGB.
    image = cv2.cvtColor(cv2.flip(image, 1), cv2.COLOR_BGR2RGB)
    # To improve performance, optionally mark the image as not writeable to
    # pass by reference.
    results = hands.process(image)
    image = cv2.cvtColor(image,cv2.COLOR_RGB2BGR)
    if results.multi_hand_landmarks:
      for hand_landmarks in results.multi_hand_landmarks:                                                                         
        mp_drawing.draw_landmarks(                                                 
            image,
            hand_landmarks,mp_hands.HAND_CONNECTIONS)
    cv2.imshow('MediaPipe Hands', image)
    vwriter.write(image)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break     

cap.release()
vwriter.release()
cv2.destroyAllWindows()