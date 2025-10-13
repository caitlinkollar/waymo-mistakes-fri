import cv2
import mediapipe as mp
import numpy as np
import math
import time

class HandGestureRecognizer:
    # Initial Assumptions: 2 FPS Output, Outputting String of Labelled Landmarks, 

    def __init__(self):
        # Initialize MediaPipe Hands
        self.mp_hands = mp.solutions.hands
        self.hands = self.mp_hands.Hands(
            static_image_mode=False,
            max_num_hands=2,
            min_detection_confidence=0.5,
            min_tracking_confidence=0.5
        )
        self.mp_drawing = mp.solutions.drawing_utils
        self.mp_drawing_styles = mp.solutions.drawing_styles
        
        # MediaPipe Hands landmark indices
        # For reference: https://developers.google.com/mediapipe/solutions/vision/hand_landmarker#hand_landmark_model
        self.WRIST = 0
        self.THUMB_TIP = 4
        self.INDEX_MCP = 5  # Index finger metacarpophalangeal joint
        self.INDEX_TIP = 8
        self.MIDDLE_MCP = 9  # Middle finger metacarpophalangeal joint
        self.MIDDLE_PIP = 10  # Middle finger proximal interphalangeal joint
        self.MIDDLE_TIP = 12
        self.RING_TIP = 16
        self.PINKY_TIP = 20
        
        # These will be the base of the palm for angle calculations
        self.PALM_BASE = 0  # Wrist

    def process_frame(self, frame):
        # Convert BGR to RGB
        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        
        # Process the frame to detect hands
        results = self.hands.process(frame_rgb)
        
        # Initialize data for the frame
        frame_data = {
            "is_hand_open": False,
            "middle_finger_angle": None
        }
        
        # If hands are detected
        if results.multi_hand_landmarks:
            for hand_landmarks in results.multi_hand_landmarks:
                # Draw hand landmarks on the frame
                self.mp_drawing.draw_landmarks(
                    frame,
                    hand_landmarks,
                    self.mp_hands.HAND_CONNECTIONS,
                    self.mp_drawing_styles.get_default_hand_landmarks_style(),
                    self.mp_drawing_styles.get_default_hand_connections_style()
                )
                
                # Determine if the hand is open
                is_open = self._is_hand_open(hand_landmarks)
                
                # Calculate middle finger angle
                middle_finger_angle = self._calculate_middle_finger_angle(hand_landmarks)
                
                # Update frame data
                frame_data["is_hand_open"] = is_open
                frame_data["middle_finger_angle"] = middle_finger_angle
                
                # Add text annotations to the frame
                height, width, _ = frame.shape
                cv2.putText(frame, f"Hand Open: {is_open}", (10, 30), 
                            cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                cv2.putText(frame, f"Middle Finger Angle: {middle_finger_angle:.2f} degrees", 
                            (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                
                # Draw vector showing direction of middle finger
                if middle_finger_angle is not None:
                    self._draw_middle_finger_vector(frame, hand_landmarks)
        
        return frame, frame_data

def main():
    # Initialize the hand gesture recognizer
    recognizer = HandGestureRecognizer()
    
    # Try different camera indices and backends
    camera_index = 0
    backends_to_try = [
        cv2.CAP_V4L2,
        cv2.CAP_ANY,
        cv2.CAP_GSTREAMER,
    ]
    
    cap = None
    for backend in backends_to_try:
        print(f"Trying camera {camera_index} with backend {backend}...")
        cap = cv2.VideoCapture(camera_index, backend)
        
        # Set camera properties for better compatibility
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        cap.set(cv2.CAP_PROP_FPS, 30)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        
        # Give camera time to initialize
        time.sleep(2)
        
        # Try to read a frame
        success, frame = cap.read()
        if success and frame is not None:
            print(f"Successfully opened camera with backend {backend}")
            break
        else:
            cap.release()
            cap = None
    
    # If still no success, try other camera indices
    if cap is None or not cap.isOpened():
        print("Trying alternate camera indices...")
        for idx in range(1, 5):
            cap = cv2.VideoCapture(idx)
            cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            time.sleep(1)
            success, frame = cap.read()
            if success and frame is not None:
                print(f"Successfully opened camera at index {idx}")
                camera_index = idx
                break
            cap.release()
    
    if cap is None or not cap.isOpened():
        print("ERROR: Could not open any camera. Please check:")
        print("1. Camera is connected and not in use by another application")
        print("2. You have permission to access the camera")
        print("3. Run: ls -l /dev/video* to see available cameras")
        return
    
    print(f"Camera opened successfully. Press 'q' to quit.")
    
    frame_count = 0
    while cap.isOpened():
        success, frame = cap.read()
        if not success or frame is None:
            print(f"Warning: Failed to read frame {frame_count}")
            # Try to re-grab
            cap.grab()
            continue
        
        frame_count += 1
        
        # Process the frame
        processed_frame, frame_data = recognizer.process_frame(frame)
        
        # Print data every 30 frames (roughly once per second at 30fps)
        if frame_count % 30 == 0:
            print(f"Hand open: {frame_data['is_hand_open']}, "
                  f"Middle finger angle: {frame_data['middle_finger_angle']}")
        
        # Display the frame
        cv2.imshow('MediaPipe Hand Gesture Recognition', processed_frame)
        
        # Exit on 'q' press
        if cv2.waitKey(5) & 0xFF == ord('q'):
            break
    
    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
