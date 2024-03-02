import numpy as np
import cv2
import cv2.aruco as aruco

# Create a dictionary of ArUco markers
aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_250)

# Create a video capture object
cap = cv2.VideoCapture(2)

while True:
    # Capture a frame from the video
    ret, frame = cap.read()

    # Convert the frame to grayscale
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Detect ArUco markers in the frame
    corners, ids, _ = aruco.detectMarkers(gray, aruco_dict)

    # If at least one ArUco marker was detected, draw a bounding box around it
    if corners:
        # Draw a bounding box around each ArUco marker
        aruco.drawDetectedMarkers(frame, corners)

        # Print the number of ArUco markers detected
        print("Number of ArUco markers detected: ", len(corners))

        # Print the IDs and locations of the ArUco markers
        for i in range(len(corners)):
            print("ID: ", ids[i], " Location: ", corners[i])

    # Display the resulting frame
    cv2.imshow('Frame', frame)

    # Press 'q' to quit
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Clean up
cap.release()
cv2.destroyAllWindows()