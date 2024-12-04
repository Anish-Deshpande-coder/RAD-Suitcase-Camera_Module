import cv2
import numpy as np
import pigpio
import time
import serial

# Servo motor GPIO pin
servo_pin = 18

# Initialize pigpio
pi = pigpio.pi()

# Initialize the USB camera
camera_index = 0
camera = cv2.VideoCapture(camera_index)

# Check if the camera is opened successfully
if not camera.isOpened():
    print("Error: Could not open camera")
    exit()

print("Camera started")

# Load the predefined ArUco dictionary and detector parameters
dictionary = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_250)
parameters = cv2.aruco.DetectorParameters_create()
print("Loaded ArUco dictionary and parameters")

# Set initial position and range of the servo motor
initial_pan_angle = 90  # Initial angle
min_pan_angle = 0  # Minimum angle (adjust as needed)
max_pan_angle = 180  # Maximum angle (adjust as needed)
pan_angle = initial_pan_angle


# Map angle to pulsewidth function
def angle_to_pulsewidth(angle):
    return 500 + (angle * 2000 / 180)  # Adjust as needed for your servo


# Set initial position of the servo motor
pi.set_servo_pulsewidth(servo_pin, angle_to_pulsewidth(pan_angle))
time.sleep(1)  # Wait for the servo to move

# Proportional control gain
kp = 0.01  # Adjust as needed

# Maximum change in angle per iteration
max_angle_change = 5  # Adjust as needed

# Motor serial communication setup
ser = serial.Serial('/dev/ttyUSB0', 9600)

try:
    while True:
        # Capture frame from the camera
        ret, frame = camera.read()
        if not ret:
            print("Error: Failed to capture frame")
            break

        # Convert the frame to grayscale for ArUco marker detection
        gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Detect ArUco markers in the grayscale image
        corners, ids, rejected = cv2.aruco.detectMarkers(gray_frame, dictionary, parameters=parameters)

        if ids is not None and len(ids) > 0:
            # Calculate the centroid of the first detected marker
            marker_id = ids[0]
            marker_index = np.where(ids == marker_id)[0][0]
            centroid = np.mean(corners[marker_index][0], axis=0).astype(int)

            # Calculate error as the difference between centroid position and center of the frame,
            # but invert the sign to make the camera move in the correct direction
            error = (frame.shape[1] / 2) - centroid[0]

            # Limit the maximum change in angle per iteration
            angle_change = kp * error
            angle_change = max(min(angle_change, max_angle_change), -max_angle_change)

            # Update pan angle
            pan_angle += int(angle_change)

            # Limit the pan angle within the defined range
            pan_angle = max(min(pan_angle, max_pan_angle), min_pan_angle)

            # Move the servo motor to track the marker
            pi.set_servo_pulsewidth(servo_pin, angle_to_pulsewidth(pan_angle))
           
            # Send data to Arduino to control motors
            if error > 10:
                ser.write(b'r')  # Move right
            elif error < -10:
                ser.write(b'l')  # Move left
            else:
                ser.write(b'f')  # Move forward

        else:
            # ArUco marker not detected, stop motors
            ser.write(b's')  # Stop

        # Display the processed image
        cv2.imshow("ArUco Marker Tracking", frame)

        # Break the loop if 'q' key is pressed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            print("Breaking loop: 'q' key pressed")
            break

finally:
    # Cleanup
    pi.stop()  # Release pigpio resources
    camera.release()  # Release the camera
    cv2.destroyAllWindows()  # Close all OpenCV windows
    ser.close()  # Close serial connection
    print("Cleaned up and exited")