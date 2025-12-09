import cv2
import numpy as np

def list_available_cameras(max_devices=5):
    available = [0]
##    for i in range(max_devices):
##        cap = cv2.VideoCapture(i)
##        if cap.read()[0]:
##            available.append(i)
##        cap.release()
    return available

# List available webcams
print("Scanning for available webcams...")
cameras = list_available_cameras()

if not cameras:
    print("No webcams found.")
    exit()

print("Available webcams:")
for idx in cameras:
    print(f"{idx}: Camera {idx}")

# Ask user to choose one
cam_index = int(input("Select camera index: "))
if cam_index not in cameras:
    print("Invalid camera index.")
    exit()

# Open selected webcam
cap = cv2.VideoCapture(cam_index)
if not cap.isOpened():
    print("Error: Could not open webcam.")
    exit()

# Define HSV color range
sensitivity  = 15
lower_color = np.array([0,0,232])
upper_color = np.array([179,108,255])

while True:
    ret, frame = cap.read()
    if not ret:
        print("Error reading frame")
        break

    blurred = cv2.GaussianBlur(frame, (11, 11), 0)
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, lower_color, upper_color)
    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if contours:
        c = max(contours, key=cv2.contourArea)
        ((x, y), radius) = cv2.minEnclosingCircle(c)
        if radius > 5:
            cv2.circle(frame, (int(x), int(y)), int(radius), (0, 255, 0), 2)
            cv2.putText(frame, f"Ball: ({int(x)}, {int(y)})", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

    output = cv2.bitwise_and(frame, frame, mask=mask)
    cv2.imshow("Ping Pong Ball Tracking", frame)
    cv2.imshow("Ping Pong Ball Mask", output)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
