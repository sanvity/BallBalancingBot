from picamera2 import Picamera2
import cv2
import numpy as np

# HSV range for orange TT ball (tune if needed)
lower_orange = np.array([4, 140, 150])
upper_orange = np.array([25, 255, 255])

# Initialize PiCamera2
picam2 = Picamera2()
picam2.configure(picam2.create_preview_configuration(main={"size": (640, 480)}))
picam2.start()

# Mouse callback to print HSV value under cursor
def show_hsv_on_click(event, x, y, flags, param):
    if event == cv2.EVENT_MOUSEMOVE:
        hsv_img = param
        if hsv_img is not None and y < hsv_img.shape[0] and x < hsv_img.shape[1]:
            hsv_pixel = hsv_img[y, x]
            print(f"HSV at ({x}, {y}): {hsv_pixel}")

# Windows for display
cv2.namedWindow("Ball Tracking")
cv2.namedWindow("HSV")
cv2.setMouseCallback("HSV", show_hsv_on_click, None)

while True:
    # Capture and convert to BGR (PiCamera2 gives RGB)
    frame = picam2.capture_array()
    frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
    frame = cv2.resize(frame, (500, 500))

    # Blur and convert to HSV
    blurred = cv2.GaussianBlur(frame, (5, 5), 0)
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

    # Update HSV image for mouse callback
    cv2.setMouseCallback("HSV", show_hsv_on_click, hsv)

    # Create mask for orange
    mask = cv2.inRange(hsv, lower_orange, upper_orange)
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    # Find contours in the mask
    contours, _ = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if contours:
        # Find largest contour and minimum enclosing circle
        c = max(contours, key=cv2.contourArea)
        ((x, y), radius) = cv2.minEnclosingCircle(c)
        area = cv2.contourArea(c)

        if radius > 10 and area > 100:
            # Draw circle and coordinates
            cv2.circle(frame, (int(x), int(y)), int(radius), (0, 255, 0), 2)
            cv2.circle(frame, (int(x), int(y)), 5, (0, 0, 255), -1)
            cv2.putText(frame, f"X: {int(x)}, Y: {int(y)}", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 2)

    # Show outputs
    cv2.imshow("Ball Tracking", frame)
    cv2.imshow("Mask", mask)
    cv2.imshow("HSV", hsv)

    # Exit on 'q'
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Cleanup
cv2.destroyAllWindows()
picam2.stop()
