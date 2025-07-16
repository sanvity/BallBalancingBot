from picamera2 import Picamera2
import cv2
import numpy as np
import math
import RPi.GPIO as GPIO
import time

# --- GPIO Setup ---
GPIO.setmode(GPIO.BCM)
servo_pins = [18, 23, 24]  # arm1, arm2, arm3
equilibrium_angles = [90, 90, 90] #need to check
def angle_to_duty_cycle(angle):
    
    return 2.5 + (angle / 180.0) * 10.0

# === Initialize servos only once ===
servos = []
for pin in servo_pins:
    GPIO.setup(pin, GPIO.OUT)
    pwm = GPIO.PWM(pin, 50)  # 50Hz for standard servos
    pwm.start(angle_to_duty_cycle(equilibrium_angles[servo_pins.index(pin)]))
    servos.append(pwm)



# ----- PID CONSTANTS (GLOBAL) -----
kp = 1.0     # Proportional gain
ki = 0     # Integral gain
kd = 0    # Derivative gain
k = 0.004      # Output magnitude gain
alpha = 0.8  # Low-pass filter coefficient

# ----- PID STATE INITIALIZATION -----
pid_state = {
    'last_output_x': 0.0,
    'last_output_y': 0.0,
    'last_error_x': 0.0,
    'integral_x': 0.0,
    'last_error_y': 0.0,
    'integral_y': 0.0,
    'last_time': None
}
#Goal = (0,0)
# ----- PID COMPUTE FUNCTION -----
def compute_pid(Goal, Current_value, pid_state):
    global kp, ki, kd, k, alpha
    if Current_value==Goal:
        return 0,0

    current_time = time.perf_counter()

    if pid_state['last_time'] is None:
        pid_state['last_time'] = current_time
        return 0, 0

    dt = current_time - pid_state['last_time']
    if dt == 0:
        dt = 1e-6  # Avoid division by zero

    # Calculate error
    error_x = Current_value[0] - Goal[0]
    error_y = Goal[1] - Current_value[1]

    # Update integral
    pid_state['integral_x'] += error_x * dt
    pid_state['integral_y'] += error_y * dt

    # Calculate derivative
    derivative_x = (error_x - pid_state['last_error_x']) / dt
    derivative_y = (error_y - pid_state['last_error_y']) / dt

    # Compute PID output
    output_x = kp * error_x + ki * pid_state['integral_x'] + kd * derivative_x
    output_y = kp * error_y + ki * pid_state['integral_y'] + kd * derivative_y

    # Apply low-pass filter
    output_x = alpha * output_x + (1 - alpha) * pid_state['last_output_x']
    output_y = alpha * output_y + (1 - alpha) * pid_state['last_output_y']

    # Calculate angle (theta) and magnitude (phi)
    theta = math.degrees(math.atan2(output_y, output_x))
    theta+=45 # you can change the value according to the value got in camera_axis code
    if theta < 0:
        theta += 360

    phi = k * math.sqrt(output_x**2 + output_y**2)

    # Update PID state
    pid_state['last_error_x'] = error_x
    pid_state['last_error_y'] = error_y
    pid_state['last_output_x'] = output_x
    pid_state['last_output_y'] = output_y
    pid_state['last_time'] = current_time

    return theta, phi


# --------- CONSTANTS ---------
r = 16.8     # Radius or arm reach check
L1 = 10    # Link 1 length check
L2 = 12.4  # Link 2 length check
X = 11.3   # Horizontal offset of the arm and edge of the total circumference

# --------- FUNCTION: Compute leg heights (ah1, ah2, ah3) ---------
def calculate_heights(theta_deg, phi_deg, ch):
    theta_rad = math.radians(theta_deg)
    sin_phi = math.sin(math.radians(phi_deg * 5))  # Change max tilt *******

    if 90 < theta_deg < 270:
        ah1 = ch - abs(r * math.sin(theta_rad) * sin_phi)
    else:
        ah1 = ch + abs(r * math.sin(theta_rad) * sin_phi)

    if 30 < theta_deg < 210:
        ah2 = ch + abs(r * math.cos(theta_rad + math.radians(30)) * sin_phi)
    else:
        ah2 = ch - abs(r * math.cos(theta_rad + math.radians(30)) * sin_phi)

    if 150 < theta_deg < 330:
        ah3 = ch + abs(r * math.cos(theta_rad - math.radians(30)) * sin_phi)
    else:
        ah3 = ch - abs(r * math.cos(theta_rad - math.radians(30)) * sin_phi)

    return ah1, ah2, ah3

# --------- FUNCTION: Compute single alpha angle ---------
def calculate_alpha(y):
    try:
        part1 = math.atan2(y, X)
        d = math.sqrt(X**2 + y**2)
        part2 = math.acos((X**2 + y**2 + L1**2 - L2**2) / (2 * L1 * d))
        alpha_rad = part1 - part2
        return math.degrees(alpha_rad)
    except ValueError:
        return float('nan')  # Handle domain errors gracefully

# --------- FUNCTION: Compute all output angles A1, A2, A3 ---------
def compute_output_angles(theta_deg, phi_deg, ch):
    ah1, ah2, ah3 = calculate_heights(theta_deg, phi_deg, ch)
    A1 = calculate_alpha(ah1)
    A2 = calculate_alpha(ah2)
    A3 = calculate_alpha(ah3)
    return A1, A2, A3


# === HSV range for orange TT ball ===
lower_orange = np.array([4, 140, 150])
upper_orange = np.array([25, 255, 255])

# === Initialize PiCamera2 with full resolution === # i have commented high resolution tio check latency
picam2 = Picamera2()
print("Max resolution:", picam2.sensor_resolution)
#picam2.configure(picam2.create_preview_configuration(main={"size": picam2.sensor_resolution}))
#config = picam2.create_preview_configuration(main={"size": (320, 240)}, buffer_count=2)
#picam2.configure(config)

# Get the full sensor resolution
full_res = picam2.sensor_resolution

# Set the crop to cover the whole sensor
picam2.set_controls({"ScalerCrop": (0, 0, full_res[0], full_res[1])})

# Now configure with your low-res output
config = picam2.create_preview_configuration(main={"size": (320, 240)}, buffer_count=2)
picam2.configure(config)

picam2.start()

# === Mouse callback to inspect HSV ===
def show_hsv_on_click(event, x, y, flags, param):
    if event == cv2.EVENT_MOUSEMOVE:
        hsv_img = param
        if hsv_img is not None and y < hsv_img.shape[0] and x < hsv_img.shape[1]:
            hsv_pixel = hsv_img[y, x]
            print(f"HSV at ({x}, {y}): {hsv_pixel}")

cv2.namedWindow("Ball Tracking")
cv2.namedWindow("HSV")
cv2.setMouseCallback("HSV", show_hsv_on_click, None)

# === Rotation angle in degrees ===
angle = 0  # offset check to align the arm to x-axis

while True:
    # === Capture frame and convert RGB to BGR ===
    frame = picam2.capture_array()
    frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

    # === Rotate frame by custom angle with canvas expansion ===
    '''(h, w) = frame.shape[:2]
    center = (w // 2, h // 2)
    M = cv2.getRotationMatrix2D(center, angle, 1.0)'''

    # Calculate new bounding dimensions to avoid cropping
    '''cos = np.abs(M[0, 0])
    sin = np.abs(M[0, 1])
    new_w = int((h * sin) + (w * cos))
    new_h = int((h * cos) + (w * sin))'''

    # Adjust transformation matrix to center the image in new canvas
    '''M[0, 2] += (new_w / 2) - center[0]
    M[1, 2] += (new_h / 2) - center[1]'''

    # Apply affine transformation (rotation)
    #frame = cv2.warpAffine(frame, M, (new_w, new_h))

    # === Resize for display only ===
    display_frame = cv2.resize(frame, (800, 600))

    # === Convert to HSV ===
    blurred = cv2.GaussianBlur(display_frame, (5, 5), 0)
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

    cv2.setMouseCallback("HSV", show_hsv_on_click, hsv)

    # === Create HSV mask ===
    mask = cv2.inRange(hsv, lower_orange, upper_orange)
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    # === Draw Cartesian axes centered at the middle ===
    h_disp, w_disp = display_frame.shape[:2]
    origin_x = w_disp // 2-31 # change to set centre of the plate
    origin_y = h_disp // 2-31
    Goal=(origin_x,origin_y)
    cv2.line(display_frame, (origin_x, 0), (origin_x, h_disp), (200, 200, 200), 1)  # Y-axis
    cv2.line(display_frame, (0, origin_y), (w_disp, origin_y), (200, 200, 200), 1)  # X-axis
    cv2.circle(display_frame, (origin_x, origin_y), 5, (0, 255, 255), -1)
    cv2.putText(display_frame, "(0,0)", (origin_x + 10, origin_y - 10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)

    # === Detect and annotate ball ===
    contours, _ = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    x_coord=Goal[0]
    y_coord=Goal[1]
    
    if contours:
        c = max(contours, key=cv2.contourArea)
        ((x, y), radius) = cv2.minEnclosingCircle(c)
        area = cv2.contourArea(c)

        if radius > 10 and area > 100:
            cv2.circle(display_frame, (int(x), int(y)), int(radius), (0, 255, 0), 2)
            cv2.circle(display_frame, (int(x), int(y)), 5, (0, 0, 255), -1)

            # Convert to origin-centered coordinates
            
            '''x_coord = int(x - origin_x)
            y_coord = int(origin_y - y)  # Flip Y so up is positive'''
            x_coord=int(x)
            y_coord=int(y)

            cv2.putText(display_frame, f"X: {x_coord}, Y: {y_coord}", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 2)
    Current_value = (x_coord,y_coord)
    theta ,phi=compute_pid(Goal, Current_value , pid_state)
    ch=13.5 # center height from arm horizontal level
    A1, A2, A3 = compute_output_angles(theta, phi, ch)
    print(f"A1: {A1:.2f}, A2: {A2:.2f}, A3: {A3:.2f}")

    
    output_angles = [A1+90, A2+90, A3+90]
    for i,pwm in enumerate(servos):
        if not math.isnan(output_angles[i]):
            angle_df = max(80, min(150, output_angles[i]))
            duty = angle_to_duty_cycle(angle_df)
            pwm.ChangeDutyCycle(duty)
    time.sleep(0.8)
        

    # === Show all frames ===
    cv2.imshow("Ball Tracking", display_frame)
    cv2.imshow("Mask", mask)
    cv2.imshow("HSV", hsv)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# === Cleanup ===
cv2.destroyAllWindows()
for pwm in servos:
    pwm.stop()
GPIO.cleanup()
picam2.stop()
