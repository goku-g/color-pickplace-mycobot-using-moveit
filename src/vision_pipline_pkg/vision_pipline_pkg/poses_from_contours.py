from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

import numpy as np
import cv2

class PosesFromContours(Node):
    def __init__(self):
        super().__init__('poses_from_contours_node')
        # Initialize any publishers or parameters here if needed
        self._poses_publisher_ = self.create_publisher(Float64MultiArray, 'detected_object_poses', 10)
        self._timer_poses_pub_ = self.create_timer(0.5, self.publish_detected_poses)  # 2 Hz

    def publish_detected_poses(self):
        # if there is any values in the poses list, publish them
        if len(self.__poses) > 0:
            # publish the whole list
            pass
        
# Global variable to store selected hue
selected_hue_ = 0
selecting_color_ = True
hover_color_ = ()

FRAME_WIDTH = 640
FRAME_HEIGHT = 480

K = np.array([[942.9732466,    0.0,         296.07250616],
     [  0.0,         941.17776222, 247.93965102],
     [  0.0,           0.0,           1.0      ]], dtype=np.float64)

DIST_COEFFS = np.array([-6.20667027e-02,  2.47197572e+00, -1.85292544e-04,  3.19094963e-03,  -2.16298138e+01])

# As all objects lies on the same plane (table), we can use same orientation for all points
# fixed euler angles (in radians)
PLANE_EULER_ANGLES = (0.0, 1.57, -0.85)  # roll, pitch, yaw (euler zyx convention)

def pixel_to_cm(pixel_x, pixel_y, z_cam=0, z_plane=0):
    """
    Convert pixel coordinates to cm using known field of view dimensions.
    
    Args:
        pixel_x, pixel_y: Pixel coordinates
    
    Returns:
        (x_cm, y_cm): Position in cm
    """
    # Input pixel as shape (1,1,2) for undistortPoints
    pts = np.array([[[pixel_x, pixel_y]]], dtype=np.float64)

    # undistortPoints maps to normalized coordinates (x/z, y/z) w.r.t. camera
    # Note: by default it returns coordinates in normalized camera coordinates.
    undist = cv2.undistortPoints(pts, K, DIST_COEFFS, P=None)  # returns (1,1,2)
    xn = float(undist[0, 0, 0])
    yn = float(undist[0, 0, 1])

    # Camera height above plane (cm)
    h_cm = float(z_cam - z_plane)  # positive if camera above plane

    # For the perpendicular camera, direction z component = 1, so:    
    x_cm = -h_cm * xn
    y_cm = -h_cm * yn
    
    # offset and adjust as needed (e.g., to set origin at center of image)
    x_cm = (x_cm + 1.518) * 1.4483
    y_cm = (y_cm - 0.500) * 1.5142
    
    return x_cm, y_cm

def displayFrames(rgb_image):
    cv2.namedWindow('Result Image')
    cv2.imshow('Result Image', rgb_image)
    key_ = cv2.waitKey(1)
    
    # Press esc or 'q' to close the image window
    if key_ & 0xFF == ord('q') or key_ == 27:
        cv2.destroyAllWindows()
        return False
    
    return True

def getColorCuntours(hue, rgb_image):
    hsv = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2HSV)
    
    # Define range of color in HSV 
    diff_lower_hue = cv2.getTrackbarPos('Lower-Hue', 'Trackbars')
    diff_upper_hue = cv2.getTrackbarPos('Upper-Hue', 'Trackbars')

    hue_lower = 0 if (hue - diff_lower_hue < 0) else hue - diff_lower_hue
    hue_upper = hue + diff_upper_hue if (hue + diff_upper_hue < 179) else 179
    
    lower_color = np.array([hue_lower, 50, 20])
    upper_color = np.array([hue_upper, 255, 255])
    
    # print("Selected Hue: ", hue)
    # print("Hue Range: ", hue_lower, " - ", hue_upper)
    # print("Lower HSV: ", lower_color)
    # print("Upper HSV: ", upper_color)
    
    mask = cv2.inRange(hsv, lower_color, upper_color)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    real_contours = []
    contour_centers = []
    
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if 300 < area < 10000:
            real_contours.append(cnt)
            
            M = cv2.moments(cnt)
            if M["m00"] != 0:
                cX = int(M["m10"] / M["m00"])
                cY = int(M["m01"] / M["m00"])
                contour_centers.append((cX, cY))
    
    return real_contours, contour_centers

def selectCallback(event, x, y, flags, frame):
    global selected_hue_, selecting_color_, hover_color_
    
    B = int(frame[y, x][0])
    G = int(frame[y, x][1])
    R = int(frame[y, x][2])
    hover_color_ = (B, G, R)

    if event == cv2.EVENT_LBUTTONDOWN:
        hsv_color = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        selected_hue_ = hsv_color[y, x][0]
        selecting_color_ = False

def trackbarCallback(x):
    pass

def processingLoop(cap):
    
    cv2.namedWindow('Trackbars')
    cv2.createTrackbar('Lower-Hue', 'Trackbars', 2, 179, trackbarCallback)
    cv2.createTrackbar('Upper-Hue', 'Trackbars', 3, 179, trackbarCallback) 
    
    while True:
        ret, color_image = cap.read()
        
        if not ret:
            break
        
        # Color selection phase
        if selecting_color_:
            display_img = color_image.copy()
            cv2.rectangle(display_img, (10, 22), (25, 37), hover_color_, -1)
            cv2.putText(display_img, "Click on desired color", (27, 37), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, hover_color_, 2)
            cv2.imshow('SelectColor', display_img)
            cv2.setMouseCallback('SelectColor', selectCallback, color_image)
            
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
            continue
        else:
            cv2.destroyWindow('SelectColor')
        
        cuntours, centroids = getColorCuntours(selected_hue_, color_image)
        
        object_poses = []
        for cnt, center in zip(cuntours, centroids):
            cv2.drawContours(color_image, [cnt], -1, (0, 255, 0), 2)
            cv2.circle(color_image, center, 3, (255, 255, 255), -1)
            
            z_position = 62  # cm (height of camera from table)
            x_cm, y_cm = pixel_to_cm(center[0], center[1], z_position, 1)
            from_arm_x = -(x_cm + 2.1)  # adjust as per robot arm base offset
            from_arm_y = -(y_cm - 17.4)  # adjust as per robot arm base offset
            
            # matching with robot axes and convert them to standard units (meters and radians)
            object_poses.append((from_arm_y/100, from_arm_x/100, 0.1275, PLANE_EULER_ANGLES[0], PLANE_EULER_ANGLES[1], PLANE_EULER_ANGLES[2]))
            
            cv2.putText(color_image, f"({from_arm_x:.1f} cm, {center[0]}, {from_arm_y:.1f} cm, {center[1]})", 
                        (center[0] + 10, center[1] + 10), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
        
        # Now can publish the object_poses to ROS topic or use as needed

        status = displayFrames(color_image)
        if not status:
            break

def main():
    
    cap = cv2.VideoCapture(0)
    
    # 1280x720 (HD)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)
    
    if cap.isOpened():
        processingLoop(cap)
    else:
        print("Failed to initialize D435i (No RGB Camera found!)")
    
if __name__ == "__main__":
    main()