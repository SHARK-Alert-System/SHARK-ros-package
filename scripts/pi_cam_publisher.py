#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

def capture_image():
    # Initialize the video capture with the first camera device
    cap = cv2.VideoCapture(0)
    
    # Set the resolution (optional, uncomment if needed)
    # cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    # cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    
    # Capture a single frame
    ret, frame = cap.read()
    
    # Release the capture after use
    cap.release()
    
    if ret:
        return frame
    else:
        return None

def camera_publisher():
    rospy.init_node('camera_publisher', anonymous=True)
    pub = rospy.Publisher('camera_image', Image, queue_size=10)
    rate = rospy.Rate(1) # 1 Hz
    bridge = CvBridge()
    
    print("Publishing photos.")
    while not rospy.is_shutdown():
        img = capture_image()
        if img is not None:
            ros_image = bridge.cv2_to_imgmsg(img, "bgr8")
            pub.publish(ros_image)
        rate.sleep()

if __name__ == '__main__':
    try:
        camera_publisher()
    except rospy.ROSInterruptException:
        pass
