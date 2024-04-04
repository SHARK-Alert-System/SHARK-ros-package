#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix
from uav_pi.srv import Survey3, Survey3Request
from uav_pi.msg import ImageWithGPS
import cv2
import time
import datetime
##############################################################################
#2 for USB wide angle, 0 for MAPIR
# v4l2-ctl --list-devices
cam_num = 0

bridge = CvBridge()

# initialize the location to -1,-1,-1 
last_alt = -1
last_long = -1
last_lat = -1

def gps_callback(msg):
    """ Updates the gps location based on a NavSatFix message """
    # access global variables
    global last_alt
    global last_lat
    global last_long
    
    #update
    last_lat = msg.latitude
    last_long = msg.longitude
    last_alt = msg.altitude

def camera_publisher():
    time.sleep(1)
   
    rospy.Subscriber('gps_state', NavSatFix, gps_callback)
    rospy.init_node('camera_publisher', anonymous=True)
    pub = rospy.Publisher('camera_image', ImageWithGPS, queue_size=10)
    rate = rospy.Rate(1) # 1 Hz maximum
    bridge = CvBridge()

    rospy.loginfo("pi_cam_publisher: RPi cam publisher node initialized. Publishing images.\n")
    
    while not rospy.is_shutdown():  
        
        #capture the image (with set up and clean up)
        cap = cv2.VideoCapture(cam_num)
        ret, frame = cap.read()
        cap.release()

        # timestamp it
        photo_timestamp = rospy.get_rostime()

        # error logging
        if frame is None:
            rospy.logerr("pi_cam_publisher: image is null...")
        
        # formatted time stamp in string form (for saving image)
        formatted_photo_t = datetime.datetime.fromtimestamp(photo_timestamp.to_sec()).strftime('%m_%d_%Y_%H-%M-%S')

        #convert cv2 image to a ROS image
        ros_image = bridge.cv2_to_imgmsg(frame, "bgr8")
        ros_image.header.stamp = photo_timestamp

        # logging the photos to our runs folder. Name found at '/home/robertobrien/Documents/runs/run_name.txt' and set by talker.py
        f = open('/home/robertobrien/Documents/runs/run_name.txt', "r")
        run_name = f.read()
        cv2.imwrite('/home/robertobrien/Documents/runs/'+run_name+'/'+formatted_photo_t+'_image.jpg', frame)

        #Initialize an image with GPS metadata, and set values
        img_gps = ImageWithGPS()
        img_gps.image = ros_image
        img_gps.latitude = float(last_lat)
        img_gps.longitude = float(last_long)
        img_gps.altitude = float(last_alt)
        img_gps.fname = str(run_name+'/'+formatted_photo_t+'_image.jpg')

        #print(img_gps)

        pub.publish(img_gps)
        rate.sleep()

if __name__ == '__main__':
    rospy.wait_for_service('/object_detect', timeout=40)
    try:
        camera_publisher()
    except rospy.ROSInterruptException:
        pass
