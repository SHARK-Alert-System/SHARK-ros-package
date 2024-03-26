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

bridge = CvBridge()

last_alt = -1
last_long = -1
last_lat = -1
first = True

def gps_callback(msg):
    """ updates the gps location """
    # access global variables
    global last_alt
    global last_lat
    global last_long
    
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

    rospy.loginfo("pi_campublisher: RPI cam publisher node initialized.\n")

    
    while not rospy.is_shutdown():  

        cap = cv2.VideoCapture('/dev/video0', cv2.CAP_V4L)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 2560)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1440)
        ret, frame = cap.read()
        cap.release()

        photo_timestamp = rospy.get_rostime()

        cv2.imwrite('/home/robertobrien/Documents/tests/image.jpg', frame)
        
        formatted_photo_t = datetime.datetime.fromtimestamp(photo_timestamp.to_sec()).strftime('%m_%d_%Y_%H-%M-%S')

        if frame is None:
            rospy.logerr("survey3_publisher: image is null...")
        ros_image = bridge.cv2_to_imgmsg(frame, "bgr8")
        ros_image.header.stamp = photo_timestamp


        # logging the photos to our runs folder
        f = open('/home/robertobrien/Documents/runs/run_name.txt', "r")
        run_name = f.read()
        cv2.imwrite('/home/robertobrien/Documents/runs/'+run_name+'/'+formatted_photo_t+'_image.jpg', frame)

        #print(last_lat)
        img_gps = ImageWithGPS()
        
        #expirimental
        img_gps.image = ros_image # comment out if you just want to debug the other stuff
        img_gps.latitude = float(last_lat)
        img_gps.longitude = float(last_long)
        img_gps.altitude = float(last_alt)
        img_gps.fname = str(run_name+'/'+formatted_photo_t+'_image.jpg')

        #print(img_gps)

        pub.publish(img_gps)
        rate.sleep()

if __name__ == '__main__':
    try:
        camera_publisher()
    except rospy.ROSInterruptException:
        pass
