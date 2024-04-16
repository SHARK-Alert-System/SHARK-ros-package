#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from uav_pi.msg import ImageWithGPS
import cv2
import os
import random



# this is a node to just publish test images to the topic '/camera_image', read from a file
def publish_image(folder_path, topic_name):
    rospy.init_node('image_publisher', anonymous=True)
    pub = rospy.Publisher(topic_name, ImageWithGPS, queue_size=10)
    rate = rospy.Rate(0.5)  # 1 Hz

    pub_raw = rospy.Publisher("raw_camera_image", Image, queue_size=10)

    #Initialize the bridge between ROS and OpenCV
    bridge = CvBridge()

    # List all files in the directory
    files = [f for f in os.listdir(folder_path) if os.path.isfile(os.path.join(folder_path, f))]
    if not files:
        rospy.logerr("image_publisher: No files found in the folder: {}".format(folder_path))
        return
    
    while True:
        # Pick a random image from the list
        image_path = os.path.join(folder_path, random.choice(files))


        # Load the image with OpenCV
        cv_image = cv2.imread(image_path)

        if cv_image is None:
            rospy.logerr("image_publisher: Failed to load image from path: {}".format(image_path))
            return

        # Convert the OpenCV image to a ROS message
        ros_image = bridge.cv2_to_imgmsg(cv_image, "bgr8")

        #publish the raw image too
        pure_image = Image()
        pure_image.data = ros_image
        pub_raw.publish(ros_image) # publishes the raw ros image (for visualization)

        img_gps = ImageWithGPS()

        img_gps.image=ros_image
        img_gps.latitude = 40.204839
        img_gps.longitude = -74.36424
        img_gps.altitude = 30
        img_gps.fname = "image_name.jpeg"
        
        rospy.loginfo("image_publisher: Running image publisher: {}".format(topic_name))
        rospy.loginfo(image_path)
            
        pub.publish(img_gps)
        rospy.loginfo("image_publisher: Publishing image")
        rospy.loginfo("Publishing: " + str(image_path))
        # rospy.loginfo("Published image to topic: {}".format(topic_name))
        rate.sleep()

if __name__ == "__main__":
    folder_path = "/home/robertobrien/Documents/april-7-test-images"
    topic_name = "camera_image"
    publish_image(folder_path, topic_name)