#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
import random



# this is a node to just publish test images to the topic '/camera_image'
def publish_image(folder_path, topic_name):
    rospy.init_node('image_publisher', anonymous=True)
    pub = rospy.Publisher(topic_name, Image, queue_size=10)
    rate = rospy.Rate(1)  # 1 Hz

    #Initialize the bridge between ROS and OpenCV
    bridge = CvBridge()

    # List all files in the directory
    files = [f for f in os.listdir(folder_path) if os.path.isfile(os.path.join(folder_path, f))]
    if not files:
        rospy.logerr("image_publisher: No files found in the folder: {}".format(folder_path))
        return
    
    # Pick a random image from the list
    image_path = os.path.join(folder_path, random.choice(files))

    # Load the image with OpenCV
    cv_image = cv2.imread(image_path)

    if cv_image is None:
        rospy.logerr("image_publisher: Failed to load image from path: {}".format(image_path))
        return

    # Convert the OpenCV image to a ROS message
    ros_image = bridge.cv2_to_imgmsg(cv_image, "bgr8")
    
    rospy.loginfo("image_publisher: Running image publisher: {}".format(topic_name))

    while not rospy.is_shutdown():
        pub.publish(ros_image)
        # rospy.loginfo("Published image to topic: {}".format(topic_name))
        rate.sleep()

if __name__ == "__main__":
    folder_path = "/home/robertobrien/Documents/validate/"
    topic_name = "camera_image"
    try:
        publish_image(folder_path, topic_name)
    except rospy.ROSInterruptException:
        pass
