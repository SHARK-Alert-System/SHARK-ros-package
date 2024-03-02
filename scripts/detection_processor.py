#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from uav_pi.msg import Detection
from uav_pi.srv import ObjectDetect, ObjectDetectRequest
from cv_bridge import CvBridge

class ImageDetectionProcessor:
    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/camera_image", Image, self.image_callback)
        self.detection_pub = rospy.Publisher("/detections", Detection, queue_size=10)
        self.is_request_processing = False  # Flag to indicate if a request is being processed, makes sure we are only doing one at a time.
        rospy.loginfo("detection_processor: Node ready to facilitate processing")


    def image_callback(self, image_msg):
        if self.is_request_processing:
            # If we are already processing a request, don't do anything
            return

        #set the flag to True to block new requests until this one is processed
        self.is_request_processing = True
        try:
            rospy.wait_for_service('/object_detect', timeout=40)
            detect_service = rospy.ServiceProxy('/object_detect', ObjectDetect)
            detect_request = ObjectDetectRequest()
            detect_request.image = image_msg  # send ros images
            response = detect_service(detect_request)
            rospy.loginfo("detection_processor: image send to detect-pt. Response was: " + str(response))
        except rospy.ServiceException as e:
            rospy.logerr("detection_processor: Service call failed: %s" % e)
            self.is_request_processing = False  # Reset the flag due to service call failure
            return

        #sends a detection for each bounding box. Assumes could be more than one
        for i in range(len(response.x1s)):
            detection_msg = Detection()
            detection_msg.x1 = response.x1s[i]
            detection_msg.y1 = response.y1s[i]
            detection_msg.x2 = response.x2s[i]
            detection_msg.y2 = response.y2s[i]
            detection_msg.conf = response.confs[i]
            detection_msg.label = response.labels[i]
            detection_msg.fname = response.fnames[i]
            # print(detection_msg.x1, "," , detection_msg.y1, " : ", detection_msg.x2, ",", detection_msg.y2)

            self.detection_pub.publish(detection_msg)
        self.is_request_processing = False  # Reset the flag as the request has been processed

def main():
    rospy.init_node('image_detection_processor', anonymous=True)
    processor = ImageDetectionProcessor()
    rospy.spin()

if __name__ == '__main__':
    main()
