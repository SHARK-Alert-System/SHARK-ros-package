#!/usr/bin/env python

from __future__ import print_function

import rospy
import cv2
import torch
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from uav_pi.srv import ObjectDetect, ObjectDetectResponse
import numpy as np
import datetime

# Assuming these functions are defined in another module or earlier in this script
# get_model, infer, draw_bounding_box, show_image, write_image

def get_model(path='best.pt'):
    model = torch.hub.load('ultralytics/yolov5', 'custom', path=path)    
    #print("model:")
    #print(model)
    return model

def infer(model,img):
    #img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    results = model(img) # this can take either an image or a path or an array of both
    return results

# draw bounding boxes and labels
def draw_bounding_box(img, model, results):
    copy = img.copy()
    for *xyxy, conf, cls in results.xyxy[0]:
        x1, y1, x2, y2 = map(int, xyxy)
        label = model.names[int(cls)]  #class label
        #print("label: ", label)
        color = (255, 0, 0)  #blue color (BRG)
        cv2.rectangle(copy, (x1, y1), (x2, y2), color, 2)
        cv2.putText(copy, f'{label} {conf:.2f}', (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
    return copy

def show_image(img):
    cv2.imshow('YOLOv5 Detection', img)
    cv2.waitKey(0)  # Wait for a key press to close the window
    cv2.destroyAllWindows()

def write_image(img):
    f = open('/home/robertobrien/Documents/runs/run_name.txt', "r")
    run_name = f.read()
    formatted_photo_t = datetime.datetime.fromtimestamp(rospy.get_rostime().to_sec()).strftime('%m_%d_%Y_%H-%M-%S')
    cv2.imwrite('/home/robertobrien/Documents/runs/'+run_name+'/' + formatted_photo_t+"_image_detected.jpg", img.copy())
    return formatted_photo_t


#get the mofel
model = get_model(path='/home/robertobrien/catkin_ws/src/uav_pi/weights/best.pt')  # load model in
bridge = CvBridge()  
detections_f_path = ""

def handle_object_detect(req):
    global detections_f_path
    rospy.loginfo("detect-pt: Got a request to detect objects")

    try:
        cv_image = bridge.imgmsg_to_cv2(req.image, "bgr8")  # convert ROS image message to OpenCV image
    except CvBridgeError as e:
        rospy.logerr("detect-pt: " + str(e))

    detections = infer(model, cv_image)  # run object detection
    detections.print() # print out the inference

    x1s = []
    y1s = []
    x2s = []
    y2s = []
    confs = []
    labels = []
    fnames = []

    # save im
    bb_img = draw_bounding_box(cv_image, model, detections)
    photo_fname = write_image(bb_img)  # Or show_image(bb_img)

    t = detections.t
    detection_data = open(detections_f_path, "w")
    detection_data.write(t[0] + "," + t[1] + "," + t[2]"\n")
    detection_data.close()
    print("")

    for *xyxy, conf, cls in detections.xyxy[0]:
        try:
            if conf < 0.5: 
                continue
            x1, y1, x2, y2 = map(int, xyxy)
            label = model.names[int(cls)]
            #print("Ahh", x1, y1, x2, y2)
            x1s.append(x1)
            y1s.append(y1)
            x2s.append(x2)
            y2s.append(y2)
            confs.append(conf)
            labels.append(label)
            fnames.append(photo_fname)
        except:
            x1s = [0]
            y1s = [0]
            x2s = [0]
            y2s = [0]
            confs = [0]
            labels = ["NaN"]
            fnames = ["NaN"]
            rospy.logerr("detect-pt: error in detection")
    response = ObjectDetectResponse()
    response.x1s = x1s
    response.y1s = y1s
    response.x2s = x2s
    response.y2s = y2s
    response.confs = confs
    response.labels = labels
    response.fnames = fnames

    rospy.loginfo("detect-pt: sending back response (ObjectDetectResponse)")
    print(response)
    #print()

    return response

def object_detect_server():
    rospy.init_node('object_detect_server')
    f = open('/home/robertobrien/Documents/runs/run_name.txt', "r")
    run_name = f.read()
    f.close()
    detections_f_path = "/home/robertobrien/Documents/runs/" + run_name + "/inference_time.txt"
    detection_data = open(detections_f_path, "w")
    detection_data.write("preprocessing,inference,postprocessing\n")
    detection_data.close()

    s = rospy.Service('object_detect', ObjectDetect, handle_object_detect)
    rospy.loginfo("detect-pt: Ready to detect objects.")
    rospy.spin()

if __name__ == "__main__":
    object_detect_server()
