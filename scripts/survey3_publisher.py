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
import sys
import os
import re
import subprocess
import datetime

CAMERA_DIRECTORY = '/media/sd_card/DCIM/Photo/'
LOCAL_PHOTO_DIRECTORY = '/home/robertobrien/Documents/Survey3_Photos/'

# Tips: if issues arise where the images aren't refreshing, reformat the camera's SD card
##############################################################################
# ubuntu usb device helpers

bridge = CvBridge()

last_alt = -1
last_long = -1
last_lat = -1

def gps_callback(msg):
    """ updates the gps location """
    # access global variables
    global last_alt
    global last_lat
    global last_long
    
    last_lat = msg.latitude
    last_long = msg.longitude
    last_alt = msg.altitude

def ubuntu_list_block_devices():
    """List all block devices, removing leading characters."""
    lsblk_output = subprocess.check_output(['lsblk', '-no', 'NAME'], text=True)
    cleaned_output = [re.sub(r'^[└─]+', '', line) for line in lsblk_output.splitlines()]
    return cleaned_output

def ubuntu_find_target_device(devices):
    """Find a device that matches the target pattern."""
    pattern = re.compile(r'^sd[a-z][0-9]$')
    for device in devices:
        if pattern.match(device):
            return '/dev/' + device
    return None

def ubuntu_is_already_mounted(device):
    """Check if the device is already mounted."""
    mounts = open('/proc/mounts').read()
    return device in mounts

def ubuntu_mount_device(device):
    """Mount the device to /media/sd_card using pmount."""
    subprocess.run(['pmount', device, 'sd_card'], check=True)

def ubuntu_unmount_device(mount_point='/media/sd_card'):
    """Unmount the device mounted at the given mount point using pumount."""
    try:
        subprocess.run(['pumount', mount_point], check=True)
        print(f"Successfully unmounted the device at {mount_point}.")
    except subprocess.CalledProcessError as e:
        print(f"Failed to unmount the device at {mount_point}: {e}")

def ubuntu_handle_mount():
    devices = ubuntu_list_block_devices()
    #print(devices)
    #print()
    target_device = ubuntu_find_target_device(devices)
    #print(target_device)
    #print()
    if target_device and not ubuntu_is_already_mounted(target_device):
        ubuntu_mount_device(target_device)
        print(f"Mounted {target_device} to /media/sd_card.")
    else:
        print("No suitable device found or device is already mounted.")

##############################################################################

def survey3_command_client(command):
    rospy.wait_for_service('/survey3_command')
    try:
        survey3_command_proxy = rospy.ServiceProxy('/survey3_command', Survey3)
        request = Survey3Request(command=command)
        response = survey3_command_proxy(request)
        return response.isMounted
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)


def trigger():
    """triggers a photo and returns the timestamp at which it was triggered"""
    mounted = survey3_command_client('t')
    timestamp = rospy.get_rostime()
    time.sleep(0.1)

    if mounted:  # If the camera is not mounted after taking a picture, for example
        rospy.loginfo("trigger(): Camera was still mounted. Now unmounting...")
        survey3_command_client('s')  # Attempt to mount or change state
        time.sleep(2)  # wait for 2 seconds before the next command for state stabilization
        survey3_command_client('t')  # Capture or trigger again if needed
        timestamp = rospy.get_rostime()
    rospy.loginfo("survey3_publisher.trigger(): sent trigger command")
    time.sleep(1.5)
    return timestamp

def survey_3_mount():
    mounted = survey3_command_client(' ') #just to get info

    if not mounted:  # If the camera is not mounted after taking a picture, for example
        #rospy.logerr("mount(): Camera was unmounted. Mounting...")
        survey3_command_client('s')  # Attempt to mount or change state
        time.sleep(1.5)  # Wait for 2 seconds before the next command for state stabilization
    rospy.loginfo("survey3_publisher.mount(): survey3 should now be mounted")
    return 1

def survey_3_unmount():
    mounted = survey3_command_client(' ') #just to get info

    if mounted:  # If the camera is not mounted after taking a picture, for example
        #rospy.logerr("mount(): Camera was unmounted. Mounting...")
        survey3_command_client('s')  # Attempt to mount or change state
        time.sleep(1.5)  # Wait for 2 seconds before the next command for state stabilization
    rospy.loginfo("survey3_publisher.ubmount(): survey3 should now be UNmounted")
    return 1

#returns an array of all file names on the camera, sorted
def listdir():
    return sorted(os.listdir(CAMERA_DIRECTORY))

# gets the most recently taken photo. Assumes filenames are date and timestamps
def get_recent_photo_path():
    #print(str(listdir()))
    fname = str(listdir().pop())
    return str(CAMERA_DIRECTORY + fname), fname

#gets a cv2 image of the most recently taken photo
def get_recent_photo_img():
    path, fname = get_recent_photo_path()
    return cv2.imread(path,0), path, fname

def camera_publisher():
    ubuntu_unmount_device()
    time.sleep(1)
   
    rospy.Subscriber('gps_state', NavSatFix, gps_callback)
    rospy.init_node('camera_publisher', anonymous=True)
    pub = rospy.Publisher('camera_image', Image, queue_size=10)
    rate = rospy.Rate(1) # 1 Hz maximum
    bridge = CvBridge()


    
    rospy.loginfo("survey3_publisher: Survey3 publisher node initialized.\n")
    
    while not rospy.is_shutdown():

        try:    
            photo_timestamp = trigger() #trigger photo
            formatted_photo_t = datetime.datetime.fromtimestamp(photo_timestamp.to_sec()).strftime('%m_%d_%Y_%H-%M-%S')

            survey_3_mount() #mount on the survey3 side
            time.sleep(2)
            ubuntu_handle_mount()#mount the usb device on the ubuntu side
            time.sleep(1)
            img, path, fname = get_recent_photo_img() # gets the recent survey3 image

            img = cv2.imread(path)
            print("We got the image!")

            ubuntu_unmount_device() #unmounts USB device
            survey_3_unmount()      #unmounts from survey3 side
            if img is None:
                rospy.logerr("survey3_publisher: image is null...")
            ros_image = bridge.cv2_to_imgmsg(img, "bgr8")
            ros_image.header.stamp = photo_timestamp
            #ros_image.header.frame_id = str(run_name+'/'+formatted_photo_t+'_image.jpg')

            pub.publish(ros_image)
            rospy.loginfo("survey3_publisher: published image " + path +  " to /camera_image\n")
            rate.sleep()

            # logging the photos to our runs folder
            f = open('/home/robertobrien/Documents/runs/run_name.txt', "r")
            run_name = f.read()
            cv2.imwrite('/home/robertobrien/Documents/runs/'+run_name+'/'+formatted_photo_t+'_image.jpg', img)

            print("now we get expirimental")
            print(last_lat)
            img_gps = ImageWithGPS()
            
            #expirimental
            img_gps.image = ros_image # comment out if you just want to debug the other stuff
            img_gps.latitude = float(last_lat)
            img_gps.longitude = float(last_long)
            img_gps.altitude = float(last_alt)
            img_gps.fname = str(run_name+'/'+formatted_photo_t+'_image.jpg')
            #print(img_gps)
            time.sleep(1)
        except:
            rospy.logerr("survey3_publisher: Error, unmounting mounts and starting loop over.")
            ubuntu_unmount_device()

if __name__ == '__main__':
    try:
        camera_publisher()
    except rospy.ROSInterruptException:
        pass
