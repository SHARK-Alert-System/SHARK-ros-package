#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import BatteryState, NavSatFix
from pymavlink import mavutil
import time

rospy.init_node('uav_info_publisher', anonymous=True)
pub_battery = rospy.Publisher('battery_state', BatteryState, queue_size=10)
pub_GPS = rospy.Publisher('gps_state', NavSatFix, queue_size=10)

connection_string = "/dev/ttyACM0"
vehicle = mavutil.mavlink_connection(connection_string)

# enable to send fake info for degubbing indoors w/o GPS access. 
enable_fake_gpsinfo = False
starting_lat = 37.7749
starting_long = -122.4194
dlat = 0.001
dlong = 0.001

# Function to handle and print GPS data
def handle_gps_data(msg):
    print(f"GPS: Lat: {msg.lat / 1e7}, Lon: {msg.lon / 1e7}, Alt: {msg.alt / 1e3} meters")


#check connection 
vehicle.wait_heartbeat()
print("Heartbeat from vehicle received.")

while True:
    msg = vehicle.recv_match(blocking=True)
    #print(msg)
    stamp = rospy.get_rostime()
    pub = None

    #handling of no message 
    if not msg:
        continue


    
    if enable_fake_gpsinfo:
        pub = NavSatFix()
        pub.header.stamp = stamp
        starting_lat = starting_lat + dlat
        starting_long = starting_long + dlong
        pub.latitude = starting_lat
        pub.longitude = starting_long
        pub.altitude =  30.45
        pub_GPS.publish(pub)
        print(starting_long)

    if msg.get_type() == 'BATTERY_STATUS':
        pub = BatteryState()
        pub.temperature = msg.temperature
        pub.voltage = msg.voltages[0]
        pub.cell_voltage =  msg.voltages
        pub.percentage = msg.battery_remaining
        pub_battery.publish(pub)
    elif msg.get_type() == 'GPS_RAW_INT':
        print(msg)
        pub = NavSatFix()
        pub.header.stamp = stamp
        pub.latitude = msg.lat / 1e7
        pub.longitude = msg.long / 1e7
        pub.altitude =  msg.alt / 1e3
        pub_GPS.publish(pub)
    #elif msg.get_type() == 'VFR_HUD':

    time.sleep(0.1)  #Sleep a little to not overload the CPU
