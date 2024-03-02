#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Temperature
import smbus2

#register adresses
MLX90614_I2CADDR = 0x5A
MLX90614_TA = 0x06 #ambient temp
MLX90614_TOBJ1 = 0x07#object temp2
MLX90614_TOBJ2 = 0x08#object temp2
MLX90614_EMIS = 0x04#emissivity register
EMISSIVITY = 0.96#https://scienceofdoom.com/2010/12/27/emissivity-of-the-ocean/#:~:text=From%20quite%20ancient%20data%2C%20the,speed%20and%20sea%20surface%20roughness.


def read_temp(reg_addr):
    try:
        """Read temperature from the given register address."""
        temp = bus.read_word_data(MLX90614_I2CADDR, reg_addr)
        temp = (temp * 0.02) - 273.15
        return temp
    except:
        rospy.logerr("OSError: [Errno 121] Remote I/O error")
        return None

def read_ambient_temp():
    """Read the ambient temperature."""
    #print(read_temp(MLX90614_TA))
    return read_temp(MLX90614_TA)

def read_object2_temp():
    """Read the object 2 temperature."""
    return read_temp(MLX90614_TOBJ2)

def read_object1_temp():
    """Read the object 1 temperature."""
    return read_temp(MLX90614_TOBJ1)

def write_emissivity(epsilon):
    bus.write_word_data(MLX90614_I2CADDR, MLX90614_EMIS, 0x00)
    val = round(65535 * epsilon)
    bus.write_word_data(MLX90614_I2CADDR, MLX90614_EMIS, val)
    rospy.loginfo("Changed emissivity to " + str(val) + " to " +  str(MLX90614_EMIS) + ".")

def mlx90614_publisher():
    rospy.init_node('mlx90614_publisher', anonymous=True)
    pub_ambient = rospy.Publisher('ambient_temperature', Temperature, queue_size=10)
    pub_object = rospy.Publisher('object_temperature', Temperature, queue_size=10)
    seq = 0
    rate = rospy.Rate(1) 

    while not rospy.is_shutdown():
        ambient_temp = read_ambient_temp()
        a_time = rospy.get_rostime()
        object1_temp = read_object1_temp()
        o_time = rospy.get_rostime()
        #object2_temp = read_object1_temp()
        ambient_temp_msg = Temperature()
        object_temp_msg = Temperature()


        ambient_temp_msg.temperature = round(ambient_temp,2)
        ambient_temp_msg.variance = 0
        ambient_temp_msg.header.frame_id = "ambient_temp"
        ambient_temp_msg.header.stamp = a_time
        ambient_temp_msg.header.seq = seq

        object_temp_msg.temperature = round(object1_temp,2)
        object_temp_msg.variance = 0
        object_temp_msg.header.frame_id = "object_temp"
        object_temp_msg.header.stamp = o_time
        object_temp_msg.header.seq = seq
        

        rospy.loginfo("Ambient Temperature: {:.2f} C".format(ambient_temp))
        rospy.loginfo("Object1 Temperature: {:.2f} C".format(object1_temp))
        rospy.loginfo("")

        if ambient_temp_msg.temperature is not None:
            pub_ambient.publish(ambient_temp_msg)
        if object_temp_msg.temperature is not None:
            pub_object.publish(object_temp_msg)

        seq = seq+1
        rate.sleep()

if __name__ == '__main__':
    while 1:
        try:
            #initialize bus 
            rospy.loginfo("MLX_Publisher: Starting up bus")
            bus = smbus2.SMBus(1)
            #write_emissivity(EMISSIVITY) #set the emissivity
            read_ambient_temp()
            mlx90614_publisher()
        except rospy.ROSInterruptException:
            rospy.logerr("There was an error. Trying to start again...")
            pass

