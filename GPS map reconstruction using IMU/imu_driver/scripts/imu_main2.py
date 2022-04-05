#!/usr/bin/env python

# Code Author: Vignesh Ravikumar
# NU ID: 002194592
# Course: EECE5554-Robot Sensing & Navigation
# Question: Lab -1


import serial
import rospy
import datetime
import utm
from imu_driver.msg import imu

ser = serial.Serial('/dev/ttyUSB2')
ser.baudrate = 115200
line_splitter = []

global latitude_deg
msg = imu()
def talker():
    pub = rospy.Publisher('imu_message', imu, queue_size=10) #declare a topic /gps_message to publish the values from GPS puck to gps.msg file
    rospy.init_node('imu_talker', anonymous=True)
    r = rospy.Rate(40)
       

    while not rospy.is_shutdown():
            
        input_serial = str(ser.readline())
        line_splitter = input_serial.split("b'") #every line gets this b', hence used it as a delimiter
        comma_splitter = line_splitter[1].split(",") #to split each line with commas to get latitude, longitude easily
    
        # print(line_splitter)

        yaw = float(comma_splitter[1])
        pitch = float(comma_splitter[2])
        roll = float(comma_splitter[3])
        magx = float(comma_splitter[4])
        magy = float(comma_splitter[5])
        magz = float(comma_splitter[6])
        accx = float(comma_splitter[7])
        accy = float(comma_splitter[8])
        accz = float(comma_splitter[9])
        gyrox = comma_splitter[9]
        gyrox = float(gyrox[:10])
        gyroy = comma_splitter[11]
        gyroy = float(gyroy[:10])
        gyroz = comma_splitter[12]
        gyroz = float(gyroz[:10])
        

        msg.yaw = float(yaw)
        msg.pitch = float(pitch)
        msg.roll = float(roll)
        msg.magx = float(magx)
        msg.magy = float(magy)
        msg.magz = float(magz)
        msg.accx = float(accx)
        msg.accy = float(accy)
        msg.accz = float(accz)
        msg.gyrox = float(gyrox)
        msg.gyroy = float(gyroy)
        msg.gyroz = float(gyroz)
        rospy.loginfo(msg)
        pub.publish(msg)
        r.sleep()


if __name__ == '__main__':

    talker()

