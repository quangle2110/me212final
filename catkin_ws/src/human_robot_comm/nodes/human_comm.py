#!/usr/bin/env python
# license removed for brevity
import rospy
import numpy as np
from std_msgs.msg import String
from human_robot_comm.msg import floatlist
import serial


ser=serial.Serial('/dev/ttyACM1')

def talker(counter):
    gesture_str = ''
    velocity_list = []
    pub1 = rospy.Publisher('/gesture', String, queue_size=10)
    pub2 = rospy.Publisher('/velocity_track', floatlist, queue_size=10)
    rospy.init_node('human_comm', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        try:
            with serial.Serial('/dev/ttyACM1', 9600, timeout=1) as ser:
                line = ser.readline()

                if(counter<= 20):
                    #do nothing
                    counter = counter + 1
                else:
                    if(line[0]=='G'):
                        gesture_str = line
                        velocity_list = floatlist()
                        velocity_list.data = [float(),float(),float()]
                        pub1.publish(gesture_str)

                    else: 
                        gesture_str = ""
                        velocity_line = line
                        x,y,z = velocity_line.split(" ")
                        z,z1 = z.split("\r\n")
                        velocity_list = floatlist()
                        velocity_list.data = [float(x),float(y),float(z)]
                        pub2.publish(velocity_list)
        except:
            pass


        #hello_str = "hello world %s" % rospy.get_time()
        rospy.loginfo(gesture_str)
        rospy.loginfo(velocity_list)
        rate.sleep()

if __name__ == '__main__':
    cntr = 0
    try:
        talker(cntr)
    except rospy.ROSInterruptException:
        pass
