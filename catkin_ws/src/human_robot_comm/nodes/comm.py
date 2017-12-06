import rospy
from std_msgs.msg import String
from human_robot_comm.msg import floatlist

import numpy as np
import os

class human_comm:
    def __init__(self):
        self.counter = 0
        self.time_between_messages = .05 #seconds between velocity messages
        rospy.Subscriber('/arm/real/gripperPosition', String, self.gripperPos_callback)
        rospy.Subscriber('/arm/real/endPosition',String, self.endPos_callback)
        rospy.Subscriber('/gesture', String, self.gesture_callback) #make the callback
        rospy.Subscriber('/velocity_track', floatlist, self.velocitytrack_callback) 
        self.gripperPosPub = rospy.Publisher('/arm/target/gripperPosition', String, queue_size=1)
        self.endPosition_pub = rospy.Publisher('/arm/target/endPosition', String, queue_size=1)
        self.endCurrentPos = [float(), float(), float()]
        self.gripperCurrentPos = float()

    def gripperPos_callback(self,msg):
        try:
            self.gripperCurrentPos = float(msg.data)

        except:
            print "Exception in gripperPos_callback"

    def endCurrentPos_callback(self,msg):
        #stores current position to be used for velocity tracking
        try:
            currentPosList = msg.data.split(',')
            self.endCurrentPos = [float(currentPosList[0]), float(currentPosList[1]), float(currentPosList[2])]

        except:
            print "Exception in endCurrentPos_callback"

    def velocitytrack_callback(self,msg):
        #process velocity to be waypoint publishing. publish 1 point at a time. 
        try:
            targetGripperVel = float(msg.data)
            targetGripperVelX = targetGripperVel[0]
            targetGripperVelY = targetGripperVel[1]
            targetGripperVelZ = targetGripperVel[2]
            Xtarget = str(self.endCurrentPos[0] + targetGripperVelX*self.time_between_messages)
            Ytarget = str(self.endCurrentPos[1])
            Ztarget = str(self.endCurrentPos[2])
            Target = Xtarget + ',' + Ytarget + ',' + Ztarget
            if(self.counter == 2):
                #track velocity by publishing way points
                self.gripperPosPub(Target)
        except: 
            print "Exception in velocitytrack_callback"

    def gesture_callback(self, msg):
    	#fill this in
    	try:
    		if (self.gesture_counter==0):
    			if(msg.data == "G C"):
    				#begin reaching for the drawer (Task A-1)
                    self.endPosition_pub()
                    self.gripperPosPub("0")
    				counter+=1 #add 1 to the counter

    		if (self.gesture_counter==1):
    			if(msg.data == "G D"):
    				#use velocity tracking to pull drawer at velocity.
                    #velocitytrack_callback takes over
    				counter+=1

    		if (self.gesture_counter==2):
    			if(msg.data != "G D"):
    				#add 1 to the counter, the drawer is no longer being pulled.
                    self.gripperPosPub("1")
    				counter+=1 #proceed to waiting for Task A-2

    		if (self.gesture_counter==3):
    			if(msg.data == "G C"):
    				#begin Task A-2
                    #send one way point
    				counter+=1 

    		if (self.gesture_counter==4):
    			if(msg.data == "G D"):
    				#close grippers
                    self.gripperPosPub("0")
    				counter+=1

    		if (self.gesture_counter==5):
    			if(msg.data == "G B"):
    				#begin wrapping procedure
                    #send series of way points
    				counter+=1

    		if (self.gesture_counter==6):
    			if(msg.data != "G B"):
    				#stop wrapping procedure
                    self.gripperPosPub("1")
    				counter+=1 #completed with all tasks



if __name__ = "__main__":
    try:
        rospy.init_node("comm")
        comm = human_comm()
        rospy.spin()
    except:
        pass