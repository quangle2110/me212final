#!/usr/bin/python


import rospy
import tf
import numpy as np
import threading
import serial
import pdb
import traceback
import sys
import tf.transformations as tfm

from me212bot.msg import WheelVelCmd
from apriltags.msg import AprilTagDetections
from geometry_msgs.msg import Pose
from std_msgs.msg import Float32, String, Float64
import me212helper.helper as helper
from human_robot_comm.msg import floatlist
from dynamixel_msgs.msg import JointState

r = 0.037
b = 0.225

purple_obs = [0.4,0.7,0.3,-1/0.25,1/0.25,1/0.25]
green_obs = [1.05,0.7,0.,-1/0.2,1/0.2,1/0.2]


class human_comm():
    def __init__(self):
        rospy.Subscriber('/gesture', String, self.gesture_callback) #make the callback
        self.gesture_str = ""
        #~ rospy.Subscriber('/velocity_track, floatlist, self.velocity_trackcallback) 
        
    def gesture_callback(self,msg):
        self.gesture_str = msg.data
        if self.gesture_str == "G B":
            print self.gesture_str

#~ def velocity_trackcallback():
    #transform wheelvelocity

class ObjectNavigator():
    def __init__(self, color='red'):
        self.listener = tf.TransformListener()
        self.br = tf.TransformBroadcaster()
        
        print color
        
        if color == 'red':
            self.object_sub = rospy.Subscriber("red", Pose, self.object_callback, queue_size=1)
        elif color == 'green':
            self.object_sub = rospy.Subscriber("green", Pose, self.object_callback, queue_size=1)
        elif color == 'blue':
            self.object_sub = rospy.Subscriber("blue", Pose, self.object_callback, queue_size=1)

        self.velcmd_pub = rospy.Publisher("/cmdvel", WheelVelCmd, queue_size=1)
        
        self.finished = False

        #~ self.thread = threading.Thread(target = self.navi_loop)
        self.navi_loop()

        #~ self.thread.start()

        rospy.sleep(1)

    def object_callback(self, msg):
        # pose of object w.r.t base
        object2base = helper.transformPose(self.listener, helper.pose2poselist(msg), '/camera', 'base_link')
        helper.pubFrame(self.br, pose=object2base, frame_id='/object', parent_frame_id='/base_link', npub=1)

    def navi_loop(self):
        robot_pos2d = [0, -0.04]
        thres_dist = 0.33
        thres_angle = 0.1
        k = 0.15
        rate = rospy.Rate(100)

        wv = WheelVelCmd()

        while not rospy.is_shutdown():
            #~ print 'object'

            object_pose3d = helper.lookupTransform(self.listener, '/base_link', '/object')

            if object_pose3d is None:
                #~ print 'object not in view'
                wv.desiredWV_R = 0
                wv.desiredWV_L = 0
                #~ self.velcmd_pub.publish(wv)
                #~ rate.sleep()
                #~ continue
            else:
                object_pos2d = np.array(object_pose3d[0:2])
                #~ print 'x, y =', object_pos2d

                dist = np.linalg.norm(object_pos2d - robot_pos2d)
                angle = np.arctan2(object_pos2d[1] - robot_pos2d[1], object_pos2d[0] - robot_pos2d[0])
                #~ print 'object =', object_pos2d
                #~ print 'angle =', angle

                if dist <= thres_dist and abs(angle) <= thres_angle:
                    self.finished = True
                    wv.desiredWV_R = 0
                    wv.desiredWV_L = 0
                    #~ break
                elif dist > thres_dist and abs(angle) <= thres_angle:
                    #~ print 'case 1'
                    wv.desiredWV_R = 0.2
                    wv.desiredWV_L = 0.2
                elif dist <= thres_dist and abs(angle) > thres_angle:
                    #~ print 'case 2'
                    wv.desiredWV_R = 0.2*np.sign(angle)
                    wv.desiredWV_L = -0.2*np.sign(angle)
                else:
                    #~ print 'case 3'
                    w = k*angle
                    vel = 0.2*r
                    wv.desiredWV_R = (vel+w*b)/r
                    wv.desiredWV_L = (vel-w*b)/r 
            
            self.velcmd_pub.publish(wv)
            
            if self.finished:
                print 'Get close enought to the object. Finished'
                break
            
            rate.sleep()


class ObstacleNavigator():
    def __init__(self, coeffs=[0.4,0.7,0.3,-1/0.25,1/0.25,1/0.25], backward=False, d=0.6):
        self.pathDist = 0
        
        self.path_sub = rospy.Subscriber("/pathDistance", Float32, self.dist_callback, queue_size=1)
        self.nav_pub =  rospy.Publisher("/cmdvel", WheelVelCmd, queue_size=1)
        
        self.coeffs = coeffs
        
        self.finished = False
        
        self.initial = True
        
        self.start = 0
        
        if backward:
            #~ self.thread = threading.Thread(target = self.backward(d))
            self.backward(d)
        else:
            #~ self.thread = threading.Thread(target = self.obstacle_loop(self.coeffs))
            self.obstacle_loop(self.coeffs)

        #~ self.thread.start()
        
        rospy.sleep(1)

    
    def dist_callback(self, msg):
        self.pathDist = msg.data
        if self.initial:
            self.start = self.pathDist
            self.initial = False
        
    def obstacle_loop(self, coeffs):
        
        d1, d2, d3, k1, k2, k3 = coeffs
     
        rate = rospy.Rate(100)
        
        while not rospy.is_shutdown():
            #~ print 'obstacle'
            vel = 0.2
            
            pathDist = self.pathDist - self.start
            #~ print 'pathDist=', pathDist
            
            if pathDist < d1:
                k = 0
            elif pathDist >= d1 and pathDist < d1+0.5*np.pi*abs(1/k1):
                k = k1
            elif pathDist >= d1+0.5*np.pi*abs(1/k1) and pathDist < d1+d2+0.5*np.pi*abs(1/k1):
                k  = 0
            elif pathDist >= d1+d2+0.5*np.pi*abs(1/k1) and pathDist < d1+d2+0.5*np.pi*(abs(1/k1)+1/k2):
                k = k2
            elif pathDist >= d1+d2+0.5*np.pi*(abs(1/k1)+1/k2) and pathDist < d1+d2+d3+0.5*np.pi*(abs(1/k1)+1/k2):
                k = 0
            elif pathDist >= d1+d2+d3+0.5*np.pi*(abs(1/k1)+1/k2) and pathDist < d1+d2+d3+0.5*np.pi*(abs(1/k1)+1/k2+1/k3):
                k = k3
            else:
                vel = 0
                k = 0
                self.finished = True


            wv = WheelVelCmd()
            wv.desiredWV_R, wv.desiredWV_L = self.computeVelocity(vel, k)
            self.nav_pub.publish(wv)
            
            if self.finished:
                break
            
            rate.sleep()
            
    def backward(self, d):
        
        rate = rospy.Rate(100)
        
        while not rospy.is_shutdown():
            #~ print 'backward'
            vel = -0.2
           
            pathDist = self.pathDist - self.start
            
            if pathDist > d:
                vel = 0
                self.finished = True
            
            wv = WheelVelCmd()
            wv.desiredWV_R, wv.desiredWV_L = self.computeVelocity(vel, 0)
            self.nav_pub.publish(wv)
            
            if self.finished:
                break
            
            rate.sleep()


    def computeVelocity(self, vel, k):
        wv_r = vel*(1+k*b)
        wv_l = vel*(1-k*b)
        return wv_r, wv_l
        
        

class ApriltagNavigator():
    def __init__(self, id, target=[0,0]):
        self.listener = tf.TransformListener()
        self.br = tf.TransformBroadcaster()
        
        self.id = id
        self.target = target
        
        self.apriltag_sub = rospy.Subscriber("/apriltags/detections", AprilTagDetections, self.apriltag_callback, queue_size = 1)
        
        self.velcmd_pub = rospy.Publisher("/cmdvel", WheelVelCmd, queue_size = 1)   ##
        
        #~ self.thread = threading.Thread(target = self.april_loop)
        self.april_loop()
            
        #~ self.thread.start()
        
        rospy.sleep(1)
        
    def apriltag_callback(self, data):
        # use apriltag pose detection to find where is the robot
        for detection in data.detections:
            if detection.id == self.id: 
                tag2base = helper.transformPose(self.listener, helper.pose2poselist(detection.pose), '/camera', 'base_link')
                helper.pubFrame(self.br, pose = helper.pose2poselist(detection.pose), frame_id = '/apriltag', parent_frame_id = '/camera', npub = 1)
                helper.pubFrame(self.br, pose = tag2base, frame_id = '/apriltag', parent_frame_id = '/base_link', npub = 1)
        
    def april_loop(self):
        robot_pos2d = self.target 
        thres_dist = 0.3
        thres_angle = 0.2
        thres_d = 0.05
        k = 0.2
        kp = 0.1
        kd = 0.25
        rate = rospy.Rate(100)

        wv = WheelVelCmd()

        while not rospy.is_shutdown():
            #~ print 'april'

            apriltag_pose3d = helper.lookupTransform(self.listener, '/base_link', '/apriltag')

            if apriltag_pose3d is None:
                #~ print 'tag not in view'
                wv.desiredWV_R = -0.1
                wv.desiredWV_L = 0.1
                self.velcmd_pub.publish(wv)
                rate.sleep()
                continue

            apriltag_pos2d = np.array(apriltag_pose3d[0:2])
            yaw = tfm.euler_from_quaternion(apriltag_pose3d[3:7])[2] + np.pi/2
            #~ print 'x, y =', apriltag_pos2d
            #~ print 'yaw =', yaw

            dist = np.linalg.norm(apriltag_pos2d - robot_pos2d)
            #~ angle = np.arctan2(apriltag_pos2d[1] - robot_pos2d[1], apriltag_pos2d[0] - rwsezobot_pos2d[0])
            angle = yaw
            #~ print 'object =', apriltag_pos2d
            #~ print 'angle =', angle
            
            d = - (apriltag_pos2d[0]-robot_pos2d[0])*np.sin(yaw) + (apriltag_pos2d[1]-robot_pos2d[1])*np.cos(yaw)
            #~ print 'd =', d

            if dist <= thres_dist and abs(angle) <= thres_angle and abs(d) <= thres_d:
                #~ print 'Get close enough to the object'
                wv.desiredWV_R = 0
                wv.desiredWV_L = 0
                self.velcmd_pub.publish(wv)
                print "finished tracking"
                break
            elif dist > thres_dist and abs(angle) <= thres_angle and abs(d) <= thres_d:
                #~ print 'case 1'
                wv.desiredWV_R = 0.2
                wv.desiredWV_L = 0.2
            elif abs(angle) > thres_angle and abs(d) <= thres_d:
                #~ print 'case 2'
                wv.desiredWV_R = 0.2*np.sign(angle)
                wv.desiredWV_L = -0.2*np.sign(angle)
            else:
                #~ print 'case 4'
                w = k*(kp*angle + kd * d)
                vel = 0.2*r
                wv.desiredWV_R = (vel+w*b)/r
                wv.desiredWV_L = (vel-w*b)/r 
            
            self.velcmd_pub.publish(wv)
            
            rate.sleep()

class Gripper():
    def __init__(self, thres=np.pi/6):
        
        self.pos_sub = rospy.Subscriber("/joint1_torque_controller/state", JointState, self.state_callback, queue_size = 1)
        
        self.grasp = rospy.Publisher("/joint1_torque_controller/command", Float64, queue_size = 1)
        
        self.thres = thres
        #~ self.is_open = mode
        self.start = 0
        self.pos = 0
        self.initial = True
        self.finished = False
        self.count = 0
        
        #~ self.gripper()
        
    def state_callback(self, msg):
        self.pos = msg.current_pos
        if self.pos > np.pi:
            self.pos = self.pos - 2*np.pi
        if self.initial:
            self.start = self.pos
            self.initial = False
            
    def gripper(self, is_open):
        
        thres_angle =  self.thres
        rate = rospy.Rate(100)
        
        
        while not rospy.is_shutdown():
            angle = self.pos - self.start
            v = Float64()
            print 'angle, thres', angle, thres_angle
            
            if not is_open:
                #~ print 'closing'
                v.data = -1
            else:
                if abs(angle) < thres_angle:
                    v.data = 1
                else:
                    v.data = 0
                    self.finished = True
            #~ else:
                #~ v.data = 0
                #~ self.finished = True
            
            self.count += 1
            self.grasp.publish(v)
            
            if self.finished:
                print 'Done', self.finished
                break
            
            if self.count == 100 and not is_open:
                #~ print 'keep closing'
                break
                
            rate.sleep()
                
            
        

def main():
    rospy.init_node('me212_robot', anonymous=True)
    hum = human_comm()
    ObstacleBool1 = True
    ObjectBool = True
    B2_Bool = True
    TowelGripBool = True
    print 'Obstacle pathing'
    while (ObstacleBool1):
        rospy.sleep(1)
        print 'Obstacle pathing'
        if hum.gesture_str == "G B\r\n":
            print "Hello"
            navi = ObstacleNavigator(green_obs)
            ObstacleBool1 = False
        if hum.gesture_str == "G A\r\n":
            print "Bye"
            navi = ObstacleNavigator(purple_obs)
            ObstacleBool1 = False
    
    print 'open'
    grasp = Gripper(thres=np.pi/12)
    grasp.gripper(True)
    print 'Start object tracking'
    while (ObjectBool):
        if hum.gesture_str == "G C\r\n":
            navi = ObjectNavigator(color='green')
            ObjectBool = False
        if hum.gesture_str == "G F\r\n":
            navi = ObjectNavigator(color='red')
            ObjectBool = False
        if hum.gesture_str == "G D\r\n":
            navi = ObjectNavigator(color='blue') 
            ObjectBool = False
    
    #~ navi = ObjectNavigator(color='red')
    grasp.gripper(False)
            
    print 'Backward 1'
    navi = ObstacleNavigator(backward = True, d = 0.2)
    print 'Apriltag 4 navigation'
    navi = ApriltagNavigator(4, target=[0,0.1])
    
    while (B2_Bool):
        if hum.gesture_str == "G B\r\n":
            #open gripper
            B2_Bool = False
            grasp.gripper(True)
            
    while (TowelGripBool):
        if hum.gesture_str == "G A\r\n":
            #close gripper around towel
            TowelGripBool = False
            grasp.gripper(False)
    print 'Backward 2'
    navi = ObstacleNavigator(backward = True, d = 1.0)
    print 'Pathing'
    navi = ObstacleNavigator(coeffs=[0,0.3,0,-1/0.25,np.inf,np.inf])
    print 'Apriltag 7 navigation'
    #~ #subscribe for commanding velocity transform
    navi = ApriltagNavigator(7, target=[0.4,0])
    grasp.gripper(True)
    rospy.spin()

if __name__=='__main__':
    main()
