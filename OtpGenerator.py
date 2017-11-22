#!/usr/bin/env python
import roslib
roslib.load_manifest('otp')
import rospy
import sys, time, os
import numpy as np
from kinect_data.msg import skeleton
from geometry_msgs.msg import PointStamped, Point
from std_msgs.msg import Float64, UInt64


class OtpGenerator:


    def __init__(self):
		
	#Publishing the estimated goal position and total trajectory time
        self.goal_pub = rospy.Publisher("/OtpGenerator/goal_position", Float64, queue_size=1)
        self.time_pub = rospy.Publisher("/OtpGenerator/goal_time", Float64, queue_size=1)
		
        self.D = np.matrix([0, 0, 0, 0])		#For Storing the Kinect sensor data 		
        self.offset = 0.1	#Hacked offset distance from the wrist to object
        self.t_demo = 1.5	#Total time of the demonstration
        self.baxter = np.matrix([[2,3,4],[3,4,5],[4,5,6],[5,6,7]])	#Baxter arm length 
        self.w = 1.0		#Weight factor for determining the confidence in the dynamic OTP
	self.i = 0.0              #Initialization for otp loop
	
	#Subscribing to the skeleton data from kinect
        self.data_sub = rospy.Subscriber("/skeleton_data", skeleton, self.static, queue_size = 1)	
		
    def static(self,data):

	self.data = data
	
	#Sorting skeleton data
	P_rs = np.matrix([self.data.joints[0].x, self.data.joints[0].y, self.data.joints[0].z, float(self.data.joints[0].stamp)])
	self.P_rw = np.matrix([self.data.joints[3].x, self.data.joints[3].y, self.data.joints[3].z, float(self.data.joints[3].stamp)])
	P_ls = np.matrix([self.data.joints[2].x, self.data.joints[2].y, self.data.joints[2].z, float(self.data.joints[2].stamp)])
	P_lw = np.matrix([self.data.joints[1].x, self.data.joints[1].y, self.data.joints[1].z, float(self.data.joints[1].stamp)])
		
	#Formatting the sensor data
	self.D = np.append(self.D, self.P_rw, axis=0)
	self.D = self.D[(np.shape(self.D)[0] - 2):np.shape(self.D)[0],:]

	if self.i==0:
		#Extracting wrist and shoulder positions of human and baxter
		P1 = np.true_divide((P_rs[0,0:3] + self.baxter[2,0:3]), 2.0) #Right Shoulder of human & Left Shoulder of Baxter
		P2 = np.true_divide((self.P_rw[0,0:3] + self.baxter[0,0:3]), 2.0) #Left Shoulder of human & Right Shoulder of Baxter
		P3 = np.true_divide((P_ls[0,0:3] + self.baxter[3,0:3]), 2.0) #Right Wrist of human & Left Wrist of Baxter
		P4 = np.true_divide((P_lw[0,0:3] + self.baxter[1,0:3]), 2.0) #Left Wrist of human & Right Wrist of Baxter
	
		self.otp_s = np.true_divide((P1 + P2 + P3 + P4), 4.0) #OTP static calculation 
		self.i = self.i + 1
		
		print(self.otp_s)
		
	self.dynamic()

	
    def dynamic(self):

	#Object position
	self.P_obj = np.add(self.P_rw[0,0:3], [0, self.offset, 0])
		
	#Human wrist position: current and previous
	self.P_new = self.D[1, 0:3]
	self.P_old = self.D[0, 0:3]
	
	#Total time of the demonstration
	self.t_goal = self.t_demo
	
	#Distance between current location and goal position
	self.e_old = np.abs(self.P_old - self.otp_s)
	self.e_new = np.abs(self.P_new - self.otp_s)
	
	if (self.e_new - self.e_old).any() > 0.02:	#Arbitrary threshold for detecting movement

		dt = self.D[1, 3] - self.D[0, 3] 	#Time step
		self.w = self.w - ((self.w*dt)/self.t_goal)	#Calculating weights
		
		#Weighted average to find the goal
		self.P_goal = (self.w*self.otp_s) + ((1-self.w)*self.P_obj)
		V = (self.P_new - self.P_old)/dt	#Velocity 
		self.t_goal = (self.P_goal - self.P_obj)/V 	#Estimating total time of the trajectory

		#Publishing the goal position and estimated time (to the next block -DMP)
		self.goal_pub.publish(self.P_goal)
		self.time_pub.publish(self.t_goal)

		print(self.P_goal)

		#return self.t_goal, self.P_goal
		
		
		
def main(args):  
    rospy.init_node('OtpGenerator', anonymous=True)
    og = OtpGenerator()	
    try:
	rospy.spin()
    except KeyboardInterrupt:
        print ("Shutting down")

if __name__ == '__main__':
    main(sys.argv)
