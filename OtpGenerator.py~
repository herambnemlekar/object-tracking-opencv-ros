#!/usr/bin/env python
import roslib
roslib.load_manifest('otp')
import rospy
import sys, time, os
import numpy as np
from geometry_msgs.msg import PointStamped, Point
from std_msgs.msg import Time


class OtpGenerator:


    def __init__(self):
		
		#Publishing the estimated goal position and total trajectory time
        self.goal_pub = rospy.Publisher("/OtpGenerator/goal_position", Point, queue_size=1)
        self.time_pub = rospy.Publisher("/OtpGenerator/goal_time", Time, queue_size=1)
		
        self.D = []		#For Storing the Kinect sensor data 		#TODO????????
        self.offset = 0.1	#Hacked offset distance from the wrist to object
        self.t_demo = 1.5	#Total time of the demonstration
        self.baxter = []	#Baxter arm length 
        self.w = 1		#Weight factor for determining the confidence in the dynamic OTP

        self.data_sub = rospy.Subscriber("pyclient/skeleton_data", PointStamped, self.static, queue_size = 1) #Collecting sensor data
		
		self.static(data_sub)
		self.dynamic()

		
		
    def static(self,data):
	
		self.data = data
		
		#Formatting the sensor data
		self.D = np.append(self.D,self.data[1,1:4]) 
		self.D = self.D[(self.D.shape(0) - 2):(self.D.shape(0) - 1),:]
		
		#Object position
		self.P_obj = np.add(self.data[1, 1:3], [0, self.offset, 0])
		
		#Human wrist position: current and previous
		self.P_new = self.D[self.D.shape(0) - 1, 1:3]
		self.P_old = self.D[self.D.shape(0) - 2, 1:3]
		
		#Total time of the demonstration
		self.t_goal = self.t_demo
		
		#Extracting wrist and shoulder positions of human and baxter
		P1 = np.true_divide((self.data[0, 1:3] + self.baxter[2,0:2]), 2) #Right Shoulder of human & Left Shoulder of Baxter
		P2 = np.true_divide((self.data[2, 1:3] + self.baxter[0,0:2]), 2) #Left Shoulder of human & Right Shoulder of Baxter
		P3 = np.true_divide((self.data[1, 1:3] + self.baxter[3,0:2]), 2) #Right Wrist of human & Left Wrist of Baxter
		P4 = np.true_divide((self.data[3, 1:3] + self.baxter[1,0:2]), 2) #Left Wrist of human & Right Wrist of Baxter
		
		self.otp_s = np.true_divide((P1 + P2 + P3 + P4), 4) #OTP static calculation using human and baxter arm lengths and position
		
		#Distance between current location and goal position
		self.e_old = np.abs(og.P_old - og.otp_s)
		self.e_new = np.abs(og.P_new - og.otp_s)

	
	
    def dynamic(self):
	
	if (self.e_new - self.e_old) < 0.02:	#WHAT THIS DO????????

		dt = self.D[1, 4] - self.D[2, 4] 	#Time step
		self.w = self.w - ((self.w*dt)/self.t_goal)	#Calculating weights
		
		#Weighted average to find the goal
		self.P_goal = (self.w*self.otp_s) + ((1-self.w)*self.P_obj)
		V = (self.P_new - self.P_old)/dt	#Velocity 
		self.t_goal = (self.P_goal - self.P_obj)/V 	#Estimating total time of the trajectory

		#Publishing the goal position and estimated time (to the next block -DMP)
		self.goal_pub.publish(self.P_goal)
		self.time_pub.publish(self.t_goal)

		#return self.t_goal, self.P_goal
		
		
		
def main(args):
    og = OtpGenerator()
	
    rospy.init_node('OtpGenerator', anonymous=True)
    
	try:
        rospy.spin()
    except KeyboardInterrupt:
        print ("Shutting down")

if __name__ == '__main__':
    main(sys.argv)
