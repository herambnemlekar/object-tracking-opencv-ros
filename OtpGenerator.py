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

        self.goal_pub = rospy.Publisher("/OtpGenerator/goal_position", Point, queue_size=1)
        self.time_pub = rospy.Publisher("/OtpGenerator/goal_time", Time, queue_size=1)

        self.D = []
        self.offset = 0.1
        self.t_demo = 1.5
        self.baxter = []
        self.w = 1

        self.data_sub = rospy.Subscriber("pyclient/skeleton_data", PointStamped, self.static, queue_size = 1)

    def static(self,data):
	
	self.data = data
	
	self.D = np.append(self.D,self.data[1,1:4])
        self.D = self.D[(self.D.shape(0) - 2):(self.D.shape(0) - 1),:]

        self.P_obj = np.add(self.data[1, 1:3], [0, self.offset, 0])

        self.P_new = self.D[self.D.shape(0) - 1, 1:3]
        self.P_old = self.D[self.D.shape(0) - 2, 1:3]

        self.t_goal = self.t_demo

        #Ph_rs = self.data[0, 1:3]
        #Ph_rw = self.data[1, 1:3]
        #Ph_ls = self.data[2, 1:3]
        #Ph_lw = self.data[3, 1:3]

        P1 = np.true_divide((self.data[0, 1:3] + self.baxter[2,0:2]), 2)
        P2 = np.true_divide((self.data[2, 1:3] + self.baxter[0,0:2]), 2)
        P3 = np.true_divide((self.data[1, 1:3] + self.baxter[3,0:2]), 2)
        P4 = np.true_divide((self.data[3, 1:3] + self.baxter[1,0:2]), 2)

        self.otp_s = np.true_divide((P1 + P2 + P3 + P4), 4)

	self.e_old = np.abs(og.P_old - og.otp_s)
	self.e_new = np.abs(og.P_new - og.otp_s)

    def dynamic(self):
	
	if (self.e_new - self.e_old) < 0.02:

		dt = self.D[1, 4] - self.D[2, 4]

		self.w = self.w - ((self.w*dt)/self.t_goal)

		self.P_goal = (self.w*self.otp_s) + ((1-self.w)*self.P_obj)

		V = (self.P_new - self.P_old)/dt

		self.t_goal = (self.P_goal - self.P_obj)/V

		#return self.t_goal, self.P_goal

		self.goal_pub.publish(self.P_goal)
		self.time_pub.publish(self.t_goal)


def main(args):
    og = OtpGenerator()
    rospy.init_node('OtpGenerator', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print ("Shutting down")

if __name__ == '__main__':
    main(sys.argv)
