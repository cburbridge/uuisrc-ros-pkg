import rospy
import nav_msgs
import math
from geometry_msgs.msg import Twist

class Robot:
    def __init__(self):
        self.x = 0
        self.y = 0
        self.th = 0
        
        self.pub = rospy.Publisher("cmd_vel_dwc", Twist)
        
        
    def odometry_callback(self,  odometry):
        quaternion = odometry.pose.pose.orientation
        
        q0 = quaternion.x
        q1 = quaternion.y
        q2 = quaternion.z
        q3 = quaternion.w
        
        self.th = math.atan2(2.*(q0*q1 + q2*q3), 1. - 2.*(q1**2 + q2**2))
        #self.th = 2*math.sin(q3/q2)
        self.x = odometry.pose.pose.position.x
        self.y = odometry.pose.pose.position.y
        
        #print (self.x, self.y, self.th)
    
    def set_vel(self,  v,  w):
        msg = Twist()
        msg.linear.x = v
        msg.angular.z = w        
        self.pub.publish(msg)
