#! /usr/bin/python

import roslib
roslib.load_manifest("potential_fields")
import rospy
import std_msgs
import nav_msgs
from nav_msgs.msg import Odometry
import robot_controller
import math
import tf

def potfield(robot, target_x, target_y):
    
    opt_dist = 2.5
    dist_thr = 0.7
    #spacing = 0.1
    
    dx = robot.x - target_x
    dy = robot.y - target_y
    d = math.sqrt( dx*dx + dy*dy )
    print "distance: ", d
    
    #tangential
    zu_circle = 0
    zv_circle = 0
    k_circle = 1.0
    if math.fabs(d - opt_dist ) < dist_thr:
	print "tangential"
	slope = math.atan2( -(dx), (dy) )
	zu_circle = -math.cos(slope)	
	zv_circle = -math.sin(slope)

    
    #attractive
    zu_attr = 0
    zv_attr = 0
    k_attr = 1.0
    if d > opt_dist:
	print "attractive"
	zu_attr = - (dx)
	zv_attr = - (dy)
    
    #repulsive
    zu_rep = 0
    zv_rep = 0
    k_rep = 1.
    d0 = opt_dist
    
    if d < d0 and d != 0:
	print "repulsive"
	zu_rep = (1./d - 1/d0) * 1./(d**2) * (dx)/d
	zv_rep = (1./d - 1/d0) * 1./(d**2) * (dy)/d
    
    zu = k_circle * zu_circle + k_attr * zu_attr + zu_rep * zu_rep
    zv = k_circle * zv_circle + k_attr * zv_attr + zv_rep * zv_rep

    #robot_u = zu*math.cos(robot.th) - zv*math.sin(robot.th)
    #robot_v = zu*math.sin(robot.th) + zv*math.cos(robot.th)
    
    robot_u = zu
    robot_v = zv

   
    print "ZU: %f ZV: %f"%(zu,zv)

    v = math.sqrt(robot_u*robot_u + robot_v*robot_v)
    if v < 0.1:
	v = 0.1
    elif v > 0.2:
	v = 0.2
    
    if (zu != 0):
    	r = math.atan2(robot_v,robot_u)
    else:
	r = 0; 
	print "AAAAAAAAAAHHHHHHH"


    r = r - robot.th
    if r < - math.pi:
	r = math.pi + math.pi + r;
    elif r > math.pi:
	r = -math.pi - math.pi + r;
    
    return v, r
    

if __name__ == "__main__":
    rospy.init_node("circle_around")
    
    robot = robot_controller.Robot()
    rospy.Subscriber("/odom",  Odometry,  robot.odometry_callback)
    
    target_x = 3.0
    target_y = 0
    
    rospy.sleep(1.0)
    
    initialtheta = robot.th
    initialx = robot.x
    initialy = robot.y
    #initialx = 0
    #initialy = 0
    
    rospy.loginfo("Initial pos: %f %f %f", initialx, initialy, initialtheta)
    
    goalx = target_x*math.cos(initialtheta) - target_y*math.sin(initialtheta) + initialx
    goaly = target_x*math.sin(initialtheta) + target_y*math.cos(initialtheta) + initialy
    
    rospy.loginfo("Target at: %f %f", goalx, goaly)
    dx = initialx - goalx
    dy = initialy - goaly
    d = math.sqrt( dx*dx + dy*dy )
    rospy.loginfo("Distance: %f", d)
    
    loop = rospy.Rate(10)
    
    listener = tf.TransformListener()
    while not rospy.is_shutdown():
	now = rospy.Time.now()
	try:
	    listener.waitForTransform("/odom","/torso", now, rospy.Duration(4.0))
	    (trans,rot) = listener.lookupTransform("/odom", "/torso", now)
	except (tf.LookupException, tf.ConnectivityException, tf.Exception):
	    rospy.loginfo("Error no transorm!!")
            continue

	goalx = trans[0]
	goaly = trans[1]
	
	print "Robot: ", (robot.x, robot.y), " goal: ", (goalx, goaly)
	
	
	#print "beep"
	v,w = potfield(robot, goalx, goaly)
	w = 0.3 * w #- initialtheta
	rospy.loginfo("Robot pos: %.2f %.2f %.2f, applying speed of: %.2f %.2f", robot.x, robot.y, robot.th, v, 180*w/math.pi)
	robot.set_vel(v,w)
	loop.sleep()
	
    
