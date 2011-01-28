#!/usr/bin/env python
import sys
#print sys.path
import roslib; roslib.load_manifest('schunk_interface')
#print ""
#print ""
#print sys.path
#sys.exit(0)
from sensor_msgs.msg import JointState
import rospy

commandVelocity = JointState()
state=JointState()
state.name=['Joint0','Joint1','Joint2','Joint3','Joint4','Joint5','Joint6']
state.position=[0, 0,0,0,0,0,0]
commandVelocity.name=['Joint0','Joint1','Joint2','Joint3','Joint4','Joint5','Joint6']
commandVelocity.velocity=[0, 0,0,0,0,0,0]
controllingActive=False

def callbackJointStates(data):
    #rospy.loginfo("JointState command received")
    global commandVelocity
    commandVelocity = data
    
def callbackJointStatesPos(data):
    #rospy.loginfo("JointState command received")
    lookup={}
    for i in range(0,7):
	lookup[data.name[i]]=i
    #print lookup
    global state
    for i in range(0,7):
	state.position[i] = data.position[lookup[state.name[i]]]
	
    

def simulation():
    rospy.init_node('SchunkVelocitySim')
    rospy.Subscriber("/schunk/vel_sim/joint_states", JointState, callbackJointStates)
    rospy.Subscriber("/schunk/pos_sim/joint_states", JointState, callbackJointStatesPos)
    
    pub = rospy.Publisher('/schunk/simulated/joint_states', JointState)
    rospy.loginfo("Simulating - listening on \"/schunk/vel_sim/joint_states\" and publishing on \"/schunk/simulated/joint_states\"")
    
    # !! HARD CODED JOINT LIMITS AND JOINT NUMBERS !!
    # this is not good as it should be read from the URDF not hard coded - i'm lazy
    global state
    #
    limits=[[-3.14,3.14] , [-2.006,2.006] , [-2.8,2.8] , [-2.007,2.007] , [-2.8,2.8] , [-1.9,1.9] , [-3.0,3.0] ]
    
    freq = rospy.Rate(100)
    lookup = {}
    while not rospy.is_shutdown():
	#    
	#print commandVelocity
	for i in range(0,7):
	    lookup[commandVelocity.name[i]]=i
	for i in range(0,7):
	    state.position[i] = state.position[i] + 0.01*commandVelocity.velocity[lookup[state.name[i]]]
	    if state.position[i] < limits[i][0]:
		state.position[i]=limits[i][0]
	    if state.position[i] > limits[i][1]:
		state.position[i]=limits[i][1]
	state.header.stamp=rospy.Time.now()
	      
	pub.publish(state)
	freq.sleep()
    
    
    
    

if __name__ == "__main__":
    simulation()


    
    
    

