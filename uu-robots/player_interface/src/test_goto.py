#!/usr/bin/env python
import roslib; roslib.load_manifest('player_interface')
import rospy
import sys
import math
from player_interface.srv import *
from geometry_msgs.msg import PoseStamped

def gotoPose(x, y, th):
    rospy.wait_for_service('GoTo')
    try:
        goto_srv = rospy.ServiceProxy('GoTo', GoTo)
        resp1 = goto_srv(x, y, th)
        print "Requesting %f, %f, %f"%(x, y, th)
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e



#def usage():
#    return "%s [x y th]"%sys.argv[0]

def goalCallback(data):
    #math.acos(data.pose.orientation.w)*2
    quaternion=data.pose.orientation
    num = 2.*(quaternion.x*quaternion.y + quaternion.z*quaternion.w)
    den = 1. - 2.*(quaternion.y**2 + quaternion.z**2)
    th = math.atan2(num,den)
    gotoPose(data.pose.position.x, data.pose.position.y, th ) #data.pose.orientation.w)
    


#if __name__ == "__main__":
#    if len(sys.argv) == 4:
#        x = float(sys.argv[1])
#        y = float(sys.argv[2])
#        th = float(sys.argv[3])
#    else:
#        print usage()
#        sys.exit(1)
#    print "Requesting %f, %f, %f"%(x, y, th)
#    #print "%s + %s = %s"%(x, y, add_two_ints_client(x, y))



if __name__ == '__main__':
    rospy.init_node('test_goto', anonymous=True)
    rospy.Subscriber("goalpose", PoseStamped, goalCallback)
    rospy.spin()

