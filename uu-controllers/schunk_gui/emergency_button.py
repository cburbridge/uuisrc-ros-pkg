#!/usr/bin/env python

try:
	import os
	import sys
	import gtk
  	import gtk.glade
 	import pygtk
  	pygtk.require("2.0")
	import roslib; roslib.load_manifest('schunk_gui')
	import rospy
	from std_msgs.msg import *
except:
	print "One (or more) of the dependencies is not satisfied"
	sys.exit(1)

	
class EmergencyStopButton:
	def __init__(self):
		self.wTree = gtk.glade.XML("emergency_button.glade", "window1")		
		dic = { "on_togglebutton1_toggled" : self.emergency_stop, "on_window1_destroy" : self.shutdown }
		self.wTree.signal_autoconnect(dic)

	def shutdown(self, widget):
		gtk.main_quit()
		rospy.signal_shutdown("Bye!")

	def emergency_stop(self, widget):
		if widget.get_active():
			# STOP
			rospy.Publisher("/emergency", Bool, latch=True).publish(True)
			self.wTree.get_widget("image1").set_from_file("go.png")
		else:
			# GO
			rospy.Publisher("/emergency", Bool, latch=True).publish(False)
			rospy.Publisher("/ackAll", Bool, latch=True).publish(True)
			self.wTree.get_widget("image1").set_from_file("stop.png")	

if __name__ == "__main__":
	try:
		wd = os.environ.get("UUISRC_ROS_PKG_PATH") + "/uu-controllers/schunk_gui/"
	except:
		print "Please set UUISRC_ROS_PKG_PATH environement variable to where your uuisrc-ros-pkg repository is"
		sys.exit(1)
	try:
		os.chdir(wd)
	except:
		print "Working directory does not exist. Check for mispellings"
		sys.exit(1)
	rospy.init_node('schunk_emergency_button')
	gtk.gdk.threads_init()
	emergency = EmergencyStopButton()
	gtk.main()
	rospy.spin()
