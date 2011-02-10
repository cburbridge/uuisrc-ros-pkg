#!/usr/bin/env python

import sys
try:
 	import pygtk
  	pygtk.require("2.0")
except:
  	pass
try:
	import gtk
  	import gtk.glade
except:
	sys.exit(1)

try:
	import roslib; roslib.load_manifest('schunk_gui')
	import rospy
	from std_msgs.msg import *
except:
	pass

	
class EmergencyStopButton:
	def __init__(self):
		self.wTree = gtk.glade.XML("emergency_button.glade", "window1")		
		dic = { "on_togglebutton1_toggled" : self.emergency_stop, "on_window1_destroy" : gtk.main_quit }
		self.wTree.signal_autoconnect(dic)


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
	rospy.init_node('schunk_emergency_button')
	emergency = EmergencyStopButton()
	gtk.main()
