#!/usr/bin/env python

try:
    import os
    import sys
    import gtk
    import pygtk
    pygtk.require("2.0")
    import gobject
#    import csv
    import roslib; roslib.load_manifest('pr2_control_gui')
    import rospy
    from sensor_msgs.msg import JointState

#    import math
    from math import pi
#    from math import degrees
#    from threading import Thread
#    import tf
    from string import strip

    from geometry_msgs.msg import Twist, Pose, PoseStamped, TwistStamped
    from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
    from pr2_controllers_msgs.msg import *

except:
    print "One (or more) of the dependencies is not satisfied"
    sys.exit(1)    


verbose = False
duration = 3.0

class pr2_pose_loader:
    def __init__(self, verbose = True):
        """ @package pr2_pose_saver : load
        init: defines goal state variables and opens publishers and clients
        """
        self.verbose = verbose

        ############### PRIVATE PUBLISHERS/CLIENTS ###############
        ## publisher for base commands
        self._base_pub = rospy.Publisher('base_controller/command', Twist)

        ## action client for grippers
        self._l_gripper_pub = rospy.Publisher('l_gripper_controller/command', Pr2GripperCommand)
        self._r_gripper_pub = rospy.Publisher('r_gripper_controller/command', Pr2GripperCommand)

        ## action client for torso
        self._torso_pub = rospy.Publisher('/torso_controller/command', JointTrajectory)

        ## action client for head
        self._head_pub = rospy.Publisher('/head_traj_controller/command', JointTrajectory)

        ## publisher for arms
        self._r_arm_pub = rospy.Publisher("r_arm_controller/command", JointTrajectory, latch=True)
        self._l_arm_pub = rospy.Publisher("l_arm_controller/command", JointTrajectory, latch=True)

        ## publishers dictionary
        self.publishers = {'base':self._base_pub,
                           'l_gripper':self._l_gripper_pub,
                           'r_gripper':self._r_gripper_pub,
                           'torso':self._torso_pub,
                           'head':self._head_pub, 
                           'r_arm':self._r_arm_pub,
                           'l_arm':self._l_arm_pub,
                           'r_gripper':self._r_gripper_pub,
                           'l_gripper':self._l_gripper_pub
                           }
        rospy.sleep(0.5)

    def set_arm_state(self, jvals, arm):
        """ Sets goal for indicated arm (r_arm/l_arm) using provided joint values"""
        # Build trajectory message
        command = JointTrajectory()
        command.joint_names = ['%s_shoulder_pan_joint' % arm[0], 
                               '%s_shoulder_lift_joint' % arm[0],
                               '%s_upper_arm_roll_joint' % arm[0],
                               '%s_elbow_flex_joint' % arm[0],
                               '%s_forearm_roll_joint' % arm[0],
                               '%s_wrist_flex_joint' % arm[0],
                               '%s_wrist_roll_joint' % arm[0]]
        command.points.append(JointTrajectoryPoint(
            positions=jvals,
            velocities = [0.0] * (len(command.joint_names)),
            accelerations = [],
            time_from_start =  rospy.Duration(duration)))
        command.header.stamp = rospy.get_rostime() + rospy.Duration(0.01)
        # Send
        try:
            self.publishers[arm].publish(command)
            print command
            print ">>> success"
            if self.verbose:
                print "published [%s] to %s_controller/command topic" % (jvals, arm)
        except:
            print "failed to publish arm positions!"

    def set_gripper_state(self, jval, gripper):
        """ Sets goal for indicated gripper (r_gripper/l_gripper) using provided joint angle"""
        # Build trajectory message
        goal = Pr2GripperCommand()
        goal.max_effort = -1
        goal.position = jval
        try:
            self.publishers[gripper].publish(goal)
            if self.verbose:
                print "published [%s] to %s_controller/command topic" % (jval, gripper)
        except:
            print "failed to publish gripper positions!"

    def set_head_state(self, jvals):
        """ Sets goal for head using provided joint values"""
        print ">>> moving head"
        # Build trajectory message
        head_goal = JointTrajectory()
        head_goal.joint_names.append('head_pan_joint')
        head_goal.joint_names.append('head_tilt_joint')
        head_goal.points.append(JointTrajectoryPoint())
        head_goal.points[0].time_from_start = rospy.Duration(0.25)
        #self.head_goal.header.frame_id = 'base_link'
        for i in range(len(head_goal.joint_names)):
            head_goal.points[0].positions.append(jvals[i])
            head_goal.points[0].velocities.append(1)
        head_goal.header.stamp = rospy.get_rostime() + rospy.Duration(0.01)
        try:
#            print head_goal
            self.publishers["head"].publish(head_goal)
            if self.verbose:
                print "published [%s] to head_traj_controller/command topic" % jvals
        except:
            print "failed to publish head position!"

    def set_torso_state(self, jval):
        """ Sets goal for torso using provided value"""
        # Build trajectory message
        torso_goal = JointTrajectory()
        torso_goal.joint_names.append('torso_lift_joint')
        torso_goal.points.append(JointTrajectoryPoint())
        torso_goal.points[0].time_from_start = rospy.Duration(0.25)
        torso_goal.points[0].velocities.append(0)
        torso_goal.points[0].positions.append(jval)
        torso_goal.header.stamp = rospy.get_rostime() + rospy.Duration(0.01)
        try:
            #print head_goal
            self.publishers["torso"].publish(torso_goal)
            if self.verbose:
                print "published [%s] to torso_controller/command topic" % jval
        except:
            print "failed to publish torso position!"

    def parse_bookmark_file(self, bfile):
        f=open(bfile,'r')

        for l in f.readlines():
            if l.find("arm") != -1:
                jvals_str = l.split(":")[1]
                jvals = map(lambda x: float(x),jvals_str.strip("\n").split(","))
                arm = l.split(":")[0]
                self.set_arm_state(jvals, arm)
            if l.find("gripper") != -1:
                jval_str = l.split(":")[1]
                jval = map(lambda x: float(x),jval_str.strip("\n").split(","))[0]
                gripper = l.split(":")[0]
                self.set_gripper_state(jval, gripper)
            if l.find("head") != -1:
                jvals_str = l.split(":")[1]
                jvals = map(lambda x: float(x),jvals_str.strip("\n").split(","))
                self.set_head_state(jvals)
            if l.find("torso") != -1:
                jvals_str = l.split(":")[1]
                jval = map(lambda x: float(x),jvals_str.strip("\n").split(","))[0]
                self.set_torso_state(jval)

        f.close()




class C_PR2JointNames:
    def __init__(self):
        self.modules = ('l_arm',
                        'r_arm',
                        'l_gripper',
                        'r_gripper',
                        'head',
                        'torso' )

        self.l_arm = ('l_shoulder_pan_joint',
                      'l_shoulder_lift_joint',
                      'l_upper_arm_roll_joint',
                      'l_elbow_flex_joint',
                      'l_forearm_roll_joint',
                      'l_wrist_flex_joint',
                      'l_wrist_roll_joint')
        
        self.r_arm = ('r_shoulder_pan_joint',
                      'r_shoulder_lift_joint',
                      'r_upper_arm_roll_joint',
                      'r_elbow_flex_joint',
                      'r_forearm_roll_joint',
                      'r_wrist_flex_joint',
                      'r_wrist_roll_joint')
        
        self.l_gripper = ('l_gripper_joint',)
        
        self.r_gripper = ('r_gripper_joint',)
        
        self.head = ('head_pan_joint',
                     'head_tilt_joint')
        
        self.all = self.l_arm + self.r_arm + self.head + self.l_gripper + self.r_gripper



class C_PR2ControlCentre:
    def __init__(self, actives=None):
        self.jointNames = C_PR2JointNames()
        self.currentJointStates = JointState()
        self.mover = pr2_pose_loader()
#        argc = len(sys.argv)
#        argv = sys.argv
#        self.pr2PoseLoader = pr2_pose_loader()
#        self.sectorNames = C_SectorNames(type)
        
        # get number of modules
#        self.numModules = self.sectorNames.fieldNumber
        
        # load gui
        self.wTree = gtk.Builder()
        self.wTree.add_from_file("pr2_control_gui.glade")
        
        # keep track of checks for fast access
        self.checks = {"l_arm":self.wTree.get_object("check_l_arm").get_active(),
               "r_arm":self.wTree.get_object("check_r_arm").get_active(),
               "l_gripper":self.wTree.get_object("check_l_gripper").get_active(),
               "r_gripper":self.wTree.get_object("check_r_gripper").get_active(),
               "head":self.wTree.get_object("check_head").get_active(),
               "torso":self.wTree.get_object("check_torso").get_active() }
        
        # set inDegrees
        self.wTree.get_object("inDegrees").set_active(False) # xxx
        self.inDegrees = self.wTree.get_object("inDegrees").get_active()
      
        # check which modules are active
        if actives == None:
            self.wTree.get_object("check_l_arm").set_active(False)
            self.wTree.get_object("check_r_arm").set_active(False)
            self.wTree.get_object("check_l_gripper").set_active(False)
            self.wTree.get_object("check_r_gripper").set_active(False)
            self.wTree.get_object("check_r_gripper").set_active(False)
            self.wTree.get_object("check_head").set_active(False)
            self.wTree.get_object("check_torso").set_active(False)

        # setup stack list
        self.stackList = self.wTree.get_object("stackList")
        comboboxStack = self.wTree.get_object("stackBox")
        cell = gtk.CellRendererText()
        comboboxStack.pack_start(cell, True)
        comboboxStack.add_attribute(cell, "text", 0)
        self.stackDict = {}
            
        # bindings
        bindings = {"on_window1_destroy":self.shutdown,
                    "on_check_l_arm_toggled":self.on_check_l_arm_toggled,
                    "on_check_r_arm_toggled":self.on_check_r_arm_toggled,
                    "on_check_l_gripper_toggled":self.on_check_l_gripper_toggled,
                    "on_check_r_gripper_toggled":self.on_check_r_gripper_toggled,
                    "on_check_head_toggled":self.on_check_head_toggled,
                    "on_check_torso_toggled":self.on_check_torso_toggled,
                    "on_inDegrees_toggled":self.on_inDegrees_toggled,
                    "on_copy_clicked":self.on_copy_clicked,
                    "on_move_clicked":self.on_move_clicked,
                    "on_add_clicked":self.on_add_clicked,
                    "on_addCancel_clicked":self.on_addCancel_clicked,
                    "on_addOK_clicked":self.on_addOK_clicked,
                    "on_delete_clicked":self.on_delete_clicked,
                    "on_stackBox_changed":self.on_stackBox_changed,
                    "on_saveStack_clicked":self.on_saveStack_clicked,
                    "on_loadStack_clicked":self.on_loadStack_clicked }
        self.wTree.connect_signals(bindings)
        
        # setup spinbuttons
        for name in self.jointNames.all:
            widget = self.wTree.get_object("c_"+name)
            widget.set_digits(10)
            widget.set_increments(1, 5)
            if self.inDegrees:
                widget.set_range(-180.0, 180.0)
            else:
                widget.set_range(-pi, pi)
        
        # Subscribers
        self.currentJointStates = JointState()
        self.jointStatesSub = rospy.Subscriber("/joint_states", JointState, self.joint_states_update)

# end of constructor
#******************************************

    def shutdown(self, widget):
        # kill gtk thread
        gtk.main_quit()
        # kill ros thread
        rospy.signal_shutdown("Because I said so!")
        pass


    def joint_states_update(self, data):
        self.currentJointStates = data
        pass

    def update_fields(self, *args):
        for name in self.jointNames.all:
#        for name in self.jointNames.l_arm:
            # get value
            try:
                i = self.currentJointStates.name.index(name)
            except ValueError:
                i = -1
                rospy.logerr("this shouldn't have happened")
                return
            value = float(self.currentJointStates.position[i])
            if self.inDegrees:
                value *= 180 / pi
            # update corresponding widget  
            self.wTree.get_object("d_"+str(name)).set_text("%.10f" % value)
        return True
        pass


    def update_checks_list(self, name, widget):
        self.checks[name] = widget.get_active()
        pass


    def on_check_l_arm_toggled(self, widget):
        self.update_checks_list("l_arm", widget)
        pass

    
    def on_check_r_arm_toggled(self, widget):
        self.update_checks_list("r_arm", widget)
        pass

    
    def on_check_l_gripper_toggled(self, widget):
        self.update_checks_list("l_gripper", widget)
        pass


    def on_check_r_gripper_toggled(self, widget):
        self.update_checks_list("r_gripper", widget)
        pass


    def on_check_head_toggled(self, widget):
        self.update_checks_list("head", widget)
        pass


    def on_check_torso_toggled(self, widget):
        self.update_checks_list("torso", widget)
        pass


    def on_inDegrees_toggled(self, widget):
        self.inDegrees = self.wTree.get_object("inDegrees").get_active()
        if self.inDegrees:
            for name in self.jointNames.all:
                # control widget
                widget = self.wTree.get_object("c_"+name)
                value = widget.get_value() * 180 / pi
                widget.set_value(value)
                # display widget is handled by update_fields
        else:
            for name in self.jointNames.all:
                widget = self.wTree.get_object("c_"+name)
                value = widget.get_value() * pi / 180
                widget.set_value(value)
        pass


    def on_move_clicked(self, widget):
        # move left arm
        if self.checks['l_arm']:
            jvals = []
            for name in self.jointNames.l_arm:
                value = float(self.wTree.get_object("c_"+name).get_value())
                if self.inDegrees:
                    value = value * pi / 180
                jvals.append(value)
            self.mover.set_arm_state(jvals, 'l_arm')

        # move right arm
        if self.checks['r_arm']:
            jvals = []
            for name in self.jointNames.r_arm:
                value = float(self.wTree.get_object("c_"+name).get_value())
                if self.inDegrees:
                    value = value * pi / 180
                jvals.append(value)
            self.mover.set_arm_state(jvals, 'r_arm')


        # move left gripper
        if self.checks['l_gripper']:
            for name in self.jointNames.l_gripper:
                value = float(self.wTree.get_object("c_"+name).get_value())
                if self.inDegrees:
                    value = value * pi / 180
                jval = value 
            self.mover.set_gripper_state(jval, 'l_gripper')
            
        # move right gripper
        if self.checks['r_gripper']:
            for name in self.jointNames.r_gripper:
                value = float(self.wTree.get_object("c_"+name).get_value())
                if self.inDegrees:
                    value = value * pi / 180
                jval = value 
            self.mover.set_gripper_state(jval, 'r_gripper')
            
        # move head
        if self.checks['head']:
            jvals = []
            for name in self.jointNames.head:
                value = float(self.wTree.get_object("c_"+name).get_value())
                if self.inDegrees:
                    value = value * pi / 180
                jvals.append(value)
#            print jvals
            self.mover.set_head_state(jvals)
        pass


    def on_copy_clicked(self, widget):
        for name in self.jointNames.all:
            value = float(self.wTree.get_object("d_"+name).get_text())
            widget = self.wTree.get_object("c_"+name)
            widget.set_value(value)
        pass
    
    
    def on_add_clicked(self, widget):
        name = self.find_unique_joint_state_name()
        dialogText = self.wTree.get_object("jointStateName") 
        dialogText.set_text(name)
#        self.wTree.get_object("labelDialogJointsAnglesErrorMessage").set_text("")
        if self.wTree.get_object("stackBox").get_active() >= 0:
#            self.wTree.get_object("addAfter").set_sensitive(True)
#            self.wTree.get_object("addBefore").set_sensitive(True)
#            name = self.wTree.get_object("stackBox").get_active_text()
#            index = self.dictJointsAngles[name]
#            text = str(self.listJointsAngles[index][0]) + ": " + str(self.listJointsAngles[index][1])
#            self.wTree.get_object("labelDialogAddJointsAnglesIndex").set_text(text)
            pass
        else:
            self.wTree.get_object("addAfter").set_sensitive(False)
            self.wTree.get_object("addBefore").set_sensitive(False)
#            self.wTree.get_object("labelDialogAddJointsAnglesIndex").set_text("")      
        self.wTree.get_object("dialog1").show()      
        pass


    def on_addCancel_clicked(self, widget):
        self.wTree.get_object("dialog1").hide()
        pass


    def on_addOK_clicked(self, widget):
        label = self.wTree.get_object("jointStateName").get_text()
        jstate = self.currentJointStates
        jstate.position = list(jstate.position) # so i can edit as it is a tuple
        for name in self.jointNames.all:
            try:
                i = self.currentJointStates.name.index(name)
            except ValueError:
                i = -1
                rospy.logerr("astral projection trance")
                return
            value = float(self.wTree.get_object("c_"+name).get_value())
            if self.inDegrees:
                value *= pi / 180
            jstate.position[i] = value
        
        self.stackDict[label] = jstate
        index = len(self.stackList) # goes to the end, need to do something about it later
        self.stackList.insert(index, [label])
        self.wTree.get_object("dialog1").hide()             
        pass


    def on_delete_clicked(self, widget):
        label = self.wTree.get_object("stackBox").get_active_text()
        boxIndex = self.wTree.get_object("stackBox").get_active()
        del self.stackDict[label]
        #self.stackList.remove(label)
        treestore = self.stackList
        treeiter = treestore.iter_nth_child(None, boxIndex)
        self.stackList.remove(treeiter)        
        print len(self.stackDict), len(self.stackList)
        pass


    def on_stackBox_changed(self, widget):
        label = self.wTree.get_object("stackBox").get_active_text()
        if label == None:
            pass
        else:
            jstate = self.stackDict[label]
            for name in self.jointNames.all:
                i = jstate.name.index(name)
                value = float(jstate.position[i])
                if self.inDegrees:
                    value *= 180 / pi
                self.wTree.get_object("c_"+name).set_value(value)
        pass


    def on_saveStack_clicked(self, widget):
        dialog = gtk.FileChooserDialog(title="Save stack of PR2 joint states (.stack)",
                                       action=gtk.FILE_CHOOSER_ACTION_SAVE,
                                       buttons=(gtk.STOCK_CANCEL, gtk.RESPONSE_CANCEL, 
                                                gtk.STOCK_SAVE, gtk.RESPONSE_OK))
        dialog.set_default_response(gtk.RESPONSE_OK)
        dialog.set_do_overwrite_confirmation(True)
        response = dialog.run()
        if response == gtk.RESPONSE_OK:
            filename = dialog.get_filename()
            try:
                fp = open(filename, 'w')        
                for treerow in self.stackList:
                    label = treerow[0]
                    jstate = self.stackDict[label]
                   
                    msg = "name:%s\n" % str(jstate.name)
                    fp.write(msg)
                    msg = "position:%s\n" % str(jstate.position)
                    fp.write(msg)
                    
                    msg = "label:%s\n" % label
                    fp.write(msg)
                fp.close()    
            except:
                print "oh"
        elif response == gtk.RESPONSE_CANCEL:
            pass
        dialog.destroy()        

        pass


    def on_loadStack_clicked(self, widget):

        dialog = gtk.FileChooserDialog(title="Load stack of PR2 joint states (.stack)", 
                                       action=gtk.FILE_CHOOSER_ACTION_OPEN, 
                                       buttons=(gtk.STOCK_CANCEL, gtk.RESPONSE_CANCEL,
                                                 gtk.STOCK_OPEN, gtk.RESPONSE_OK))
        dialog.set_default_response(gtk.RESPONSE_OK)
        dialog.select_filename("default.list")
        response = dialog.run()
        if response == gtk.RESPONSE_OK:
            filename = dialog.get_filename()
            try:
                fp = open(filename, 'r')
                buffer = fp.read()
                fp.close()
        
                lines = buffer.split("\n")
                jstate = JointState()
                for line in lines:
                    a = line.split(":")
                    if len(a) == 2:
                        tag = a[0]
                        value = a[1]
                        if tag == "label":
                            index = len(self.stackList)
                            self.stackList.insert(len(self.stackList), [value])
                            self.stackDict[value] = jstate
                            jstate = JointState()
        
                        elif tag == "name":
                            value = value.strip("[]")
                            foo = value.split(", ")
                            foobar = []
                            for bar in foo:
                                bar = bar.strip('"')
                                bar = bar.strip("'")
                                foobar.append(bar)
                            jstate.name = foobar
        
                        elif tag == "position":
                            value = value.strip("[]")
                            foo = value.split(", ")
                            foobar = []
                            for bar in foo:
                                bar = float(bar)
                                foobar.append(bar)                    
                            jstate.position = foobar
            except:
                print "load file failed"
        elif response == gtk.RESPONSE_CANCEL:
            pass
        dialog.destroy()
        pass


    # other
    def find_unique_joint_state_name(self):
        i = 0
        while True:
            name = "PR2_State_" + str(i)
            if name in self.stackDict:
                i += 1
            else:
                return name
        pass

# end of class
#####################


if __name__ == "__main__":
    wd = os.path.dirname(sys.argv[0])
    try:
        os.chdir(wd)
    except:
        print "Working directory does not exist. Check for mispellings"
        sys.exit(1)
           
    gtk.gdk.threads_init()
    rospy.init_node('pr2_gui', anonymous=True)
    gui = C_PR2ControlCentre()
    gobject.timeout_add(100, gui.update_fields)
    gtk.main()
    rospy.spin()
    pass
