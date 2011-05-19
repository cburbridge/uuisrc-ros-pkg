#!/usr/bin/env python

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

from pr2_control_utilities import pr2_joint_mover


class C_PR2ControlCentre:
    def __init__(self, actives=None):
        
        self.robot_state = pr2_joint_mover.RobotState()
        self.mover = pr2_joint_mover.PR2JointMover(self.robot_state)
        
        self.currentJointStates = JointState()
#        argc = len(sys.argv)
#        argv = sys.argv
#        self.pr2PoseLoader = pr2_pose_loader()
#        self.sectorNames = C_SectorNames(type)
        
        # get number of modules
#        self.numModules = self.sectorNames.fieldNumber
        
        # load gui
        self.wTree = gtk.Builder()
        self.wTree.add_from_file("pr2_control_gui.glade")

        # check which modules are active
        if actives == None:
            self.wTree.get_object("check_l_arm").set_active(True)
            self.wTree.get_object("check_r_arm").set_active(True)
            self.wTree.get_object("check_l_gripper").set_active(True)
            self.wTree.get_object("check_r_gripper").set_active(True)
            self.wTree.get_object("check_r_gripper").set_active(True)
            self.wTree.get_object("check_head").set_active(True)
            self.wTree.get_object("check_torso").set_active(False)
               
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
                    "on_loadStack_clicked":self.on_loadStack_clicked,
                    "on_exe_clicked":self.on_exe_clicked,
                    "on_clear_stack_clicked":self.on_clear_stack_clicked,
                    "on_stackUp_clicked":self.on_stackUp_clicked,
                    "on_stackDown_clicked":self.on_stackDown_clicked,
                    "on_update_clicked":self.on_update_clicked }
        self.wTree.connect_signals(bindings)
        
        # setup spinbuttons
        for name in self.robot_state.all:
            widget = self.wTree.get_object("c_"+name)
            widget.set_digits(10)
            widget.set_increments(1, 5)
            if self.inDegrees:
                widget.set_range(-180.0, 180.0)
            else:
                widget.set_range(-1000*pi, 1000*pi)
        self.wTree.get_object("time_to_action_completion").set_range(0, 60)
        self.wTree.get_object("time_to_action_completion").set_digits(2)
        self.wTree.get_object("time_to_action_completion").set_increments(0.5, 1)
        self.wTree.get_object("time_to_action_completion").set_value(1.0)
        
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
        for name in self.robot_state.all:
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
            for name in self.robot_state.all:
                # control widget
                widget = self.wTree.get_object("c_"+name)
                value = widget.get_value() * 180 / pi
                widget.set_value(value)
                # display widget is handled by update_fields
        else:
            for name in self.robot_state.all:
                widget = self.wTree.get_object("c_"+name)
                value = widget.get_value() * pi / 180
                widget.set_value(value)
        pass


    def on_move_clicked(self, widget):
        # move left arm
        self.mover.time_to_reach = float(self.wTree.get_object("time_to_action_completion").get_value())
        if self.checks['l_arm']:
            jvals = []
            for name in self.robot_state.left_joint_names:
                value = float(self.wTree.get_object("c_"+name).get_value())
                if self.inDegrees:
                    value = value * pi / 180
                jvals.append(value)
            self.mover.set_arm_state(jvals, 'l_arm')

        # move right arm
        if self.checks['r_arm']:
            jvals = []
            for name in self.robot_state.right_joint_names:
                value = float(self.wTree.get_object("c_"+name).get_value())
                if self.inDegrees:
                    value = value * pi / 180
                jvals.append(value)
            self.mover.set_arm_state(jvals, 'r_arm')


        # move left gripper
        if self.checks['l_gripper']:
            for name in self.robot_state.l_gripper_names:
                value = float(self.wTree.get_object("c_"+name).get_value())
                if self.inDegrees:
                    value = value * pi / 180
                jval = value 
            self.mover.set_gripper_state(jval, 'l_gripper')
            
        # move right gripper
        if self.checks['r_gripper']:
            for name in self.robot_state.r_gripper_names:
                value = float(self.wTree.get_object("c_"+name).get_value())
                if self.inDegrees:
                    value = value * pi / 180
                jval = value 
            self.mover.set_gripper_state(jval, 'r_gripper')
            
        # move head
        if self.checks['head']:
            jvals = []
            for name in self.robot_state.head_joint_names:
                value = float(self.wTree.get_object("c_"+name).get_value())
                if self.inDegrees:
                    value = value * pi / 180
                jvals.append(value)
            self.mover.set_head_state(jvals)
        pass


    def on_copy_clicked(self, widget):
        for name in self.robot_state.all:
            value = float(self.wTree.get_object("d_"+name).get_text())
            widget = self.wTree.get_object("c_"+name)
            widget.set_value(value)
        pass
    
    
    def on_add_clicked(self, widget):
        if not self.wTree.get_object("add_intention").get_active():
            self.on_copy_clicked(widget)
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
        teatime = float(self.wTree.get_object("time_to_action_completion").get_value())
        jstate = self.currentJointStates
        jstate.position = list(jstate.position) # so i can edit as it is a tuple
        for name in self.robot_state.all:
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
        
        if not self.wTree.get_object("add_intention").get_active():
            self.mover.store_targets()
        else:
            self.mover.store_targets(jstate)
        self.mover.time_to_reach = teatime
        self.stackDict[label] = (jstate, self.mover)
        
        self.mover = pr2_joint_mover.PR2JointMover(self.robot_state)
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
        pass


    def on_stackBox_changed(self, widget):
        label = self.wTree.get_object("stackBox").get_active_text()
        if label == None:
            pass
        else:
            (jstate, self.mover) = self.stackDict[label]
            for name in self.robot_state.all:
                i = jstate.name.index(name)
                value = float(jstate.position[i])
                if self.inDegrees:
                    value *= 180 / pi
                self.wTree.get_object("c_"+name).set_value(value)
            self.wTree.get_object("time_to_action_completion").set_value(float(self.mover.time_to_reach))
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
            fp = open(filename, 'w')        
            for treerow in self.stackList:
                label = treerow[0]
                (jstate,mover) = self.stackDict[label]
                mover.write_targets(fp)
                msg = "label:%s\n\n" % label                
                fp.write(msg)
                

            fp.close()
        elif response == gtk.RESPONSE_CANCEL:
            pass
        dialog.destroy()        

        pass

    def mover_to_jointstate(self, mover):
        jstate = JointState()
        
        jstate.position = (mover.target_head + 
                           mover.target_left_arm +
                           mover.target_right_arm + 
                           mover.target_left_gripper +
                           mover.target_right_gripper
                           )
                          
        jstate.name = [i for i in self.robot_state.head_joint_names +
                       self.robot_state.left_joint_names + 
                       self.robot_state.right_joint_names +
                       self.robot_state.l_gripper_names + 
                       self.robot_state.r_gripper_names]
        return jstate


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
            fp = open(filename, "r")
            num_read = 0
            try:                                
                while True:
                    newmover = pr2_joint_mover.PR2JointMover(self.robot_state)
                    go_on = newmover.parse_bookmark_file(fp)
                    if not go_on:
                        rospy.loginfo("%d entries read"%num_read)
                        break
                    jstate = self.mover_to_jointstate(newmover)
                    newmover.name = newmover.name.strip("\n")
                    label = newmover.name
                    self.stackList.insert(len(self.stackList), [label])
                    self.stackDict[label] = (jstate,newmover)
                    num_read += 1                    
            except:
                dialog.destroy()
                raise
        elif response == gtk.RESPONSE_CANCEL:
            pass
        dialog.destroy()                
        pass

    def on_exe_clicked(self, widget):
        for treerow in self.stackList:
            label = treerow[0]
            _, mover = self.stackDict[label]
            mover.execute_and_wait()
        pass

    def on_clear_stack_clicked(self, widget):
        self.stackDict = {}
        self.clear_stack()
        pass

    def on_stackUp_clicked(self, widget):
        label = self.wTree.get_object("stackBox").get_active_text()
        boxIndex = self.wTree.get_object("stackBox").get_active()
        if boxIndex < 0:
            rospy.logerr("can't move thin air")
        elif boxIndex == 0:
            rospy.logerr("and try to move your head above your head")
        else:  
            self.stackList.insert(boxIndex-1, [label])
            treestore = self.stackList
            treeiter = treestore.iter_nth_child(None, boxIndex+1)
            self.stackList.remove(treeiter)
            self.wTree.get_object("stackBox").set_active(boxIndex-1)
        pass
    
    def on_stackDown_clicked(self, widget):
        label = self.wTree.get_object("stackBox").get_active_text()
        boxIndex = self.wTree.get_object("stackBox").get_active()
        if boxIndex < 0:
            rospy.logerr("can't move thin air")
        elif boxIndex == len(self.stackList)-1:
            rospy.logerr("and try to move your foot below your foot, same foot")
        else:  
            self.stackList.insert(boxIndex+2, [label])
            treestore = self.stackList
            treeiter = treestore.iter_nth_child(None, boxIndex)
            self.stackList.remove(treeiter)
            self.wTree.get_object("stackBox").set_active(boxIndex+1)
        pass

    def on_update_clicked(self, widget):
        if not self.wTree.get_object("add_intention").get_active():
            self.on_copy_clicked(widget)
        label = self.wTree.get_object("stackBox").get_active_text()
        boxIndex = self.wTree.get_object("stackBox").get_active()
        if boxIndex < 0:
            rospy.logerr("update what?")
            return      
        teatime = float(self.wTree.get_object("time_to_action_completion").get_value())
        jstate = self.currentJointStates
        jstate.position = list(jstate.position) # so i can edit as it is a tuple
        for name in self.robot_state.all:
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
        
        if not self.wTree.get_object("add_intention").get_active():
            self.mover.store_targets()
        else:
            self.mover.store_targets(jstate)
        self.mover.time_to_reach = teatime
        self.stackDict[label] = (jstate, self.mover)
        self.mover = pr2_joint_mover.PR2JointMover(self.robot_state)            
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
    
    def clear_stack(self):
        treestore = self.stackList
        for i in range(len(self.stackList)):
            treestore = self.stackList
            treeiter = treestore.iter_nth_child(None, 0)
            self.stackList.remove(treeiter)
        pass

# end of class
#####################


if __name__ == "__main__":
    wd = os.path.dirname(sys.argv[0])
    try:
        os.chdir(wd)
    except:
        rospy.logerr("Working directory does not exist. Check for mispellings")
        rospy.signal_shutdown()
        sys.exit(1)
           
    gtk.gdk.threads_init()
    rospy.init_node('pr2_gui', anonymous=True)
    gui = C_PR2ControlCentre()
    gobject.timeout_add(100, gui.update_fields)
    gtk.main()
    rospy.spin()
    pass
