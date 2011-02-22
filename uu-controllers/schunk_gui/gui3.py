#!/usr/bin/env python

try:
    import os
    import sys
    import gtk
#    import gtk.glade
    import pygtk
    pygtk.require("2.0")
    import gobject
    import csv
    import roslib; roslib.load_manifest('schunk_gui')
    import rospy
    from std_msgs.msg import *
    import xml.dom.minidom
    from sensor_msgs.msg import JointState
    from metralabs_ros.msg import SchunkStatus
    import math
    from math import pi
    from math import degrees
    from threading import Thread
    import tf

except:
    print "One (or more) of the dependencies is not satisfied"
    sys.exit(1)


RANGE = 10000
VELOCITY_CMD_TOPIC="/schunk/target_vc/joint_states"
POSITION_CMD_TOPIC="/schunk/target_pc/joint_states"
JOINT_STATE_TOPIC="/schunk/position/joint_states"
SCHUNK_STATUS_TOPIC="/schunk/status"

class RosCommunication():
    def __init__(self):
        description = rospy.get_param("schunk_description")
        robot = xml.dom.minidom.parseString(description).getElementsByTagName('robot')[0]
        self.free_joints = {}
        self.joint_lookup={}
        self.joint_list = [] # for maintaining the original order of the joints
        self.currentJointStates=JointState()
        self.currentSchunkStatus=SchunkStatus()
        self.dependent_joints = rospy.get_param("dependent_joints", {})
        
        # Find all non-fixed joints
        self.numModules = 0
        for child in robot.childNodes:
            if child.nodeType is child.TEXT_NODE:
                continue
            if child.localName == 'joint':
                jtype = child.getAttribute('type')
                if jtype == 'fixed':
                    continue
                name = child.getAttribute('name')
                if jtype == 'continuous':
                    minval = -pi
                    maxval = pi
                else:
                    limit = child.getElementsByTagName('limit')[0]
                    minval = float(limit.getAttribute('lower'))
                    maxval = float(limit.getAttribute('upper'))

                if name in self.dependent_joints:
                    continue
                if minval > 0 or maxval < 0:
                    zeroval = (maxval + minval)/2
                else:
                    zeroval = 0

                joint = {'min':minval, 'max':maxval, 'zero':zeroval, 'value':zeroval }
                self.free_joints[name] = joint
                self.joint_list.append(name)
                self.joint_lookup[name] = self.numModules
                self.numModules += 1

        # Setup all of the pubs and subs
        self.velocityPub = rospy.Publisher(VELOCITY_CMD_TOPIC, JointState)
        self.positionPub = rospy.Publisher(POSITION_CMD_TOPIC, JointState)
        self.jointSub = rospy.Subscriber(JOINT_STATE_TOPIC, JointState, self.jointStateUpdate)
        self.statusSub = rospy.Subscriber(SCHUNK_STATUS_TOPIC, SchunkStatus, self.schunkStatusUpdate)
        
        # Members that will be filled by the gui for commanding
        self.targetVelocity = JointState()
        self.targetPosition = JointState()
        self.setVelocity = False
        self.setPosition = False
        
        self.ackJoint = False
        self.ackNumber =0
        self.refJoint = False
        self.refNumber =0
        self.ackAll = False
        self.refAll = False
        self.maxCurrents = False
        self.emergencyStop = False

#        self.targetCurrent = JointState() # TODO: Set the current controls with the effort field (in SchunkRos also) 

        # A tf listener so that we can find the position of the end effector without service calls to
        # kinematics node
        self.tfListener = tf.TransformListener()

        
        
    def jointStateUpdate(self, data):
        #new joint states
        self.currentJointStates = data
        pass
    
    def schunkStatusUpdate(self, data):
        #schunk status
        self.currentSchunkStatus = data
        pass

    # The actual communication loop
    def loop(self):
        hz = 10 # 10hz
        r = rospy.Rate(hz) 
        
        while not rospy.is_shutdown():
            msg = JointState()
            self.targetPosition.header.stamp = rospy.Time.now()

            # Publish commands if wanted
            if self.setPosition:
                self.positionPub.publish(self.targetPosition)
                self.setPosition = False
            if self.setVelocity:
                self.velocityPub.publish(self.targetVelocity)
                self.setVelocity = False
            if self.ackJoint:
                print "/ack"
                rospy.Publisher("/ack", Int8, latch=True).publish(self.ackNumber)
                self.ackJoint = False
            if self.refJoint:
                self.refJoint = False
                print "/ref"
                rospy.Publisher("/ref", Int8, latch=True).publish(self.refNumber)
            if self.ackAll:
                print "/ackAll"
                rospy.Publisher("/ackAll", Bool, latch=True).publish(True)
                self.ackAll = False
            if self.refAll:
                print "/refAll"
                rospy.Publisher("/refAll", Bool, latch=True).publish(True)
                self.refAll = False
            if self.maxCurrents:
                print "/currentsmaxall"
                rospy.Publisher("/currentsMaxAll", Bool, latch=True).publish(True)
                self.maxCurrents = False
            if self.emergencyStop:
                print "/emergency"
                rospy.Publisher("/emergency", Bool, latch=True).publish(True)
                self.emergencyStop = False
            
            r.sleep()
            
    def getEndPosition(self):
        #TODO: these frames are hard coded - need to make them parameters
        frame_from = '/schunk/position/PAM112_BaseConector'
        frame_to = '/schunk/position/GripperBox'

        try:
            now = rospy.Time(0) # just get the latest rospy.Time.now()
            self.tfListener.waitForTransform(frame_from, frame_to, now, rospy.Duration(3.0))
            (trans,rot) = self.tfListener.lookupTransform(frame_from, frame_to, now)
        except (tf.LookupException, tf.ConnectivityException):
            print "Can't get end effector transform.!"
            return 0, 0, 0, 0, 0, 0, 0
        return trans[0], trans[1], trans[2], rot[0], rot[1], rot[2], rot[3]




class SchunkTextControl:
    def __init__(self):
        argc = len(sys.argv)
        argv = sys.argv
        
        # roscomms
        self.roscomms = RosCommunication()
        self.roscommsThread = Thread(target=self.roscomms.loop)
        #Thread(target=self.roscomms.loop).start() # run roscomms in a seperate thread
        self.roscommsThread.start()
        
        # get number of modules
        self.numModules = self.roscomms.numModules
        
        # load gui
#        self.wTree = gtk.glade.XML("gui2.glade", "window1")
        self.wTree = gtk.Builder()
        self.wTree.add_from_file("gui3.glade") 

        # set some initial display messages
        self.wTree.get_object("labelNumModules").set_text(str(self.numModules))
        self.wTree.get_object("status").set_text("Yes, Master")
        self.commandWidget = self.wTree.get_object("command")
        
        # bindings
        bindings = {"on_window1_destroy":self.shutdown, 
                    "on_buttonClear_clicked":self.clear, 
                    "on_buttonExecute_clicked":self.execute, 
                    "on_command_changed":self.command_changed,
                    "on_tbHelp_toggled":self.tb_help,
                    "on_command_move_active":self.history, 
                    "on_buttonSavePose_clicked":self.save_pose,
                    "on_buttonLoadPose_clicked":self.load_pose,
                    "on_buttonMoveAll_clicked":self.cb_move_all,
                    "on_buttonAckAll_clicked":self.cb_ack_all,
                    "on_buttonRefAll_clicked":self.cb_ref_all,
                    "on_tbEmergency_toggled":self.emergency_stop,
                    "on_buttonMoveVelAll_clicked":self.cb_move_vel_all,
                    "on_buttonCurMax_clicked":self.cb_currents_max,
                    "on_buttonVelStop_clicked":self.cb_stop_vel_all,
                    "on_radiobuttonJointAngleDegrees_toggled":self.degrees_or_radians,
                    "on_buttonAddJointsAnglesVector_clicked":self.add_joints_angles_vector }
        #self.wTree.signal_autoconnect(bindings)
        self.wTree.connect_signals(bindings)
        # Text input field of comboboxentry command is a gtk.Entry object
        entry = self.commandWidget.get_children()[0]
        entry.connect("activate", self.command_enter_pressed, self.commandWidget)

        # handle history
        self.historyLength = 100
        self.history = gtk.ListStore(gobject.TYPE_STRING)
        self.historyCounter = 0
        self.history_append("")
        self.load_history()
        self.commandWidget.set_model(self.history)
        self.commandWidget.set_text_column(0)
        self.commandWidget.set_active(self.historyCounter-1)
        
        # vocabulary and auto completion
        self.completion = gtk.EntryCompletion()
        self.vocabulary = gtk.ListStore(gobject.TYPE_STRING)
        #self.words = ["help", "info", "ack", "ref", "move", "curmax", "save", "load", "vel", "setvel", "setcur" ]
        self.words = ["help", "ack", "ref", "move", "vel", "curmax", "save", "load" ]
        self.vocabulary = self.add_words(self.words)
        self.completion.set_model(self.vocabulary)
        self.completion.set_minimum_key_length(1)
        self.completion.set_text_column(0)
        self.commandWidget.child.set_completion(self.completion)
        
        # set help box
        self.wTree.get_object("labelHelp").set_text(str(self.words))

        # pose limits of joints (deg/s)
        self.pose = []
        self.modules_maxlimits = []
        self.modules_minlimits = []
        self.limitsStrings = []
        for i in range(0,self.numModules):
            self.pose.append(0)
            moduleName = "Joint" + str(i)
            minLimit = self.roscomms.free_joints[moduleName]["min"]
            minLimit *= 180 / pi
            minLimit = int(minLimit) - 1
            maxLimit = self.roscomms.free_joints[moduleName]["max"]
            maxLimit *= 180 / pi
            maxLimit = int(maxLimit) + 1
            self.modules_minlimits.append(minLimit)
            self.modules_maxlimits.append(maxLimit)
            string = str(minLimit) + " to " + str(maxLimit)
            self.limitsStrings.append(string)
        
        # vel limits of joints (deg/s)
        self.modules_velmax = 90
        self.modules_velmin = -90
        
        # position fields
        posesframe_hbox = gtk.HBox(False, 6)
        posesframe_vboxes = []
        posesframe_labels = []
        self.posesframe_spinButtons = []
        for i in range (0,self.numModules):
            #name = "joint " + str(i) + ":"
            name = str(i) + ":"
            vbox = gtk.VBox(False, 0)
            posesframe_vboxes.append(vbox)
            label = gtk.Label(name)
            posesframe_labels.append(label)
            spinButton = gtk.SpinButton(digits=4)
            spinButton.set_range(self.modules_minlimits[i], self.modules_maxlimits[i])
            spinButton.set_increments(1, 5)
            spinButton.connect("activate", self.pose_spinButton_enter_pressed)
            tooltip = "Move Joint" + str(i) + " to given position"
            spinButton.set_tooltip_text(tooltip)
            self.posesframe_spinButtons.append(spinButton)
            vbox.add(label)
            vbox.add(spinButton)
            posesframe_hbox.add(vbox)
        self.wTree.get_object("posesFrame").add(posesframe_hbox)
        self.wTree.get_object("posesFrame").show_all()
        
        # velocity fields
        velframe_hbox = gtk.HBox(False, 6)
        velframe_vboxes = []
        velframe_labels = []
        self.velframe_spinButtons = []
        for i in range (0,self.numModules):
            name = str(i) + ":"
            vbox = gtk.VBox(False, 0)
            velframe_vboxes.append(vbox)
            label = gtk.Label(name)
            velframe_labels.append(label)
            spinButton = gtk.SpinButton()
            spinButton.set_range(self.modules_velmin, self.modules_velmax)
            spinButton.set_increments(1, 5)
            spinButton.connect("activate", self.vel_spinButton_enter_pressed)
            tooltip = "Move Joint" + str(i) + " with given velocity"
            spinButton.set_tooltip_text(tooltip)
            self.velframe_spinButtons.append(spinButton)
            vbox.add(label)
            vbox.add(spinButton)
            velframe_hbox.add(vbox)
        self.wTree.get_object("velFrame").add(velframe_hbox)
        self.wTree.get_object("velFrame").show_all()
        
        # flags fields
        flagsTitles = ["Position", "Referenced", "MoveEnd", "Brake", "Warning", "Current", "Moving", "PosReached", "Error", "Error code"]
        self.flagsDict = {"Position":0, "Referenced":1, "MoveEnd":2, "Brake":3, "Warning": 4, "Current":5, "Moving":6, "PosReached":7, "Error":8, "ErrorCode": 9}
        self.tableFlags = gtk.Table(self.numModules+1, len(flagsTitles)+1, homogeneous=False)
        self.tableFlags.set_col_spacings(12)
        self.wTree.get_object("flagsFrame").add(self.tableFlags)
        label = gtk.Label("#")
        self.tableFlags.attach(label, 0, 1, 0, 1)
        for i in range(1,len(flagsTitles)+1): # skip first column
            label = gtk.Label(flagsTitles[i-1])
            self.tableFlags.attach(label, i, i+1, 0, 1)
        self.flags = []
        for i in range(1,self.numModules+1): # need to skip first title row
            label = gtk.Label(str(i-1)+":")
            self.tableFlags.attach(label, 0, 1, i, i+1)
            flagsRow = []
            for j in range(1,len(flagsTitles)+1): # also skip first index column
                label = gtk.Label(".")
                self.tableFlags.attach(label, j, j+1, i, i+1)
                flagsRow.append(label)
            self.flags.append(flagsRow)
        self.wTree.get_object("flagsFrame").show_all()
                        
        # no argument full interface, also medium and mini modes
        if argc > 1:
            if (argv[1] == "medium") or (argv[1] == "mini"):
                self.wTree.get_object("aPoseFrame").hide()
                self.wTree.get_object("aVelFrame").hide()
            if argv[1] == "mini":
                self.wTree.get_object("aFlagsFrame").hide()
        w = self.wTree.get_object("window1")
        w.resize(*w.size_request())
        
        # in degrees
        self.inDegrees = self.wTree.get_object("radiobuttonJointAngleDegrees").get_active()
        print self.inDegrees
        

    def shutdown(self, widget):
        # kill gtk thread
        gtk.main_quit()
        # kill roscomms thread
        self.roscommsThread.join(0) # I think it is dead!
        # kill ros thread
        rospy.signal_shutdown("Because I said so!")


    def parser(self, string):
        tokens = string.split()
        if tokens[0] == "ack":
            self.ack(tokens)
        elif tokens[0] == "ref":
            self.ref(tokens)
        elif tokens[0] == "move":
            self.move(tokens)
        elif tokens[0] == "vel":
            self.move_vel(tokens)
        elif tokens[0] == "curmax":
            self.currents_max(tokens)
        elif tokens[0] == "help":
            self.help()
        else:
            self.command_not_found(tokens[0])


    def command_enter_pressed(self, entry, combo):
        self.wTree.get_object("buttonExecute").activate()


    def clear(self, widget):
        self.commandWidget.set_active(self.historyCounter-1)


    def execute(self, widget):
        commandStr = self.commandWidget.get_active_text()
        if commandStr != "":
            #iterator = self.store.get_iter_from_string("")
            #self.store.insert_before(iterator, [commandStr])
            self.history_append(commandStr)
            self.parser(commandStr)
            self.commandWidget.set_active(self.historyCounter-1)


    def emergency_stop(self, widget):
        if widget.get_active():
            # STOP
            self.roscomms.emergencyStop = True
            self.wTree.get_object("aPoseFrame").set_sensitive(False)
            self.wTree.get_object("aVelFrame").set_sensitive(False)
            self.wTree.get_object("aFlagsFrame").set_sensitive(False)
            self.wTree.get_object("vboxCommand").set_sensitive(False)
            self.wTree.get_object("image1").set_from_file("go75.png")
            self.wTree.get_object("status").set_text("Astalavista baby. No way Master Yianni's fault")
            self.wTree.get_object("status").modify_fg(gtk.STATE_NORMAL, gtk.gdk.color_parse('#FF0000'))
        else:
            # GO
            #self.roscomms.emergencyStop = False
            #self.ack("ack all".split())
            for i in range(0,self.numModules):
                command = "ack " + str(i) 
                self.ack(command.split())
                while self.roscomms.ackJoint == True:
                    pass
            self.wTree.get_object("aPoseFrame").set_sensitive(True)
            self.wTree.get_object("aVelFrame").set_sensitive(True)
            self.wTree.get_object("aFlagsFrame").set_sensitive(True)
            self.wTree.get_object("vboxCommand").set_sensitive(True)
            self.wTree.get_object("image1").set_from_file("stop75.png")
            self.wTree.get_object("status").set_text("I am back, Master")
            self.wTree.get_object("status").modify_fg(gtk.STATE_NORMAL, gtk.gdk.color_parse('#000000'))


    def load_history(self):
        try:
            f = open("history", "r")
            lines = f.readlines()
            f.close()
            lines = lines[-self.historyLength:]
            for line in lines:
                line = line.rstrip("\n")
                self.history_append(line, False)
        except:
            pass


    def history_append_to_file(self, string):
        f = open("history", "a")
        f.write(string + "\n")
        f.close()


    def history_append(self, string, tofile=True):
        self.history.insert(self.historyCounter-1, [string])
        self.historyCounter += 1
        if tofile and (string != ""):
            self.history_append_to_file(string)


    def history(self, widget, arg2):
        # following should bring the cursor to the end but it does not unless you hit the up arrow twice and this works only on the top item
        widget.child.set_position(-1)


    def save_pose(self, widget):
        # get the joint angles from the robot or from the pose display
        #eg.
        jointAngles = []
        for spinButton in self.posesframe_spinButtons:
            print spinButton.get_value()
            jointAngles.append(spinButton.get_value())
        dialog = gtk.FileChooserDialog(title="Save pose",
                                       action=gtk.FILE_CHOOSER_ACTION_SAVE,
                                       buttons=(gtk.STOCK_CANCEL, gtk.RESPONSE_CANCEL, 
                                                gtk.STOCK_SAVE, gtk.RESPONSE_OK))
        dialog.set_default_response(gtk.RESPONSE_OK)
        dialog.set_do_overwrite_confirmation(True)
        response = dialog.run()
        if response == gtk.RESPONSE_OK:
            filename = dialog.get_filename()
            writer = csv.writer(open(filename, "wb"))
            writer.writerow(jointAngles)
        elif response == gtk.RESPONSE_CANCEL:
            pass
        dialog.destroy()
        

    def load_pose(self, widget):
        dialog = gtk.FileChooserDialog(title="Load pose", 
                                       action=gtk.FILE_CHOOSER_ACTION_OPEN, 
                                       buttons=(gtk.STOCK_CANCEL, gtk.RESPONSE_CANCEL,
                                                 gtk.STOCK_OPEN, gtk.RESPONSE_OK))
        dialog.set_default_response(gtk.RESPONSE_OK)
        dialog.select_filename("default.pose")
        response = dialog.run()
        if response == gtk.RESPONSE_OK:
            filename = dialog.get_filename()
            reader = csv.reader(open(filename, "rb"))
            for row in reader:
                for i in range(0,len(self.pose)):
                    try:
                        self.pose[i] = float(row[i])
                    except:
                        print "error in loading pose"
                        return
            self.update_pose_display()
        elif response == gtk.RESPONSE_CANCEL:
            pass
        dialog.destroy()
        pass


    def add_words(self, words):
        vocabulary = gtk.ListStore(gobject.TYPE_STRING)
        for word in words:
            vocabulary.append([word])
        return vocabulary


    def command_changed(self, widget):
        tokens = widget.get_active_text().split()
        if tokens != []:
            self.wTree.get_object("status").modify_fg(gtk.STATE_NORMAL, gtk.gdk.color_parse('#000000'))
        try:
            if tokens[0] == "move":
                try:
                    module = int(tokens[1])
                    try:
                        string = "Range (deg/s): " + self.limitsStrings[module]
                        self.wTree.get_object("status").set_text(string)
                        try:
                            value = int(tokens[2])
                            if value > self.modules_maxlimits[module] or value < self.modules_minlimits[module]:
                                self.wTree.get_object("status").set_text("WARNING: Let me see you licking your elbow mate")
                                self.wTree.get_object("status").modify_fg(gtk.STATE_NORMAL, gtk.gdk.color_parse('#FF00FF'))
                        except:
                            pass
                    except:
                        self.wTree.get_object("status").set_text("WARNING: module does not exist")
                        self.wTree.get_object("status").modify_fg(gtk.STATE_NORMAL, gtk.gdk.color_parse('#FF00FF'))
                except:
                    self.wTree.get_object("status").set_text("")
                    
            elif tokens[0] == "vel":
                try:
                    module = int(tokens[1])
                    try:
                        moduleExists = self.modules_maxlimits[module] # this is actually the joint angle limits and not the velocity, but it helps to detect whether the module entered exists
                        string = "Range (deg/s): " + str(self.modules_velmin) + " to " + str(self.modules_velmax)
                        self.wTree.get_object("status").set_text(string)
                        try:
                            value = int(tokens[2])
                            if value > self.modules_velmax or value < self.modules_velmin:
                                self.wTree.get_object("status").set_text("WARNING: Hope you have a safe distance mate")
                                self.wTree.get_object("status").modify_fg(gtk.STATE_NORMAL, gtk.gdk.color_parse('#FF00FF'))
                        except:
                            pass
                    except:
                        self.wTree.get_object("status").set_text("WARNING: module does not exist")
                        self.wTree.get_object("status").modify_fg(gtk.STATE_NORMAL, gtk.gdk.color_parse('#FF00FF'))
                except:
                    self.wTree.get_object("status").set_text("")                
                pass
            else:
                try:
                    module = int(tokens[1])
                    if (module >= self.numModules) or (module < 0):
                        self.wTree.get_object("status").set_text("WARNING: module does not exist")
                        self.wTree.get_object("status").modify_fg(gtk.STATE_NORMAL, gtk.gdk.color_parse('#FF00FF'))
                except:
                    self.wTree.get_object("status").set_text("")
        except:
            pass


    def tb_help(self, widget):
        label = self.wTree.get_object("labelHelp")
        if widget.get_active():
            label.show()
        else:
            label.hide()


    def help(self):
        self.wTree.get_object("tbHelp").set_active(True)


    def update_pose_display(self):
        for i in range(0,len(self.pose)):
            try:
                self.posesframe_spinButtons[i].set_value(self.pose[i])
            except:
                print "error message on " + str(i)
                return


    def cb_ack_all(self, widget):
        command = "ack all"
        tokens = command.split()
        self.ack(tokens)
        # yes I know it could be
        #self.ack("ack all".split())


    def ack(self, tokens):
        try:
            module = tokens[1]
            if module == "all":
                self.roscomms.ackAll = True
                return
            try:
                module = int(module)
                if module >= 0 and module < self.numModules:
                    self.roscomms.ackNumber = module
                    self.roscomms.ackJoint = True
                else:
                    self.wTree.get_object("status").set_text("ERROR: ack failed. Module does not exist")
                    self.wTree.get_object("status").modify_fg(gtk.STATE_NORMAL, gtk.gdk.color_parse('#FF0000'))
            except:
                self.wTree.get_object("status").set_text("ERROR: move velocity failed. Module does not exist")
                self.wTree.get_object("status").modify_fg(gtk.STATE_NORMAL, gtk.gdk.color_parse('#FF0000'))
        except:
            self.roscomms.ackAll = True


    def cb_ref_all(self, widget):
        command = "ref all"
        tokens = command.split()
        self.ref(tokens)
        #or yes I know it could be
        #self.ref("ref all".split())


    def ref(self, tokens):
        try:
            module = tokens[1]
            if module == "all":
                self.roscomms.refAll = True
                return
            try:
                module = int(module)
                if module >= 0 and module < self.numModules:
                    self.roscomms.refNumber = module
                    self.roscomms.refJoint = True
                else:
                    self.wTree.get_object("status").set_text("ERROR: ref failed. Module does not exist")
                    self.wTree.get_object("status").modify_fg(gtk.STATE_NORMAL, gtk.gdk.color_parse('#FF0000'))
            except:
                self.wTree.get_object("status").set_text("ERROR: ref failed. Module does not exist")
                self.wTree.get_object("status").modify_fg(gtk.STATE_NORMAL, gtk.gdk.color_parse('#FF0000'))
        except:
            self.wTree.get_object("status").set_text("ERROR: ref failed. Need to specify module id or 'all'")
            self.wTree.get_object("status").modify_fg(gtk.STATE_NORMAL, gtk.gdk.color_parse('#FF0000'))


    def cb_move_all(self, widget):
        self.move_all()


    def pose_spinButton_enter_pressed(self, widget):
        module = self.posesframe_spinButtons.index(widget)
        self.posesframe_spinButtons[module].update()
        value = float(self.posesframe_spinButtons[module].get_value())
        command = "move " + str(module) + " " + str(value)
        tokens = command.split()
        self.move(tokens)


    def move(self, tokens):
        try:
            module = tokens[1]
            if module == "all":
                self.move_all()
                return
            try:
                module = int(module)
                if module >= 0 and module < self.numModules:
                    try:
                        value = tokens[2]
                        if self.inDegrees:
                            valueCheckLimit = int(value)
                        else:
                            valueCheckLimit = float(value) * 180 / pi
                            valueCheckLimit = int(valueCheckLimit)
                        if valueCheckLimit > self.modules_maxlimits[module] or valueCheckLimit < self.modules_minlimits[module]:
                            self.wTree.get_object("status").set_text("ERROR: I told you I can't lick my elbow. Move failed")
                            self.wTree.get_object("status").modify_fg(gtk.STATE_NORMAL, gtk.gdk.color_parse('#FF0000'))
                            return
                        try:
                            value = float(value)
                            if self.inDegrees:
                                value = value * pi / 180
                            self.roscomms.targetPosition.name=[]
                            self.roscomms.targetPosition.name.append("Joint"+str(module))
                            self.roscomms.targetPosition.position = [value]
                            #print self.roscomms.targetPosition
                            self.roscomms.setPosition = True
                        except:
                            print "move failed: not valid value"
                    except:
                        # move from spinbutton if tokens[2] not given
                        value = float(self.posesframe_spinButtons[module].get_value()) * pi / 180
                        self.roscomms.targetPosition.name=[]
                        self.roscomms.targetPosition.name.append("Joint"+str(module)) 
                        self.roscomms.targetPosition.position = [value]
                        #print self.roscomms.targetPosition
                        self.roscomms.setPosition = True 
                else:
                    self.wTree.get_object("status").set_text("ERROR: move failed. Module does not exist")
                    self.wTree.get_object("status").modify_fg(gtk.STATE_NORMAL, gtk.gdk.color_parse('#FF0000'))
            except:
                self.wTree.get_object("status").set_text("ERROR: move failed. Module does not exist")
                self.wTree.get_object("status").modify_fg(gtk.STATE_NORMAL, gtk.gdk.color_parse('#FF0000'))
        except:
            self.wTree.get_object("status").set_text("ERROR: move failed. Need to specify module id or 'all'")
            self.wTree.get_object("status").modify_fg(gtk.STATE_NORMAL, gtk.gdk.color_parse('#FF0000'))      


    def move_all(self):
        self.roscomms.targetPosition.name=[]
        self.roscomms.targetPosition.position=[]
        for module in range(0,self.numModules):
            name = "Joint" + str(module)
            self.roscomms.targetPosition.name.append(name)
            value = float(self.posesframe_spinButtons[module].get_value())
            if self.inDegrees: # convert to radians, else it is already in radians
                 value *= pi / 180
            self.roscomms.targetPosition.position.append(value)
            #print roscomms.targetPosition
            self.roscomms.setPosition = True


    def cb_currents_max(self, widget):
        command = "curmax"
        tokens = command.split()
        self.currents_max(tokens)


    def currents_max(self, tokens):
        try:
            module = tokens[1]
            if module == "all":
                self.roscomms.maxCurrents = True
                return
            try:
                module = int(module)
                if module >= 0 and module < self.numModules:
                    pass
                else:
                    self.wTree.get_object("status").set_text("ERROR: currents max failed. Module does not exist")
                    self.wTree.get_object("status").modify_fg(gtk.STATE_NORMAL, gtk.gdk.color_parse('#FF0000'))
            except:
                self.wTree.get_object("status").set_text("ERROR: currents max failed. Module does not exist")
                self.wTree.get_object("status").modify_fg(gtk.STATE_NORMAL, gtk.gdk.color_parse('#FF0000'))
        except:
            self.roscomms.maxCurrents = True
    

    def cb_move_vel_all(self, widget):
        self.move_vel_all()

        
    def cb_stop_vel_all(self, widget):
        self.roscomms.targetVelocity.name=[]
        self.roscomms.targetVelocity.velocity=[]
        for module in range(0,self.numModules):
            name = "Joint" + str(module)
            self.roscomms.targetVelocity.name.append(name)
            value = 0.0
            self.roscomms.targetVelocity.velocity.append(value)
            self.roscomms.setVelocity = True
#        for i in range(0,self.numModules):
#            command = "vel " + str(i) + " 0"
#            tokens = command.split()
#            self.move_vel(tokens)

            
    def move_vel(self, tokens):
        try:
            module = tokens[1]
            if module == "all":
                self.move_vel_all()
                return
            try:
                module = int(module)
                if module >= 0 and module < self.numModules:
                    try:
                        value = tokens[2]
                        if int(value) > self.modules_velmax or int(value) < self.modules_velmin:
                            self.wTree.get_object("status").set_text("ERROR: Can't go at speed of light. Move velocity failed")
                            self.wTree.get_object("status").modify_fg(gtk.STATE_NORMAL, gtk.gdk.color_parse('#FF0000'))
                            return
                        try:
                            value = float(value) * pi / 180
                            self.roscomms.targetVelocity.name=[]
                            self.roscomms.targetVelocity.name.append("Joint"+str(module))
                            self.roscomms.targetVelocity.velocity = [value]
                            self.roscomms.setVelocity = True
                        except:
                            print "move_vel failed: not valid value"
                    except:
                        # move from spinbutton if tokens[2] not given
                        value = float(self.velframe_spinButtons[module].get_value()) * pi / 180
                        self.roscomms.targetVelocity.name=[]
                        self.roscomms.targetVelocity.name.append("Joint"+str(module)) 
                        self.roscomms.targetVelocity.velocity = [value]
                        self.roscomms.setVelocity = True 
                else:
                    self.wTree.get_object("status").set_text("ERROR: move velocity failed. Module does not exist")
                    self.wTree.get_object("status").modify_fg(gtk.STATE_NORMAL, gtk.gdk.color_parse('#FF0000'))
            except:
                self.wTree.get_object("status").set_text("ERROR: move velocity failed. Module does not exist")
                self.wTree.get_object("status").modify_fg(gtk.STATE_NORMAL, gtk.gdk.color_parse('#FF0000'))
        except:
            self.wTree.get_object("status").set_text("ERROR: move velocity failed. Need to specify module id or 'all'")
            self.wTree.get_object("status").modify_fg(gtk.STATE_NORMAL, gtk.gdk.color_parse('#FF0000'))
      

    def move_vel_all(self):
        self.roscomms.targetVelocity.name=[]
        self.roscomms.targetVelocity.velocity=[]
        for i in range(0,self.numModules):
            name = "Joint" + str(i)
            self.roscomms.targetVelocity.name.append(name)
            value = float(self.velframe_spinButtons[i].get_value()) * pi / 180
            self.roscomms.targetVelocity.velocity.append(value)
            self.roscomms.setVelocity = True


    def vel_spinButton_enter_pressed(self, widget):
        module = self.velframe_spinButtons.index(widget)
        self.velframe_spinButtons[module].update()
        value = float(self.velframe_spinButtons[module].get_value())
        command = "vel " + str(module) + " " + str(value)
        tokens = command.split()
        self.move_vel(tokens)


    def degrees_or_radians(self, widget):
        self.inDegrees = widget.get_active()
        if self.inDegrees:
            #self.wTree.get_object("labelJointAngles").set_text("Joint angles (deg)")
            for i in range(0,self.numModules):
                value = float(self.posesframe_spinButtons[i].get_value())
                value *= 180 / pi
                self.posesframe_spinButtons[i].set_range(self.modules_minlimits[i], self.modules_maxlimits[i])
                self.posesframe_spinButtons[i].set_value(value)
                self.posesframe_spinButtons[i].update()
        else:
            #self.wTree.get_object("labelJointAngles").set_text("Joint angles (rad)")
            for i in range(0,self.numModules):
                value = float(self.posesframe_spinButtons[i].get_value())
                value *= pi / 180
                self.posesframe_spinButtons[i].set_range(self.modules_minlimits[i]*pi/180, self.modules_maxlimits[i]*pi/180)
                self.posesframe_spinButtons[i].set_value(value)
                self.posesframe_spinButtons[i].update()
   

    def update_flags(self, *args):
        for i in range(0, self.numModules):
            label = self.flags[i][self.flagsDict["Position"]]
            flagRadians = self.roscomms.currentJointStates.position[i]            
            flag = flagRadians * 180 / pi
            if (flag < 0.05) and (flag > -0.05):
                flag = 0.0            
            string = "%.2f / %.2f" % (flag, flagRadians)
            label.set_text(string)
            #flag = round(flag, 2)
            #label.set_text(str(flag))
            
            label = self.flags[i][self.flagsDict["Referenced"]]
            flag = self.roscomms.currentSchunkStatus.joints[i].referenced
            label.set_text(str(flag))
            if not flag:
                label.modify_fg(gtk.STATE_NORMAL, gtk.gdk.color_parse('#FF0000'))
            else:
                label.modify_fg(gtk.STATE_NORMAL, gtk.gdk.color_parse('#000000'))
                
            label = self.flags[i][self.flagsDict["MoveEnd"]]
            flag = self.roscomms.currentSchunkStatus.joints[i].moveEnd
            label.set_text(str(flag))
            if not flag:
                label.modify_fg(gtk.STATE_NORMAL, gtk.gdk.color_parse('#FF0000'))
            else:
                label.modify_fg(gtk.STATE_NORMAL, gtk.gdk.color_parse('#000000'))

            label = self.flags[i][self.flagsDict["Brake"]]
            flag = self.roscomms.currentSchunkStatus.joints[i].brake
            label.set_text(str(flag))
            if not flag:
                label.modify_fg(gtk.STATE_NORMAL, gtk.gdk.color_parse('#FF0000'))
            else:
                label.modify_fg(gtk.STATE_NORMAL, gtk.gdk.color_parse('#000000'))


            label = self.flags[i][self.flagsDict["Warning"]]
            flag = self.roscomms.currentSchunkStatus.joints[i].warning
            label.set_text(str(flag))
            if flag:
                label.modify_fg(gtk.STATE_NORMAL, gtk.gdk.color_parse('#FF0000'))
            else:
                label.modify_fg(gtk.STATE_NORMAL, gtk.gdk.color_parse('#000000'))

            label = self.flags[i][self.flagsDict["Current"]]
            flag = self.roscomms.currentSchunkStatus.joints[i].current
            string = "%.2f" % flag
            label.set_text(string)
            #flag = round(flag,2)
            #label.set_text(str(flag))

            label = self.flags[i][self.flagsDict["Moving"]]
            flag = self.roscomms.currentSchunkStatus.joints[i].moving
            label.set_text(str(flag))
            if flag:
                label.modify_fg(gtk.STATE_NORMAL, gtk.gdk.color_parse('#FF0000'))
            else:
                label.modify_fg(gtk.STATE_NORMAL, gtk.gdk.color_parse('#000000'))

            label = self.flags[i][self.flagsDict["PosReached"]]
            flag = self.roscomms.currentSchunkStatus.joints[i].posReached
            label.set_text(str(flag))
            if not flag:
                label.modify_fg(gtk.STATE_NORMAL, gtk.gdk.color_parse('#FF0000'))
            else:
                label.modify_fg(gtk.STATE_NORMAL, gtk.gdk.color_parse('#000000'))
 
            label = self.flags[i][self.flagsDict["Error"]]
            flag = self.roscomms.currentSchunkStatus.joints[i].error
            label.set_text(str(flag))
            if flag:
                label.modify_fg(gtk.STATE_NORMAL, gtk.gdk.color_parse('#FF0000'))
            else:
                label.modify_fg(gtk.STATE_NORMAL, gtk.gdk.color_parse('#000000'))
                
            label = self.flags[i][self.flagsDict["ErrorCode"]]
            flag = self.roscomms.currentSchunkStatus.joints[i].errorCode
            label.set_text(str(flag))
            if flag != 0:
                label.modify_fg(gtk.STATE_NORMAL, gtk.gdk.color_parse('#FF0000'))
            else:
                label.modify_fg(gtk.STATE_NORMAL, gtk.gdk.color_parse('#000000'))           

        return True


    def update_pose(self, *args):
        value = self.roscomms.getEndPosition()
        pose = []
        for v in value:
            pose.append(v)
        for i in range(0,len(pose)):
            if pose[i] < 0.005 and pose[i] > -0.005:
                pose[i] = 0.0
        
        value = quaternion_to_euler(pose[3], pose[4], pose[5], pose[6])
        rpy = []
        for v in value:
            rpy.append(v*180/pi)
        for i in range(0,len(rpy)):
            if rpy[i] < 0.005 and rpy[i] > -0.005:
                rpy[i] = 0.0

        msg = "%.2f" % pose[0]
        self.wTree.get_object("poseX").set_text(msg)
        msg = "%.2f" % pose[1]
        self.wTree.get_object("poseY").set_text(msg)
        msg = "%.2f" % pose[2]
        self.wTree.get_object("poseZ").set_text(msg)
        msg = "%.2f" % rpy[0]
        self.wTree.get_object("poseRoll").set_text(msg)
        msg = "%.2f" % rpy[1]
        self.wTree.get_object("posePitch").set_text(msg)
        msg = "%.2f" % rpy[2]
        self.wTree.get_object("poseYaw").set_text(msg)
        msg = "%.2f" % pose[3]
        self.wTree.get_object("poseQx").set_text(msg)
        msg = "%.2f" % pose[4]
        self.wTree.get_object("poseQy").set_text(msg)
        msg = "%.2f" % pose[5]
        self.wTree.get_object("poseQz").set_text(msg)
        msg = "%.2f" % pose[6]
        self.wTree.get_object("poseQw").set_text(msg)

        return True


    def command_not_found(self, token):
        msg = "Ich spreche nicht Deutch. Was ist '" +  token + "'? Druckte 'Hilfe' fur Vokabelliste"
        self.wTree.get_object("status").set_text(msg)
        self.wTree.get_object("status").modify_fg(gtk.STATE_NORMAL, gtk.gdk.color_parse('#FF0000'))

# delete not needed any more
#    def get_limits_strings(self):
#        limitsStrings = {}
#        for i in range(0,self.numModules):
#            min = self.modules_minlimits[i]
#            max = self.modules_maxlimits[i]
#            string = str(min) + " to " + str(max)
#            limitsStrings[i] = string
#        return limitsStrings


def quaternion_to_euler(qx,qy,qz,qw):
    heading = math.atan2(2*qy*qw-2*qx*qz , 1 - 2*qy*qy - 2*qz*qz)
    attitude = math.asin(2*qx*qy + 2*qz*qw)
    bank = math.atan2(2*qx*qw-2*qy*qz , 1 - 2*qx*qx - 2*qz*qz)
    if math.fabs(qx*qy + qz*qw - 0.5) < 0.001: # (north pole):
        heading = 2 * math.atan2(qx,qw)
        bank = 0
    if math.fabs(qx*qy + qz*qw + 0.5) < 0.001: # (south pole):
        heading = -2 * math.atan2(qx,qw)   
        bank = 0

    return heading, attitude, bank


if __name__ == "__main__":
    wd = os.path.dirname(sys.argv[0])
    try:
        os.chdir(wd)
    except:
        print "Working directory does not exist. Check for mispellings"
        sys.exit(1)
    
    gtk.gdk.threads_init()
    rospy.init_node('schunk_gui_text')
    gui = SchunkTextControl()
    #Thread(target=gui.roscomms.loop).start() # statement is in the constructor of SchunkTextControl, either there or here
    gobject.timeout_add(100, gui.update_flags)
    gobject.timeout_add(100, gui.update_pose)
    gtk.main()
    rospy.spin()
