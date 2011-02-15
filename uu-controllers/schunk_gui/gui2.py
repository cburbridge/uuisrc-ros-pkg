#!/usr/bin/env python

try:
    import os
    import sys
    import gtk
    import gtk.glade
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
    from math import pi
    from math import degrees
    from threading import Thread
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
                self.ackJoint = False
                print "/ack"
                rospy.Publisher("/ack", Int8, latch=True).publish(self.ackNumber)
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



class SchunkTextControl:
    def __init__(self):
        argc = len(sys.argv)
        argv = sys.argv
        
        # roscomms
        self.roscomms = RosCommunication()
        Thread(target=self.roscomms.loop).start() # run roscomms in a seperate thread
        
        # get number of modules
        self.numModules = self.roscomms.numModules
        
        # load gui
        self.wTree = gtk.glade.XML("gui2.glade", "window1")
        self.wTree.get_widget("labelNumModules").set_text(str(self.numModules))
        self.commandWidget = self.wTree.get_widget("command")
        
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
                    "on_buttonCurMax_clicked":self.cb_currents_max }
        self.wTree.signal_autoconnect(bindings)
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
        self.wTree.get_widget("labelHelp").set_text(str(self.words))

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
            spinButton = gtk.SpinButton()
            spinButton.set_range(self.modules_minlimits[i], self.modules_maxlimits[i])
            spinButton.set_increments(1, 5)
            spinButton.connect("activate", self.pose_spinButton_enter_pressed)
            self.posesframe_spinButtons.append(spinButton)
            vbox.add(label)
            vbox.add(spinButton)
            posesframe_hbox.add(vbox)
        self.wTree.get_widget("posesFrame").add(posesframe_hbox)
        self.wTree.get_widget("posesFrame").show_all()
        
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
            self.velframe_spinButtons.append(spinButton)
            vbox.add(label)
            vbox.add(spinButton)
            velframe_hbox.add(vbox)
        self.wTree.get_widget("velFrame").add(velframe_hbox)
        self.wTree.get_widget("velFrame").show_all()
        
        # flags fields
        flagsTitles = ["Position", "Referenced", "MoveEnd", "Current", "Moving", "PosReached"]
        self.flagsDict = {"Position":0, "Referenced":1, "MoveEnd":2, "Current":3, "Moving":4, "PosReached":5}
        self.tableFlags = gtk.Table(self.numModules+1, len(flagsTitles)+1)
        self.wTree.get_widget("flagsFrame").add(self.tableFlags)
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
        self.wTree.get_widget("flagsFrame").show_all()
        self.wTree.get_widget("flagsFrame").set_sensitive(False)
                        
        # no argument full interface, also medium and mini modes
        if argc > 1:
            if (argv[1] == "medium") or (argv[1] == "mini"):
                self.wTree.get_widget("aPoseFrame").hide()
                self.wTree.get_widget("aVelFrame").hide()
            if argv[1] == "mini":
                self.wTree.get_widget("aFlagsFrame").hide()
        w = self.wTree.get_widget("window1")
        w.resize(*w.size_request())
        

    def test(self, widget):
        print "Hello world!"


    def shutdown(self, widget):
        #Thread(target=self.roscomms.loop).join()
        gtk.main_quit()


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
            self.command_not_found()


    def command_enter_pressed(self, entry, combo):
        self.wTree.get_widget("buttonExecute").activate()


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
            self.wTree.get_widget("aPoseFrame").set_sensitive(False)
            self.wTree.get_widget("aVelFrame").set_sensitive(False)
            self.wTree.get_widget("aFlagsFrame").set_sensitive(False)
            self.wTree.get_widget("vboxCommand").set_sensitive(False)
            self.wTree.get_widget("image1").set_from_file("go75.png")
            self.wTree.get_widget("status").set_text("Emergency stopped enabled. No way Yiannis is the coder.")
        else:
            # GO
            self.roscomms.emergencyStop = False
            self.ack("ack all".split())
            self.wTree.get_widget("aPoseFrame").set_sensitive(True)
            self.wTree.get_widget("aVelFrame").set_sensitive(True)
            self.wTree.get_widget("aFlagsFrame").set_sensitive(True)
            self.wTree.get_widget("vboxCommand").set_sensitive(True)
            self.wTree.get_widget("image1").set_from_file("stop75.png")
            self.wTree.get_widget("status").set_text("")


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
                        print "error"
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
            self.wTree.get_widget("status").modify_fg(gtk.STATE_NORMAL, gtk.gdk.color_parse('#000000'))
        try:
            if tokens[0] == "move":
                try:
                    module = int(tokens[1])
                    try:
                        string = "Range (deg/s): " + self.limitsStrings[module]
                        self.wTree.get_widget("status").set_text(string)
                        try:
                            value = int(tokens[2])
                            if value > self.modules_maxlimits[module] or value < self.modules_minlimits[module]:
                                self.wTree.get_widget("status").set_text("ERROR: Let me see licking your elbow mate")
                                self.wTree.get_widget("status").modify_fg(gtk.STATE_NORMAL, gtk.gdk.color_parse('#FF0000'))
                        except:
                            pass
                    except:
                        self.wTree.get_widget("status").set_text("ERROR: module not found")
                        self.wTree.get_widget("status").modify_fg(gtk.STATE_NORMAL, gtk.gdk.color_parse('#FF0000'))
                except:
                    self.wTree.get_widget("status").set_text("")
            elif tokens[0] == "vel":
                try:
                    module = int(tokens[1])
                    try:
                        string = "Range (deg/s): " + str(self.modules_velmin) + " to " + str(self.modules_velmax)
                        self.wTree.get_widget("status").set_text(string)
                        try:
                            value = int(tokens[2])
                            if value > self.modules_velmax or value < self.modules_velmin:
                                self.wTree.get_widget("status").set_text("ERROR: Hope you have a safe distance mate")
                                self.wTree.get_widget("status").modify_fg(gtk.STATE_NORMAL, gtk.gdk.color_parse('#FF0000'))
                        except:
                            pass
                    except:
                        self.wTree.get_widget("status").set_text("ERROR: module not found")
                        self.wTree.get_widget("status").modify_fg(gtk.STATE_NORMAL, gtk.gdk.color_parse('#FF0000'))
                except:
                    self.wTree.get_widget("status").set_text("")                
                pass
            else:
                try:
                    module = int(tokens[1])
                    if (module >= self.numModules) or (module < 0):
                        self.wTree.get_widget("status").set_text("ERROR: module not found")
                        self.wTree.get_widget("status").modify_fg(gtk.STATE_NORMAL, gtk.gdk.color_parse('#FF0000'))
                except:
                    self.wTree.get_widget("status").set_text("")
        except:
            pass


    def tb_help(self, widget):
        label = self.wTree.get_widget("labelHelp")
        if widget.get_active():
            label.show()
        else:
            label.hide()


    def help(self):
        self.wTree.get_widget("tbHelp").set_active(True)


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
                print "ack all"
                self.roscomms.ackAll = True
                return
            try:
                module = int(module)
                if module >= 0 and module < self.numModules:
                    self.roscomms.ackNumber = module
                    self.roscomms.ackJoint = True
                else:
                    print "ack failed: module does not exist"
            except:
                print "ack failed: module does not exist"
        except:
            print "ack all"
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
                print "ref all"
                self.roscomms.refAll = True
                return
            try:
                module = int(module)
                if module >= 0 and module < self.numModules:
                    self.roscomms.refNumber = module
                    self.roscomms.refJoint = True
                else:
                    print "ref failed: module does not exist"
            except:
                print "ref failed: module does not exist"
        except:
            print "Enter 'all' or module id"


    def cb_move_all(self, widget):
        self.move_all()


    def pose_spinButton_enter_pressed(self, widget):
        module = self.posesframe_spinButtons.index(widget)
        self.posesframe_spinButtons[module].update()
        value = float(self.posesframe_spinButtons[module].get_value())
        command = "move " + str(module) + " " + str(value)
        tokens = command.split()
        print tokens
        self.move(tokens)


    def move(self, tokens):
        print tokens
        try:
            module = tokens[1]
            if module == "all":
                print "move all"
                self.move_all()
                return
            try:
                module = int(module)
                if module >= 0 and module < self.numModules:
                    try:
                        value = tokens[2]
                        try:
                            value = float(value) * pi / 180
                            self.roscomms.targetPosition.name=[]
                            self.roscomms.targetPosition.name.append("Joint"+str(module))
                            self.roscomms.targetPosition.position = [value]
                            print self.roscomms.targetPosition
                            self.roscomms.setPosition = True
                        except:
                            print "move failed: not valid value"
                    except:
                        # move from spinbutton if tokens[2] not given
                        value = float(self.posesframe_spinButtons[module].get_value()) * pi / 180
                        self.roscomms.targetPosition.name=[]
                        self.roscomms.targetPosition.name.append("Joint"+str(module)) 
                        self.roscomms.targetPosition.position = [value]
                        print self.roscomms.targetPosition
                        self.roscomms.setPosition = True 
                else:
                    print "move failed: module does not exist"
            except:
                print "move failed: module does not exist"
        except:
            print "Enter 'all' or module id"


    def move_all(self):
        self.roscomms.targetPosition.name=[]
        self.roscomms.targetPosition.position=[]
        for module in range(0,self.numModules):
            name = "Joint" + str(module)
            self.roscomms.targetPosition.name.append(name)
            value = float(self.posesframe_spinButtons[module].get_value()) * pi / 180
            print value
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
                print "currents max all"
                self.roscomms.maxCurrents = True
                return
            try:
                module = int(module)
                if module >= 0 and module < self.numModules:
                    pass
                else:
                    print "ref failed: module does not exist"
            except:
                print "ref failed: module does not exist"
        except:
            print "currents max all"
            self.roscomms.maxCurrents = True
    

    def cb_move_vel_all(self, widget):
        self.move_vel_all()

            
    def move_vel(self, tokens):
        print tokens
        try:
            module = tokens[1]
            if module == "all":
                print "move_vel all"
                self.move_vel_all()
                return
            try:
                module = int(module)
                if module >= 0 and module < self.numModules:
                    try:
                        value = tokens[2]
                        if int(value) > self.modules_velmax or int(value) < self.modules_velmin:
                            self.wTree.get_widget("status").set_text("ERROR: Are you mental? Velocity out of range. Move velocity failed")
                            self.wTree.get_widget("status").modify_fg(gtk.STATE_NORMAL, gtk.gdk.color_parse('#FF0000'))
                            return                         
                        try:
                            value = float(value) * pi / 180
                            self.roscomms.targetVelocity.name=[]
                            self.roscomms.targetVelocity.name.append("Joint"+str(module))
                            self.roscomms.targetVelocity.velocity = [value]
                            #self.roscomms.setVelocity = True
                        except:
                            print "move_vel failed: not valid value"
                    except:
                        # move from spinbutton if tokens[2] not given
                        value = float(self.velframe_spinButtons[module].get_value()) * pi / 180
                        self.roscomms.targetVelocity.name=[]
                        self.roscomms.targetVelocity.name.append("Joint"+str(module)) 
                        self.roscomms.targetVelocity.velocity = [value]
                        #self.roscomms.setVelocity = True 
                else:
                    print "move_vel failed: module does not exist"
            except:
                print "move_vel failed: module does not exist"
        except:
            print "Enter 'all' or module id"        
      

    def move_vel_all(self):
        self.roscomms.targetVelocity.name=[]
        self.roscomms.targetVelocity.velocity=[]
        for module in range(0,self.numModules):
            name = "Joint" + str(module)
            self.roscomms.targetVelocity.name.append(name)
            value = float(self.velframe_spinButtons[module].get_value()) * pi / 180
            print value
            self.roscomms.targetVelocity.velocity.append(value)
            #self.roscomms.setVelocity = True


    def command_not_found(self):
        print "command not found"

# delete not needed any more
#    def get_limits_strings(self):
#        limitsStrings = {}
#        for i in range(0,self.numModules):
#            min = self.modules_minlimits[i]
#            max = self.modules_maxlimits[i]
#            string = str(min) + " to " + str(max)
#            limitsStrings[i] = string
#        return limitsStrings


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
    
    gtk.gdk.threads_init()
    rospy.init_node('schunk_gui_text')
    gui = SchunkTextControl()
    #Thread(target=gui.roscomms.loop).start() # statement is in the constructor of SchunkTextControl, either there or here
    gtk.main()
    rospy.spin()
