#!/usr/bin/python
# -*- coding: utf-8 -*-
# Schunk Control Interface
# A total hack
# Chris Burbridge cburbridge@gmail.com
# November 2010

import roslib; roslib.load_manifest('schunk_gui')
import sys
    
import rospy
import wx
import xml.dom.minidom
from sensor_msgs.msg import JointState
from metralabs_ros.msg import SchunkStatus
from std_msgs.msg import *
from math import pi
from math import degrees
from threading import Thread

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
        number=0
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
                self.joint_lookup[name]=number
                number=number+1

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

class SchunkGui(wx.Frame):
    def __init__(self, title, roscom):
        wx.Frame.__init__(self, None, -1, title, (-1, -1));
        self.roscomms = roscom
        self.joint_map = {}
        panel = wx.Panel(self, wx.ID_ANY);
        box = wx.BoxSizer(wx.VERTICAL)
        masterbox = wx.BoxSizer(wx.VERTICAL)
        font = wx.Font(9, wx.SWISS, wx.NORMAL, wx.BOLD)
        
        ### Sliders ###
        for name in self.roscomms.joint_list:
            joint = self.roscomms.free_joints[name]

            if joint['min'] == joint['max']:
                continue

            row = wx.FlexGridSizer(3,2)
            label = wx.StaticText(panel, -1, name)
            label.SetFont(font)
            row.Add(label, 1, wx.ALIGN_CENTER_VERTICAL)
            row.Add(wx.StaticText(panel, -1, ''), 1, wx.ALIGN_CENTER_VERTICAL)
            

            display = wx.StaticText (panel,-1, str(0), 
                        style=wx.TE_READONLY | wx.ALIGN_RIGHT)
                        
                        
            display_velocity = wx.StaticText (panel,-1, str(0), 
                        style=wx.TE_READONLY | wx.ALIGN_RIGHT)
#            row.Add(display, flag= wx.ALIGN_CENTER| wx.ALIGN_CENTER_VERTICAL)
            
#            position = wx.TextCtrl (panel, value=str(0), 
#                        style=wx.TE_READONLY | wx.ALIGN_RIGHT)
#            row.Add(position, flag= wx.ALIGN_RIGHT| wx.ALIGN_CENTER_VERTICAL)
            
#            box.Add(label, 1, wx.EXPAND) #was adding row
            
            slider = wx.Slider(panel, -1, RANGE/2, 0, RANGE, 
                        style= wx.SL_AUTOTICKS | wx.SL_HORIZONTAL)
            slider_vel = wx.Slider(panel, -1, 50, 0, 100, 	# fixed the resolution o
                        style= wx.SL_AUTOTICKS | wx.SL_HORIZONTAL)
            slider.SetFont(font)
	    slider.isvel = False

            slider_vel.SetFont(font)
            slider_vel.isvel = True
            
            row2 = wx.FlexGridSizer(1,3)  
            row2.AddGrowableCol(0,2)

            refandack = wx.FlexGridSizer(4,1)
            
            row.Add(slider,flag=wx.ALIGN_LEFT| wx.ALIGN_CENTER_VERTICAL|wx.EXPAND)            
            row.Add(display,1,wx.ALIGN_RIGHT | wx.ALIGN_CENTER_VERTICAL)
            row.Add(slider_vel,flag=wx.ALIGN_LEFT| wx.ALIGN_CENTER_VERTICAL|wx.EXPAND)
            
            row.Add(display_velocity, 1, wx.ALIGN_CENTER_VERTICAL)
            
            
            row.AddGrowableRow(1, 0)
            row.AddGrowableCol(0,1)

            row2.Add(row,flag=wx.ALIGN_LEFT| wx.ALIGN_CENTER_VERTICAL|wx.EXPAND)
            
            # Add a "go" buttons
            btnAck = wx.Button(panel, id=wx.ID_ANY, label="Ack", name=name)
            btnRef = wx.Button(panel, id=wx.ID_ANY, label="Ref", name=name)
            btnMove = wx.Button(panel, id=wx.ID_ANY, label="Move", name=name)
            btnVel = wx.Button(panel, id=wx.ID_ANY, label="MoveVel", name=name)
            refandack.Add(btnAck, 1, wx.EXPAND)
            refandack.Add(btnRef, 1, wx.EXPAND)
            refandack.Add(btnMove, 1, wx.EXPAND)
            refandack.Add(btnVel, 1, wx.EXPAND)
            row2.Add(refandack,flag=wx.ALIGN_LEFT| wx.ALIGN_CENTER_VERTICAL|wx.EXPAND)
            
            # The status flags
            statusflags = wx.GridSizer(5,4)
            statusflags.Add(wx.StaticText (panel,-1, 'Position:', style=wx.TE_READONLY | wx.ALIGN_LEFT))
            positionlabel=wx.StaticText (panel,-1, '0', style=wx.TE_READONLY | wx.ALIGN_LEFT)
            statusflags.Add(positionlabel)
            statusflags.Add(wx.StaticText (panel,-1, 'Current:', style=wx.TE_READONLY | wx.ALIGN_LEFT))
            currentlabel=wx.StaticText (panel,-1, '0', style=wx.TE_READONLY | wx.ALIGN_LEFT)
            statusflags.Add(currentlabel)
            statusflags.Add(wx.StaticText (panel,-1, 'Referenced:', style=wx.TE_READONLY | wx.ALIGN_LEFT))
            referencedlabel=wx.StaticText (panel,-1, '0', style=wx.TE_READONLY | wx.ALIGN_LEFT)
            statusflags.Add(referencedlabel)
            statusflags.Add(wx.StaticText (panel,-1, 'Moving:', style=wx.TE_READONLY | wx.ALIGN_LEFT))
            movinglabel=wx.StaticText (panel,-1, '0', style=wx.TE_READONLY | wx.ALIGN_LEFT)
            statusflags.Add(movinglabel)
            statusflags.Add(wx.StaticText (panel,-1, 'MoveEnd:', style=wx.TE_READONLY | wx.ALIGN_LEFT))
            moveendlabel=wx.StaticText (panel,-1, '0', style=wx.TE_READONLY | wx.ALIGN_LEFT)
            statusflags.Add(moveendlabel)
            statusflags.Add(wx.StaticText (panel,-1, 'PosReached:', style=wx.TE_READONLY | wx.ALIGN_LEFT))
            posreachedlabel=wx.StaticText (panel,-1, '0', style=wx.TE_READONLY | wx.ALIGN_LEFT)
            statusflags.Add(posreachedlabel)
            statusflags.Add(wx.StaticText (panel,-1, 'Brake:', style=wx.TE_READONLY | wx.ALIGN_LEFT))
            brakelabel=wx.StaticText (panel,-1, '0', style=wx.TE_READONLY | wx.ALIGN_LEFT)
            statusflags.Add(brakelabel)
            statusflags.Add(wx.StaticText (panel,-1, 'Error:', style=wx.TE_READONLY | wx.ALIGN_LEFT))
            errorlabel=wx.StaticText (panel,-1, '0', style=wx.TE_READONLY | wx.ALIGN_LEFT)
            statusflags.Add(errorlabel)
            statusflags.Add(wx.StaticText (panel,-1, 'Warning:', style=wx.TE_READONLY | wx.ALIGN_LEFT))
            warninglabel=wx.StaticText (panel,-1, '0', style=wx.TE_READONLY | wx.ALIGN_LEFT)
            statusflags.Add(warninglabel)
            statusflags.Add(wx.StaticText (panel,-1, 'Error code:', style=wx.TE_READONLY | wx.ALIGN_LEFT))
            errorcodelabel=wx.StaticText (panel,-1, '0', style=wx.TE_READONLY | wx.ALIGN_LEFT)
            statusflags.Add(errorcodelabel)
#            self.celsius =  wx.StaticText(self, -1, '', (150, 150))
            row2.Add(statusflags,flag=wx.ALIGN_LEFT| wx.ALIGN_CENTER_VERTICAL|wx.EXPAND)

            box.Add(row2, 1, wx.EXPAND)
            line =wx.StaticLine(panel, -1)
            line.SetBackgroundColour("Black")
            box.Add(line)

            self.joint_map[name] = {'slidervalue':0, 'display':display, 'display_velocity':display_velocity, 
                                    'slider':slider, 'joint':joint, 
                                    'positionlabel':positionlabel, 'currentlabel':currentlabel,
                                    'referencedlabel':referencedlabel, 'movinglabel':movinglabel,
                                    'moveendlabel':moveendlabel, 'posreachedlabel':posreachedlabel,
                                    'brakelabel':brakelabel, 'errorlabel':errorlabel,
                                    'warninglabel':warninglabel, 'errorcodelabel':errorcodelabel,
                                    'slider_vel': slider_vel }
                                    

        ### Buttons ###
        self.loadbutton = wx.Button(panel, 1, 'Load Pose')
        self.savebutton = wx.Button(panel, 1, 'Save Pose')
        self.ctrbutton = wx.Button(panel, 1, 'Ack All')
        self.refallbutton = wx.Button(panel, 1, 'Ref All')
        self.moveallbutton = wx.Button(panel, 1, 'Move All')
        self.maxcurrentbutton = wx.Button(panel, 1, 'Max Currents')
        self.emergency = wx.Button(panel, 1, 'EMERGENCY STOP')
        self.emergency.SetBackgroundColour("Red")
        
        self.Bind(wx.EVT_SLIDER, self.sliderUpdate)
        basebuttons = wx.GridSizer(2,3)
        basebuttons.Add(self.ctrbutton, 0, wx.EXPAND)
        basebuttons.Add(self.refallbutton, 0, wx.EXPAND)
        basebuttons.Add(self.maxcurrentbutton, 0, wx.EXPAND)
        basebuttons.Add(self.moveallbutton, 0, wx.EXPAND)
        basebuttons.Add(self.loadbutton, 0, wx.EXPAND)
        basebuttons.Add(self.savebutton, 0, wx.EXPAND)
        
        box.Add(basebuttons, 0, wx.EXPAND)
        box.Add(self.emergency, 0, wx.EXPAND)
        
        
        self.Bind(wx.EVT_BUTTON, self.button_event)
        panel.SetSizer(masterbox)
        self.center()
        box.Fit(self)
        masterbox.Add(box,1,wx.EXPAND)
        masterbox.Fit(self)
        self.update_values()
        
        self.Bind(wx.EVT_CLOSE, self.OnClose)

        self.timer = wx.Timer(self)
        self.timer.Start(100) #10hz
        self.Bind(wx.EVT_TIMER, self.OnTimer, self.timer)  # call the on_timer function

    def OnTimer(self,event):
        for jointstatus in  self.roscomms.currentSchunkStatus.joints:
            joint=self.joint_map[jointstatus.jointName]
            if jointstatus.warning:
                joint['warninglabel'].SetLabel("True")
                joint['warninglabel'].SetForegroundColour("Red")
            else:
                joint['warninglabel'].SetLabel("False")
                joint['warninglabel'].SetForegroundColour("Black")
            if jointstatus.referenced:
                joint['referencedlabel'].SetLabel("True")
                joint['referencedlabel'].SetForegroundColour("Black")

            else:
                joint['referencedlabel'].SetLabel("False")
                joint['referencedlabel'].SetForegroundColour("Red")
            if jointstatus.moving:
                joint['movinglabel'].SetLabel("True")
                joint['movinglabel'].SetForegroundColour("Green")
            else:
                joint['movinglabel'].SetLabel("False")
                joint['movinglabel'].SetForegroundColour("Black")
            if jointstatus.error:
                joint['errorlabel'].SetLabel("True")
                joint['errorlabel'].SetForegroundColour("Red")
            else:
                joint['errorlabel'].SetLabel("False")
                joint['errorlabel'].SetForegroundColour("Black")
            if jointstatus.brake:
                joint['brakelabel'].SetLabel("True")
                joint['brakelabel'].SetForegroundColour("Black")
            else:
                joint['brakelabel'].SetLabel("False")
                joint['brakelabel'].SetForegroundColour("Green")
            if jointstatus.moveEnd:
                joint['moveendlabel'].SetLabel("True")
            else:
                joint['moveendlabel'].SetLabel("False")
            if jointstatus.posReached:
                joint['posreachedlabel'].SetLabel("True")
            else:
                joint['posreachedlabel'].SetLabel("False")
            
            joint['errorcodelabel'].SetLabel("%d"%jointstatus.errorCode)
            joint['currentlabel'].SetLabel("%.2f"%jointstatus.current)

        for jointname,jointposition in  zip(self.roscomms.currentJointStates.name,self.roscomms.currentJointStates.position):
            self.joint_map[jointname]['positionlabel'].SetLabel("%.2f"%degrees(jointposition))
            pass

        pass
        
    def OnClose(self, event):
        print "Closing"
        self.timer.Stop()
        self.Destroy()


    def update_values(self):
        for (name,joint_info) in self.joint_map.items():
            purevalue = joint_info['slidervalue']
            joint = joint_info['joint']
            value = self.sliderToValue(purevalue, joint)
            joint['value'] = value
            joint_info['slider'].SetValue(purevalue)
            joint_info['display'].SetLabel("%.2f"%degrees(value))
    
    def update_velocity_values(self):
        for (name,joint_info) in self.joint_map.items():
            purevalue = joint_info['slider_velocity_value']
            joint = joint_info['joint']
            #value = (purevalue)/100. *(1.57+1.57) + (-1.57)  #self.sliderToValue(purevalue, joint)
            value = (purevalue-50.)/50. *1.57 #self.sliderToValue(purevalue, joint)
            joint['value_velocity'] = value
            #joint_info['slider_vel'].SetValue(purevalue)
            joint_info['display_velocity'].SetLabel("%.2f"%degrees(value))
                    

    def button_event(self, event):
        button = event.GetEventObject()
        if button.GetLabel()=='EMERGENCY STOP':
            roscomms.emergencyStop = True
            print 'Ahhhhhhh!'
        elif button.GetLabel()=='Ack':
            roscomms.ackNumber=roscomms.joint_lookup[button.GetName()]
            roscomms.ackJoint = True
        elif button.GetLabel()=='Ref':
            roscomms.refNumber=roscomms.joint_lookup[button.GetName()]
            roscomms.refJoint = True
        elif button.GetLabel()=='Move':
            ## populate the roscoms command message
            roscomms.targetPosition.name=[]
            roscomms.targetPosition.name.append(button.GetName().encode('ascii','ignore')) 
            roscomms.targetPosition.position=[self.joint_map[button.GetName()]['joint']['value']]
            print roscomms.targetPosition
            roscomms.setPosition = True
	elif button.GetLabel()=='MoveVel':
            ## populate the roscoms command message
            roscomms.targetVelocity.name=[]
            roscomms.targetVelocity.name.append(button.GetName().encode('ascii','ignore')) 
            roscomms.targetVelocity.velocity=[self.joint_map[button.GetName()]['joint']['value_velocity']]
            #print roscomms.targetPosition
            roscomms.setVelocity = True
        elif button.GetLabel()=='Ack All':
            roscomms.ackAll = True
        elif button.GetLabel()=='Ref All':
            roscomms.refAll = True
        elif button.GetLabel()=='Max Currents':
            roscomms.maxCurrents = True
        elif button.GetLabel()=='Move All':
            roscomms.targetPosition.name=[]
            roscomms.targetPosition.position=[]
            for (name,joint_info) in self.joint_map.items():
                roscomms.targetPosition.name.append(name.encode('ascii','ignore'))
                roscomms.targetPosition.position.append(joint_info['joint']['value'])
#            print roscomms.targetPosition
            roscomms.setPosition = True
            pass
        elif button.GetLabel()=='Load Pose':
            dialog = wx.FileDialog ( None, message = 'Choose pose file', defaultDir='/home/chris/ROS/schunk/poses', style = wx.FD_OPEN )

            # Show the dialog and get user input
            if dialog.ShowModal() == wx.ID_OK:
                print 'Loading:', dialog.GetPath()
                f = open(dialog.GetPath(), 'r')
                for name,line in zip(roscomms.joint_list,f):
                    print name
                    print line
                    joint = self.joint_map[name]['joint']
                    self.joint_map[name]['slidervalue'] = self.valueToSlider(float(line), joint)
                f.close()
                self.update_values()

            # The user did not select anything
            else:
                print 'Load aborted.'
            
            # Destroy the dialog
            
            dialog.Destroy()
            pass
        elif button.GetLabel()=='Save Pose':
            # Create a save file dialog
            dialog = wx.FileDialog ( None, style = wx.FD_SAVE | wx.FD_OVERWRITE_PROMPT, defaultDir='/home/chris/ROS/schunk/poses' )
            
            # Show the dialog and get user input
            if dialog.ShowModal() == wx.ID_OK:
                print 'Saving:', dialog.GetPath()
                f = open(dialog.GetPath(), 'w')
                for name in roscomms.joint_list:
                    f.write('%.5f\n'%self.joint_map[name]['joint']['value'])
                f.close()

            # The user did not select anything
            else:
                print 'Save aborted'
            
            # Destroy the dialog
            dialog.Destroy()
            pass

        

    def center(self):
        rospy.loginfo("Centering")
        for (name,joint_info) in self.joint_map.items():
            joint = joint_info['joint']
            joint_info['slidervalue'] = self.valueToSlider(joint['zero'], joint)
        self.update_values()

    def sliderUpdate(self, event):
	slider = event.GetEventObject()
	if slider.isvel:
	    for (name,joint_info) in self.joint_map.items():
		joint_info['slider_velocity_value'] = joint_info['slider_vel'].GetValue()
	    self.update_velocity_values()
	else:
	    for (name,joint_info) in self.joint_map.items():
		joint_info['slidervalue'] = joint_info['slider'].GetValue()
	    self.update_values()

    def valueToSlider(self, value, joint):
        return (value - joint['min']) * float(RANGE) / (joint['max'] - joint['min'])
        
    def sliderToValue(self, slider, joint):
        pctvalue = slider / float(RANGE)
        return joint['min'] + (joint['max']-joint['min']) * pctvalue


if __name__ == '__main__':
    try:
        rospy.init_node('schunk_gui')
        roscomms = RosCommunication()
        app = wx.App()
        gui = SchunkGui("Skunk Control", roscomms)
        gui.Show()
        Thread(target=app.MainLoop).start() # run the gui in a seperate thread
        
        roscomms.loop()
        
    except rospy.ROSInterruptException: pass

