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
#        import roslib; roslib.load_manifest('schunk_gui')
#        import rospy
#        from std_msgs.msg import *
except:
        print "One (or more) of the dependencies is not satisfied"
        sys.exit(1)


class SchunkTextControl:
        def __init__(self):
		argc = len(sys.argv)
		argv = sys.argv

		self.historyLength = 100

		self.numModules = 7
		self.pose = []
		self.modules_maxlimits = []
		self.modules_minlimits = []
		for i in range(0,self.numModules):
			self.pose.append(0)
			self.modules_maxlimits.append(90+i)
			self.modules_minlimits.append(-90-i)
		self.limitsStrings = self.get_limits_strings() # limits as string: "min to max"

                self.wTree = gtk.glade.XML("schunk_text_control.glade", "window1")
		self.commandWidget = self.wTree.get_widget("command")
                bindings = {"on_window1_destroy":self.shutdown, 
			    "on_buttonClear_clicked":self.clear, 
			    "on_buttonExecute_clicked":self.execute, 
			    "on_command_changed":self.command_changed,
			    "on_tbHelp_toggled":self.tb_help,
			    "on_command_move_active":self.history, 
			    "on_buttonSavePose_clicked":self.save_pose,
			    "on_buttonLoadPose_clicked":self.load_pose }
                self.wTree.signal_autoconnect(bindings)

		self.history = gtk.ListStore(gobject.TYPE_STRING)
		self.historyCounter = 0
		self.history_append("")
		self.load_history()
		self.commandWidget.set_model(self.history)
		self.commandWidget.set_text_column(0)
		self.commandWidget.set_active(self.historyCounter-1)

		self.completion = gtk.EntryCompletion()
		self.vocabulary = gtk.ListStore(gobject.TYPE_STRING)
		self.words = ["help", "info", "ack", "ref", "move", "curmax", "save", "load", "vel", "setvel", "setcur" ]
		self.vocabulary = self.add_words(self.words)
		self.completion.set_model(self.vocabulary)
		self.completion.set_minimum_key_length(1)
		self.completion.set_text_column(0)
		self.commandWidget.child.set_completion(self.completion)

		self.wTree.get_widget("labelHelp").set_text(str(self.words))

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
			spinButton.set_range(-90, 90)
			spinButton.set_increments(1, 5)
			self.posesframe_spinButtons.append(spinButton)
			vbox.add(label)
			vbox.add(spinButton)
			posesframe_hbox.add(vbox)
		self.wTree.get_widget("posesFrame").add(posesframe_hbox)
		self.wTree.get_widget("posesFrame").show_all()
		
		velframe_hbox = gtk.HBox(False, 6)
		velframe_vboxes = []
		velframe_labels = []
		self.velframe_spinButtons = []
		for i in range (0,self.numModules):
			#name = "joint " + str(i) + ":"
			name = str(i) + ":"
			vbox = gtk.VBox(False, 0)
			velframe_vboxes.append(vbox)
			label = gtk.Label(name)
			velframe_labels.append(label)
			spinButton = gtk.SpinButton(digits=2)
			spinButton.set_range(-0.5, 0.5)
			spinButton.set_increments(1, 5)
			self.velframe_spinButtons.append(spinButton)
			vbox.add(label)
			vbox.add(spinButton)
			velframe_hbox.add(vbox)
		self.wTree.get_widget("velFrame").add(velframe_hbox)
		self.wTree.get_widget("velFrame").show_all()

		if argc > 1:
			if (argv[1] == "medium") or (argv[1] == "mini"):
				self.wTree.get_widget("aPoseFrame").hide()
				self.wTree.get_widget("aVelFrame").hide()
			if argv[1] == "mini":
				self.wTree.get_widget("aFlagsFrame").hide()
		w = self.wTree.get_widget("window1")
		w.resize(*w.size_request())




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


        def shutdown(self, widget):
                gtk.main_quit()

	def clear(self, widget):
		self.commandWidget.set_active(0)

	def execute(self, widget):
		commandStr = self.commandWidget.get_active_text()
		if commandStr != "":
			#iterator = self.store.get_iter_from_string("")
			#self.store.insert_before(iterator, [commandStr])
			self.history_append(commandStr)
			self.parser(commandStr)
			self.commandWidget.set_active(self.historyCounter-1)

	def add_words(self, words):
		vocabulary = gtk.ListStore(gobject.TYPE_STRING)
		for word in words:
			vocabulary.append([word])
		return vocabulary

	def command_changed(self, widget):
		tokens = widget.get_active_text().split()
		try:
			if (tokens[0] == "move"):
				try:
					module = int(tokens[1])
					try:
						string = "Limit: " + self.limitsStrings[module]
						self.wTree.get_widget("status").set_text(string)
					except:
						self.wTree.get_widget("status").set_text("ERROR: module not found")
				except:
					self.wTree.get_widget("status").set_text("")
			else:
				try:
					module = int(tokens[1])
					if (module >= self.numModules) or (module < 0):
						self.wTree.get_widget("status").set_text("ERROR: module not found")
				except:
					self.wTree.get_widget("status").set_text("")
		except:
			pass




	def parser(self, string):
		tokens = string.split()
		if tokens[0] == "ack":
			self.ack(tokens)
		elif tokens[0] == "move":
			self.move(tokens)
		elif tokens[0] == "help":
			self.help()
		else:
			self.command_not_found()

	def tb_help(self, widget):
		label = self.wTree.get_widget("labelHelp")
		if widget.get_active():
			label.show()
		else:
			label.hide()

	def help(self):
		self.wTree.get_widget("tbHelp").set_active(True)
		#tview = self.wTree.get_widget("out")
		#buf = tview.get_buffer()
		#itera = buf.get_end_iter()
		#string = "Available commands: " + str(self.words)
		#buf.insert(itera, string)
		#tview.set_buffer(buf)
		#tview.scroll_to_mark(buf.get_insert(), 0.0)
		
	def update_pose_display(self):
		for i in range(0,len(self.pose)):
			try:
				self.posesframe_spinButtons[i].set_value(self.pose[i])
			except:
				print "error message on " + str(i)
				return
		pass


	def ack(self, tokens):
		print "ack"
		try:
			module = tokens[1]
			value = tokens[2]
			if module >= 0:
				pass # ack this module
			else:
				pass # ack all modules

		except:
			print "error ack"


	def move(self, tokens):
		print "move"
		try:
			module = tokens[1]
			value = tokens[2]
			if module >= 0:
				pass # move this module
			else:
				pass # move all modules
		except:
			print "error move"


	def command_not_found(self):
		print "command not found"

	def get_limits_strings(self):
		limitsStrings = {}
		for i in range(0,self.numModules):
			min = self.modules_minlimits[i]
			max = self.modules_maxlimits[i]
			string = str(min) + " to " + str(max)
			limitsStrings[i] = string
		return limitsStrings


if __name__ == "__main__":
	gui = SchunkTextControl()
        gtk.main()
