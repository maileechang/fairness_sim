from mttkinter import mtTkinter as tk
from datetime import datetime
from threading import Thread
import rospy
from std_msgs.msg import String
from std_msgs.msg import Float64
import os
import signal
import sys
import csv
import random
import time
import copy

SID = raw_input("What is the user ID?\n")


'''
TODO:
Make sure CSV files are printing correctly
Add arm publisher and subscriber to find out when the robot is done with pickup
'''

def signal_handler(sig, frame):
	print "\nCleaning up..."
	rospy.signal_shutdown("Ctrl+C was pressed")
	table.CSV_file.close()
	sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)
'''
MATH_PROBLEMS = {"EASY" : {"7 - 4 + 5" : 8, "2 + 11 - 5" : 8, "13 + 10 - 3" : 20, "8 - 4 + 3" : 7, "10 - 2 - 5" : 3, "8 + 7 + 1" : 16, "15 + 7 - 4" : 18, "9 - 3 + 4" : 10},
				"MEDIUM" : {"5 * 7 + 3" : 38, "2 * 12 - 4" : 20, "15 - 3 * 4" : 3, "8 + 4 * 6" : 32, "7 * 3 + 9" : 30, "8 * 4 + 7" : 39, "2 * 13 - 11" : 15, "6 * 9 + 5" : 59},
				"HARD": {"36 / 4 - 4": 5, "121 / 11 + 6" : 17, "115 / 5 * 2" : 46, "3 * 28 / 14" : 6, "13 + 56 / 8" : 20, "24 / 8 * 4" : 12, "45 / 9 * 6" : 30, "48 / 12 + 9" : 13}}
'''

MATH_PROBLEMS = None

try:
	ROBOT_MODE = int(sys.argv[2])
except IndexError:
	ROBOT_MODE = 0

TASK_START_TIME = datetime.now().minute*60+datetime.now().second

DIFFICULTY = "MEDIUM"
try:
	DIFFICULTY = str(sys.argv[1])
except IndexError:
	pass

KT_DICT = {str(i) : chr(97+i) for i in range(20)}

ROBOT_MATH_SLEEP_TIME = 2

class Table:
	def get_time(self):
		return datetime.now().minute*60+datetime.now().second - TASK_START_TIME

	def start_human_pickup_timer(self):
		self.human_pickup_timer_on = True
		self.human_pickup_start_time = self.get_time()
		return self.human_pickup_start_time

	def stop_human_pickup_timer(self):
		self.human_pickup_timer_on = False
		self.human_action_done = True #Will be set false by robot later
		self.human_pickup_end_time = self.get_time()
		print "Human pickup time:", self.human_pickup_end_time-self.human_pickup_start_time
		return self.human_pickup_end_time

	def start_human_math_timer(self):
		self.human_math_timer_on = True
		self.human_math_start_time = self.get_time()
		return self.human_math_start_time

	def stop_human_math_timer(self):
		self.human_math_timer_on = False
		self.human_action_done = True #Will be set false by robot later
		self.human_math_end_time = self.get_time()
		print "Human math time:", self.human_math_end_time-self.human_math_start_time
		return self.human_math_end_time

	def button_handler(self, button):
		if self.human_pickup_timer_on:
			end_time = self.stop_human_pickup_timer()
			self.CSV_writer.writerow([SID, "human_policy?", ["LE", "LE+F", "HE", "HE+F", "Test"][ROBOT_MODE], "1_1", end_time, "end_pickup", "human", self.current_button.replace("button","")])
		if not self.current_button: #No current_button has been set yet
			pass
		elif self.current_button == button:
			pass
		elif self.button_dict[self.current_button]["button"]["background"] == 'grey':
			self.button_dict[self.current_button]["button"]["background"] = 'white'
		elif self.button_dict[self.current_button]["button"]["background"] == 'blue':
			self.button_dict[self.current_button]["button"]["background"] = 'dodger blue'
		self.current_button = button
		button_object = self.button_dict[button]["button"]

		self.correct_text.set("")

		#Has math been done?
		if not self.button_dict[button]["math_status"]:
			button_object['background'] = 'grey'
			self.math_text.set(KT_DICT[button.replace("button","")].upper()+": "+self.button_dict[button]["math"].keys()[0])
			self.pickup_button.config(state='disabled')
			self.start_human_math_timer()
			self.CSV_writer.writerow([SID, "human_policy?", ["LE", "LE+F", "HE", "HE+F", "Test"][ROBOT_MODE], "1_1", self.get_time(), "start_math", "human", self.current_button.replace("button","")]) #Log every time the human clicks a button
		#Math has been done, has the item been picked up?
		elif not self.button_dict[button]["pickup_status"]:
			self.button_dict[self.current_button]["button"]["background"] = 'blue'
			self.math_text.set(KT_DICT[self.current_button.replace("button","")].upper())
			self.pickup_button.config(state='normal')

	def human_pickup(self):
		start_time = self.start_human_pickup_timer()
		self.CSV_writer.writerow([SID, "human_policy?", ["LE", "LE+F", "HE", "HE+F", "Test"][ROBOT_MODE], "1_1", start_time, "start_pickup", "human", self.current_button.replace("button","")])
		self.button_dict[self.current_button]["pickup_status"] = True
		self.button_dict[self.current_button]["button"].config(bg='green4',relief='sunken', state='disabled')
		for button in self.button_dict.keys():
			if not self.button_dict[button]["pickup_status"]:
				return
		self.math_text.set("You're done! Press \"Finish and Exit\"")

	def enter_button_handler(self, event=None):
		if not self.current_button:
			self.math_text.set("Please press a button first")
			self.math_input.delete(0,'end')
			return
		if self.button_dict[self.current_button]["math_status"]:
			return #The math is already solved

		if self.math_input.get() == "":
			self.math_text.set(self.button_dict[self.current_button]["math"].keys()[0])
			return

		try:
			answer = int(self.math_input.get().translate(None, " ")) #Takes the input from the entry box, strips away any spaces, and converts it into an integer
			self.math_input.delete(0,'end')
		except ValueError:
			self.math_text.set("Answer must be a number") #Means the input wasn't an integer
			self.math_input.delete(0,'end')
			return

		if answer == self.button_dict[self.current_button]["math"].values()[0] or answer == 1: #The answer was correct
			self.button_dict[self.current_button]["button"].config(bg='dodger blue')
			self.correct_text.set("Correct")
			self.math_text.set("")
			self.correct_label.config(fg='green')
			self.button_dict[self.current_button]["math_status"] = True
			bin_number = self.bin_numbers.pop(0)
			self.button_dict[self.current_button]["bin_number"] = bin_number
			self.button_dict[self.current_button]["button"].config(text=KT_DICT[self.current_button.replace("button","")].upper() + "\n"+str(bin_number), font=('Arial', 16), height=4, width=7)
			self.pickup_button.config(state='normal')

			end_time = self.stop_human_math_timer()
			self.CSV_writer.writerow([SID, "human_policy?", ["LE", "LE+F", "HE", "HE+F", "Test"][ROBOT_MODE], "1_1", end_time, "end_math", "human", self.current_button.replace("button","")])
		else:
			self.CSV_writer.writerow([SID, "human_policy?", ["LE", "LE+F", "HE", "HE+F", "Test"][ROBOT_MODE], "1_1", self.get_time(), "incorrect_math", "human", self.current_button.replace("button","")])
			self.correct_text.set("Incorrect")
			self.correct_label.config(fg='red')

	def robot_math_claim(self, button=None):
		if button == self.current_button:
			return False
		elif self.button_dict[button]["math_status"]:
			return False
		elif self.button_dict[button]["button"]['state'] == 'disabled':
			return False
		#The current button isn't claimed, the math isn't done, and the button isn't disabled at this point
		self.button_dict[button]["button"].config(height=100, width=100, image=self.robot_photo, state='disabled') #Place the robot face
		return True

	def robot_math_finish(self, button=None):
		self.button_dict[button]["button"].config(bg='dodger blue', image="", state='normal')
		self.button_dict[button]["math_status"] = True
		bin_number = self.bin_numbers.pop(0)
		self.button_dict[button]["bin_number"] = bin_number
		self.button_dict[button]["button"].config(text=KT_DICT[button.replace("button","")].upper() + "\n"+str(bin_number), font=('Arial', 16), height=4, width=7)
		return True


	def robot_pickup_claim(self, button=None):
		if button == self.current_button:
			return False
		elif self.button_dict[button]["pickup_status"]:
			return False
		elif self.button_dict[button]["button"]['state'] == 'disabled':
			return False
		elif not self.button_dict[button]["math_status"]:
			return False
		#The current button isn't claimed, the item hasn't been picked up, and the button isn't disabled at this point
		self.button_dict[button]["button"].config(height=100, width=100, image=self.robot_photo, state='disabled') #Place the robot face
		return True

	#Used for after the robot returns from the arm controller
	def robot_pickup_finish(self, button=None):
		print "Setting picked up button to green..."
		self.button_dict[button]["pickup_status"] = True
		self.button_dict[button]["button"].config(bg='green4',relief='sunken', image="", state='disabled', height=4, width=7)
		return True

	def end_task(self):
		if self.human_pickup_start_time:
			end_time = self.stop_human_pickup_timer()
			self.CSV_writer.writerow([SID, "human_policy?", ["LE", "LE+F", "HE", "HE+F", "Test"][ROBOT_MODE], "1_1", end_time, "end_pickup", "human", self.current_button.replace("button","")])
		signal_handler(signal.SIGINT, None)

	def math_generator(self):
		global MATH_PROBLEMS
		MATH_PROBLEMS = {"EASY" : math_sets.get_easy(), "MEDIUM" : math_sets.get_medium(), "HARD" : math_sets.get_hard()}
		maths = [{k : v} for (k,v) in MATH_PROBLEMS[self.difficulty].iteritems()]
		random.shuffle(maths)
		for key in self.button_dict.keys():
			try:
				self.button_dict[key].update({"math" : maths.pop(0)})
			except IndexError:
				self.button_dict[key].update({"math" : {key[-1]:0}})

	def __init__(self):
		LOG_PATH = "./csv_files/GUI/"
		if os.path.isfile(LOG_PATH+"SID_"+SID+"_"+str(ROBOT_MODE)+".csv"):
			print "That filename is already in use"
			print "Either delete the old SID_"+SID+"_"+str(ROBOT_MODE)+".csv or use a different filename"
			signal_handler(signal.SIGINT, None)
		self.CSV_file = open(LOG_PATH +"SID_"+SID+"_"+str(ROBOT_MODE)+".csv", "w")
		self.CSV_writer = csv.writer(self.CSV_file, dialect='excel', lineterminator='\n')
		self.CSV_writer.writerow(["SID", "human_policy", "robot_policy", "math_bin_ratio", "time", "task", "agent", "item",None,"Human to robot ratio:", "=(COUNTIF(G2:G100, \"human\")/COUNTIF(G2:G100,\"robot\"))"])

		self.human_action_done = False

		self.human_math_timer_on = False
		self.human_pickup_timer_on = False
		self.human_pickup_start_time = None
		self.human_pickup_end_time = None
		self.human_math_start_time = None
		self.human_math_end_time = None

		self.root = tk.Tk()
		self.root.attributes('-zoomed', True)
		self.root.minsize(1100,550)
		self.root.bind('<KP_Enter>', self.enter_button_handler)
		self.root.bind('<Return>', self.enter_button_handler) #When you press enter, the label_button_handler runs


		#Location = x*10px
		#button_locations = [[20,20],[400,20],[140, 140],[280, 140],[140,260],[280,260],[20,380],[400,380]]
		button_locations = [[20,20],[140,20],[260,20],[380,20],[500,20],
							[20, 140],[140,140],[260,140],[380,140],[500,140],
							[20, 260],[140,260],[260,260],[380,260],[500,260],
							[20, 380],[140,380],[260,380],[380,380],[500,380]]
		self.button_dict = {"button"+str(i) : {} for i in xrange(len(button_locations))}
		self.bin_numbers = ["red","blue"]*(len(button_locations)/2)
		for i in range(len(button_locations)):
			button = tk.Button(self.root, text=KT_DICT[str(i)].upper(), font=('Arial', 16), bg='white', command=lambda x=str(i): self.button_handler("button"+x))
			button.config(height=4, width=7)
			button.place(x=button_locations[i][0], y=button_locations[i][1])

			self.button_dict["button"+str(i)].update({"button" : button})
			self.button_dict["button"+str(i)].update({"math_status" : False}) #True if solved, false if unsolved
			self.button_dict["button"+str(i)].update({"pickup_status" : False}) #True if picked up, false if not
			self.button_dict["button"+str(i)].update({"bin_number" : None})

		self.current_button = None

		self.robot_photo=tk.PhotoImage(file="robot_face.gif")

		#Create button for users to press when picking up
		self.pickup_button = tk.Button(self.root, command=self.human_pickup, text="Pick up", bg='green', height=2, width=8, state='disabled')
		self.pickup_button.place(x=697+120, y=180)

		#Create text box for math
		self.math_text = tk.StringVar()
		self.math_label = tk.Label(self.root, textvariable=self.math_text, font=('Arial', 18), justify=tk.CENTER, height=2, width=30, bg='white')
		self.math_label.place(x=550+120,y=20)

		#Create a box for users to input their answers
		self.math_input = tk.Entry(self.root)
		self.math_input.place(x=660+120, y=80)

		#Create a button for users to press when they submit math
		self.enter_button = tk.Button(self.root, command=self.enter_button_handler, text="Enter", bg='green')
		self.enter_button.config(height=1, width=10)
		self.enter_button.place(x=690+120, y=105)

		#The text that's displayed when the human is right or wrong after a math problem
		self.correct_text = tk.StringVar()
		self.correct_label = tk.Label(self.root, textvariable=self.correct_text, font=('Arial', 16), height=2, width=15, justify=tk.CENTER)
		self.correct_label.place(x=652+120, y=130)

		#The button the user presses when they're done with the task
		self.finish_button = tk.Button(self.root, command=self.end_task, text = "Finish and\nExit", font=('Arial', 12), bg = 'red', fg='gray80', height=4, width=10)
		self.finish_button.place(x=970,y=450)

		self.difficulty = DIFFICULTY
		self.is_running = False


	def start(self):
		print "Tk running..."
		self.is_running = True
		self.math_generator()
		self.root.mainloop()

'''Robot modes:
0: LE
1: LE+F
2: HE
3: HE+F
'''

class Robot(Thread):
	def __init__(self, table):
		super(Robot, self).__init__()
		self.table = table
		self.daemon = True
		self.math_buttons = copy.deepcopy(self.table.button_dict.keys())
		self.pickup_buttons = copy.deepcopy(self.math_buttons)
		random.shuffle(self.math_buttons)
		random.shuffle(self.pickup_buttons)

		self.robot_pickup_start_time = -4 #The robot needs to start pickup right away
		self.pickup_claim_done = True
		self.pickup_waiting = False
		self.current_pickup_button = None
		self.arm_subscriber = rospy.Subscriber("/mdp_cleaning/arm_status", String, self.arm_listener)
		self.arm_publisher = rospy.Publisher("/mdp_cleaning/arm_commands", String, queue_size=3)
		self.tilt_controller = rospy.Publisher("/tilt_controller/command", Float64, queue_size=1)
		self.pan_controller = rospy.Publisher("/pan_controller/command", Float64, queue_size=1)
		self.pan_LR = True #Whether to pan left or right during HE/HE+F
		self.arm_done_flag = False
		self.mode = ROBOT_MODE

	def arm_listener(self, data):
		if data.data == "Done":
			print "Got \"Done\""
			self.arm_done_flag = True
		else:
			print "Got \"Working\""
			self.arm_done_flag = False

	def start_robot_pickup_timer(self):
		self.robot_pickup_timer_on = True
		self.robot_pickup_start_time = self.table.get_time()

	def stop_robot_pickup_timer(self):
		self.robot_pickup_timer_on = False
		self.robot_pickup_end_time = self.table.get_time()
		return self.robot_pickup_end_time

	def start_robot_math_timer(self):
		self.robot_math_timer_on = True
		self.robot_math_start_time = self.table.get_time()

	def stop_robot_math_timer(self):
		self.robot_math_timer_on = False
		self.robot_math_end_time = self.table.get_time()
		return self.robot_math_end_time

	def execute_math(self, button):
		self.start_robot_math_timer()
		if not self.table.robot_math_claim(button):
			if not self.table.button_dict[button]["math_status"] and button not in self.math_buttons: #If the math hasn't been done on the button but it's still unable to be claimed, look at it later
				self.math_buttons.append(button)
			return False
		else:
			self.table.CSV_writer.writerow([SID, "human_policy?", ["LE", "LE+F", "HE", "HE+F", "Test"][ROBOT_MODE], "1_1", self.robot_math_start_time, "start_math", "robot", button.replace("button","")])
			time.sleep(ROBOT_MATH_SLEEP_TIME)
			self.table.robot_math_finish(button)
			self.stop_robot_math_timer()
			self.table.CSV_writer.writerow([SID, "human_policy?", ["LE", "LE+F", "HE", "HE+F", "Test"][ROBOT_MODE], "1_1", self.robot_math_end_time, "end_math", "robot", button.replace("button","")])
			return True

	def execute_pickup(self, button, step=0):
		if step == 1 or step == 0:
			self.start_robot_pickup_timer()
			result = self.table.robot_pickup_claim(button)
			if result:
				self.table.CSV_writer.writerow([SID, "human_policy?", ["LE", "LE+F", "HE", "HE+F", "Test"][ROBOT_MODE], "1_1", self.robot_pickup_start_time, "start_pickup", "robot", button.replace("button","")])
				self.arm_done_flag = False
				print "Waiting for connection..."
				while self.arm_publisher.get_num_connections() == 0:
					rospy.sleep(0.1)
				self.arm_publisher.publish(button.replace("button", "") + "," + str(self.table.button_dict[button]["bin_number"]))
				print "Publishing "+button.replace("button", "") + "," + str(self.table.button_dict[button]["bin_number"])
				while self.pan_controller.get_num_connections() == 0:
					rospy.sleep(0.1)
				if self.table.button_dict[button]["bin_number"] == "blue":
					self.pan_controller.publish(-0.6)
				elif self.table.button_dict[button]["bin_number"] == "red":
					self.pan_controller.publish(0.6)
				else:
					self.pan_controller.publish(0.0)
				return True
			if not result:
				self.stop_robot_pickup_timer()
				return False
			elif step != 0:
				return True
		elif step == 2 or step == 0:
			self.stop_robot_pickup_timer()
			self.table.CSV_writer.writerow([SID, "human_policy?", ["LE", "LE+F", "HE", "HE+F", "Test"][ROBOT_MODE], "1_1", self.robot_pickup_end_time, "end_pickup", "robot", button.replace("button","")])
			self.table.robot_pickup_finish(button)
			return True

	def run(self):
		while not self.table.is_running:
			time.sleep(0.1)

		if self.mode == 0: #LE
			self.pan_controller.publish(0.8)
			while self.math_buttons:
				random.shuffle(self.math_buttons)
				button = self.math_buttons[0]
				result = self.execute_math(button)
				if result or self.table.button_dict[button]["math_status"]:
					self.math_buttons.remove(button)
				'''
				if result:
					print "Robot did math on", button
				else:
					print "Robot failed math on", button
				'''

		elif self.mode == 1: #LE+F
			self.pan_controller.publish(0.8)
			while not self.table.human_action_done:
				time.sleep(0.5)
			while self.math_buttons:
				while not self.table.human_action_done:
					if not self.math_buttons:
						break
					time.sleep(0.1)
				if self.math_buttons:
					self.table.human_action_done = False
				result = False
				while not result and self.math_buttons:
					random.shuffle(self.math_buttons)
					button = self.math_buttons[0]
					result = self.execute_math(button)
					if result or self.table.button_dict[button]["math_status"]:
						self.math_buttons.remove(button)

			while self.pickup_buttons:
				while not self.table.human_action_done:
					if not self.pickup_buttons:
						return
					time.sleep(0.1)
				self.table.human_action_done = False
				result = False
				while not result and self.pickup_buttons:
					random.shuffle(self.pickup_buttons)
					button = self.pickup_buttons[0]
					result = self.execute_pickup(button, step=1)
					if result:
						if result or self.table.button_dict[button]["pickup_status"]:
							self.pickup_buttons.remove(button)
						while not self.arm_done_flag:
							time.sleep(0.5)
						self.execute_pickup(button, step=2)

		elif self.mode == 2: #HE
			while self.math_buttons or self.pickup_buttons:
				if self.math_buttons:
					print "math_buttons len:", len(self.math_buttons)
					if len(self.math_buttons) > 20:
						print "Something's wrong:"
						print "**************self.math_buttons SHOULD NOT HAVE MORE THAN 20 ITEMS****************************"
						while len(self.math_buttons) > 20:
							print "Please press Ctrl+C and tell Zach about the error"
							time.sleep(1)
					result = False
					while not result and self.math_buttons:
						random.shuffle(self.math_buttons)
						button = self.math_buttons[0]
						result = self.execute_math(button)
						if result or self.table.button_dict[button]["math_status"]:
							if result:
								print "Robot did math on", button
							self.math_buttons.remove(button)
				if not self.pickup_buttons:
					print "***NO MORE PICKUP BUTTONS"

				if self.pickup_buttons:
					result = False
					while not result and self.pickup_buttons:
						random.shuffle(self.pickup_buttons)
						button = self.pickup_buttons[0]
						result = self.execute_pickup(button, step=1)
						if result:
							if result or self.table.button_dict[button]["pickup_status"]:
								self.pickup_buttons.remove(button)
							while not self.arm_done_flag:
								result2 = False
								while not result2 and self.math_buttons:
									button2 = self.math_buttons[0]
									result2 = False
									if self.arm_done_flag:
										break
									else:
										result2 = self.execute_math(button2)
									if result2 or self.table.button_dict[button2]["math_status"]:
										if result2:
											print "Robot did math on", button2
										self.math_buttons.remove(button2)
								time.sleep(0.5)
							self.execute_pickup(button, step=2)
							print "Robot picked up", button, "in", str(self.robot_pickup_end_time-self.robot_pickup_start_time), "seconds"
						elif self.table.button_dict[button]["pickup_status"]:
							self.pickup_buttons.remove(button)
			'''
			while self.math_buttons or self.pickup_buttons:
				for key in self.table.button_dict.keys():
					if self.table.button_dict[key]["pickup_status"] and key in self.pickup_buttons:
						print "Removed", key, "from robot pickup list"
						self.pickup_buttons.remove(key)
				if self.math_buttons:
					print "# of math buttons:", len(self.math_buttons)
					button = self.math_buttons.pop(0)
					math_result = self.execute_math(button)
				if self.pickup_buttons:
					print "# of pickup buttons:", len(self.pickup_buttons)
					pickup_result = False
					while not pickup_result:
						random.shuffle(self.pickup_buttons)
						button2 = self.pickup_buttons.pop(0)
						if not self.table.button_dict[button2]["math_status"]: #If math hasn't been done on this button
							self.pickup_buttons.append(button2)
							continue
						pickup_result = self.execute_pickup(button2, step=1)
					if pickup_result:
						while not self.arm_done_flag:
							if self.math_buttons:
								button = self.math_buttons.pop(0)
								math_result = self.execute_math(button)
							else:
								rospy.sleep(0.1)
						pickup_result = self.execute_pickup(button2, step=2)
					else:
						print button2, "failed"
						if button2 not in self.pickup_buttons and not self.table.button_dict[button2]["pickup_status"]:
							self.pickup_buttons.append(button2)
							random.shuffle(self.pickup_buttons)
			'''

		elif self.mode == 3: #HE+F
			while not self.table.human_action_done:
				time.sleep(0.5)
			while self.math_buttons or self.pickup_buttons:
				if self.math_buttons and self.table.human_action_done:
					self.table.human_action_done = False
					result = False
					while not result and self.math_buttons:
						random.shuffle(self.math_buttons)
						button = self.math_buttons[0]
						result = self.execute_math(button)
						if result or self.table.button_dict[button]["math_status"]:
							if result:
								print "Robot did math on", button
							self.math_buttons.remove(button)
				if self.pickup_buttons:
					result = False
					while not result and self.pickup_buttons:
						random.shuffle(self.pickup_buttons)
						button = self.pickup_buttons[0]
						result = self.execute_pickup(button, step=1)
						if result:
							if result or self.table.button_dict[button]["pickup_status"]:
								self.pickup_buttons.remove(button)
							while not self.arm_done_flag:
								if self.table.human_action_done:
									self.table.human_action_done = False
									result2 = False
									while not result2 and self.math_buttons:
										button2 = self.math_buttons[0]
										result2 = self.execute_math(button2)
										if result2 or self.table.button_dict[button2]["math_status"]:
											self.math_buttons.remove(button2)
								time.sleep(0.5)
							self.execute_pickup(button, step=2)
							print "Robot picked up", button, "in", str(self.robot_pickup_end_time-self.robot_pickup_start_time), "seconds"
						elif self.table.button_dict[button]["pickup_status"]:
							self.pickup_buttons.remove(button)
			'''
			while self.math_buttons or self.pickup_buttons:
				for key in self.table.button_dict.keys():
					if self.table.button_dict[key]["pickup_status"] and key in self.pickup_buttons:
						print "Removed", key, "from robot pickup list"
						self.pickup_buttons.remove(key)

				if self.math_buttons and self.table.human_action_done:
					button = self.math_buttons.pop(0)
					math_result = self.execute_math(button)
					self.table.human_action_done = False

				pickup_result = False
				random.shuffle(self.pickup_buttons)
				timeout_time = self.table.get_time()
				while not pickup_result:
					if self.table.get_time() >= timeout_time+1:
						print "Breaking out of pickup looking..."
						break
					try:
						button2 = self.pickup_buttons.pop(0)
					except IndexError:
						pickup_result = True
						continue
					if not self.table.button_dict[button2]["math_status"]: #If math hasn't been done on this button
						self.pickup_buttons.append(button2)
						time.sleep(0.01)
						continue
					pickup_result = self.execute_pickup(button2, step=1)
					time.sleep(0.05)

				if pickup_result:
					print "Picking up", button2 + "..."
					while not self.arm_done_flag:
						if self.math_buttons and self.table.human_action_done:
							print "Doing math..."
							button = self.math_buttons.pop(0)
							math_result = self.execute_math(button)
							self.table.human_action_done = False
						else:
							time.sleep(0.1)
					self.execute_pickup(button2, step=2)
				time.sleep(0.1)
			'''

		elif self.mode == 4: #Test mode, no robot action
			return
		else:
			print "Not a valid robot mode, exiting..."
			signal_handler(signal.SIGINT, None)
		print "Robot thread done, exiting..."
		while self.pan_controller.get_num_connections() == 0:
			rospy.sleep(0.1)
		self.pan_controller.publish(0.0)

if __name__ == "__main__":
	rospy.init_node("MDP_cleaning", anonymous=True)
	table = Table()
	robot = Robot(table)
	while robot.tilt_controller.get_num_connections() == 0:
		rospy.sleep(0.1)
	robot.tilt_controller.publish(0.55)
	if ROBOT_MODE == 0:
		import math_conditions1 as math_sets
	elif ROBOT_MODE == 1:
		import math_conditions2 as math_sets
	elif ROBOT_MODE == 2:
		import math_conditions3 as math_sets
	elif ROBOT_MODE == 3:
		import math_conditions4 as math_sets
	elif ROBOT_MODE == 4:
		import math_test_set as math_sets
	else:
		print "Invalid robot mode", ROBOT_MODE
		sys.exit(-1)
	robot.start()
	table.start()
