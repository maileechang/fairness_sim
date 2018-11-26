from hlpr_manipulation_utils.arm_moveit2 import ArmMoveIt
from std_msgs.msg import String
from kt_files import location_controller
import rospy
import sys
import time
import moveit_commander
import signal

def signal_handler(sig, frame):
	print "\nCleaning up..."
	rospy.signal_shutdown("Ctrl+C was pressed")
	sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)

kt_dict = {}
for i in range(20):
	kt_dict.update({str(i) : chr(97+i)}) #97 is the ASCII value of "a", this just maps buttons to their locations
bin_dict = {"red" : "right", "blue" : "left"}

pub = rospy.Publisher('/mdp_cleaning/arm_status', String, queue_size=10)

if __name__ == "__main__":
	rospy.init_node('mdp_arm_controller', anonymous=True)
	## First initialize moveit_commander and rospy.
	print "Initializing arm..."
	moveit_commander.roscpp_initialize(sys.argv)

	arm =  ArmMoveIt()

	while not rospy.is_shutdown():
		print "Waiting for a message..."
		msg = rospy.wait_for_message('/mdp_cleaning/arm_commands', String)
		while pub.get_num_connections() == 0:
			rospy.sleep(0.1)
		pub.publish("Working")
		msg_content = msg.data.split(",") #msgs will be in the format button#,bin# e.g. "0,0"
		if msg_content[0] not in [str(i) for i in range(20)]:
			print "Bad message data:", msg.data
			continue
		else:
			while pub.get_num_connections() == 0:
				rospy.sleep(0.1)
			pub.publish("Working")
			result = False
			result = location_controller.execute(arm, kt_dict[msg_content[0]], bin_dict[msg_content[1]])
		if result:
			print "Publishing \"Done\""
			while pub.get_num_connections() == 0:
				rospy.sleep(0.1)
			pub.publish("Done")
