from avoid_ultrasonic import avoid
from manual import manual
from pyrealsense_viewer import viewer
# from rowdy_control import rowdycontrol
from tracking import track
import sys

# Version 3/20/2024
# This is the main control for the rover. 
# It presents the options and facilitates the commands.
# Rowdy Rover Capstone Group

UserInput = "what even is materials engineering"

mappingStarted = False

def control_rover():
	global UserInput
	global mappingStarted
	while UserInput != 'Q':
		print("Which would you like to perform:\n1: Manual Control")
		print("2: Follow line\n3: Obstacle Avoidance\n4: Start Mapping\nQ to quit")
		if mappingStarted == True:
			print("5: Stop Mapping")

		UserInput = input("Enter Command:")

		if UserInput == '1':
			print("Starting Manual Control")
			manual()
		elif UserInput == '2':
			print("Starting Line Following")
			dummyinput = input("Enter when rover on line and ready:")
			track()
		elif UserInput == "3":
			print('Starting Obstacle Avoidance')
			avoid()
		elif UserInput == '4':
			mappingStarted = True
			print('Start Mapping')
		elif mappingStarted and UserInput == '5':
			mappingStarted = False
			print('Stopping Mapping')
		elif UserInput == 'Q':
			print("Ending...")
			sys.exit()
		elif UserInput == '99':
			viewer()
		else:
			print("Not a command. Try again.")
		print('')
		print('')

control_rover()