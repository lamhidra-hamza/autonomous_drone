
import time
from dronekit import connect, VehicleMode, LocationGlobalRelative, Command, LocationGlobal, APIException
from pymavlink import mavutil
import socket
#import exeptions
import math
import argparse

def connectMyCopter():
	parser = argparse.ArgumentParser(description='commands')
	parser.add_argument('--connect')
	args = parser.parse_args()

	connection_string = args.connect
	
	if not connection_string:
		import dronekit_sitl
		sitl = dronekit_sitl.start_default()
		connection_string = sitl.connection_string()

	vehicle = connect(connection_string, wait_ready=True)
	vehicle.wait_ready(True, raise_exception=False)

	return vehicle


def arm_and_takeoff(altitude):
	while not vehicle.is_armable:
		print ("Waiting for vehicle to initialise...")
		time.sleep(1)
	print ("Arming motors")
	vehicle.mode = VehicleMode("GUIDED")
	vehicle.armed = True
	while not vehicle.armed:
		print (" Waiting for arming ...")
		time.sleep(1) 

	print( "Taking off!")
	vehicle.simple_takeoff(altitude)
	while True:
		print (" Altitude: ", vehicle.location.global_relative_frame.alt)
		#Break and return from function just below target altitude.
		if vehicle.location.global_relative_frame.alt>=altitude*0.95:
			print ("Reached target altitude")
			break
		time.sleep(1)

def download_mission(vehicle):
	cmds = vehicle.commands
	cmds.download()
	cmds.wait_ready()

def clear_mission(vehicle):
	cmds = vehicle.commands
	cmds.clear()
	cmds.upload()
	download_mission(vehicle)

def add_mission(vehicle):
	missionlst  = []
	n_wp        = 0

	print("download the mission")
	download_mission(vehicle)
	for wp in vehicle.commands:
		missionlst.append(wp)
		n_wp += 1
	return n_wp, missionlst

def add_last_waypoint(vehicle, lat, long, alt):
	download_mission(vehicle)
	cmds = vehicle.commands
	missionlst = []
	for wp in cmds:
		print("wayponit heeere=>")
		print(wp)
		missionlst.append(wp)
	wp_last = Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
					0, 0, 0, 0, 0, 0,
					lat, long, alt)
	missionlst.append(wp_last)

	cmds.clear()

	for wp in missionlst:
		cmds.add(wp)
	cmds.upload()

	return (cmds.count)

def changeMode(vehicle, mode):
	while vehicle.mode != VehicleMode(mode):
		vehicle.mode = VehicleMode(mode)
		time.sleep(0.5)
	return True


speed = 10
mode = 'ground'

vehicle = connectMyCopter()

while True:
	if mode == 'ground':
		n_wp, missionlst = add_mission(vehicle)
		time.sleep(2)
		
		if (n_wp > 0):
			print("there is a mission, lets taking off")
			mode = 'takeoff'
	
	elif mode == 'takeoff':
		add_last_waypoint(vehicle, vehicle.location.global_relative_frame.lat, 
                                       vehicle.location.global_relative_frame.lon, 
                                       vehicle.location.global_relative_frame.alt)

		print("final waypoint added to mission")
		time.sleep(1)
		arm_and_takeoff(10)
		changeMode(vehicle,"AUTO")
		vehicle.groundspeed = speed
		mode = 'mission'
		print("switch to mission mode")
	elif mode == 'mission':
		print("current wp: %d of %d " % (vehicle.commands.next, vehicle.commands.count))
		if (vehicle.commands.next == vehicle.commands.count):
			print ("final wp reached: go back home")
			clear_mission(vehicle)
			changeMode(vehicle, "RTL")
			mode = "back"
	elif mode == 'back':
		if (vehicle.location.global_relative_frame.alt < 1.0):
			print ("ground vehicle")
			mode = 'ground'
	time.sleep(0.5)
		
