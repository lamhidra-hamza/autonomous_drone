
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

def get_target(target):
	lat = target.x
	lon = target.y
	alt = target.z
	ret = LocationGlobalRelative(lat, lon, alt)
	return ret

def set_other_waypoints(vehicle):
	cmds = vehicle.commands
  	cmds.download()
	cmds.wait_ready()
	#download_mission(vehicle)
	#cmds = vehicle.commands
	missionlst = []
	for wp in cmds:
		missionlst.append(wp)
	dis1 = get_distance_metres(get_target(missionlst[1]), get_target(missionlst[2]))
	dis2 = get_distance_metres(get_target(missionlst[2]), get_target(missionlst[3]))
	#if dis2 >= dis1:
	num = dis2 / 10
	i = 0
	n = 3
	n1 , n2 = 1, 1
	xA = missionlst[1].x
	yA = missionlst[1].y
	xB = missionlst[2].x
	yB = missionlst[2].y
	xC = missionlst[3].x
	yC = missionlst[3].y
	xD = missionlst[4].x
	yD = missionlst[4].y
	disBC = math.sqrt((xC - xB)*(xC - xB) + (yC - yB)*(yC - yB))
	disAD = math.sqrt((xD - xA)*(xD - xA) + (yD - yA)*(yD - yA))
	l1 = 1
	l2 = 0
	while (i / 2 < num - 1):
		if (l1 != 0):
			x = xB - ((n1 * (disBC/num)) * (xB - xC)) / disBC
			y = yB - ((n1 * (disBC/num)) * (yB - yC)) / disBC
			l1 -= 1
			if l1 == 0:
				l2 = 2
			n1 += 1
		elif l2 != 0:
			x = xA - ((n2 * (disAD/num)) * (xA - xD)) / disAD
			y = yA - ((n2 * (disAD/num)) * (yA - yD)) / disAD
			l2 -= 1
			if l2 == 0:
				l1 = 2
			n2 += 1
		wp1 = Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
					0, 0, 0, 0, 0, 0,
					x, y, 10)
		missionlst.insert(n, wp1)
		n += 1
		i += 1
	cmds.clear()
	for wp in missionlst:
		cmds.add(wp)
	cmds.upload()
	time.sleep(5)

def changeMode(vehicle, mode):
	while vehicle.mode != VehicleMode(mode):
		vehicle.mode = VehicleMode(mode)
		time.sleep(0.5)
	return True

def get_distance_metres(aLocation1, aLocation2):
    dlat = aLocation2.lat - aLocation1.lat
    dlong = aLocation2.lon - aLocation1.lon
    return math.sqrt((dlat*dlat) + (dlong*dlong)) * 1.113195e5


speed = 5
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
		set_other_waypoints(vehicle)
		#add_last_waypoint(vehicle, vehicle.location.global_relative_frame.lat, 
         #                              vehicle.location.global_relative_frame.lon, 
          #                             vehicle.location.global_relative_frame.alt)
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
		
