import time
import os
import platform
import sys
import math

import collections
import collections.abc
collections.MutableMapping = collections.abc.MutableMapping

from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative, Command
from pymavlink import mavutil

#############################

targetAltitude = 12
manualArm = False

############ DRONEKIT #################
vehicle = connect('/dev/ttyAMA0', baud=57600, wait_ready=True)

# Select /dev/ttyAMA0 for UART. /dev/ttyACM0 for USB

######### FUNCTIONS ###########
def arm_and_takeoff(targetHeight):
	while not vehicle.is_armable:
		print("Waiting for vehicle to become armable.")
		time.sleep(1)
	print("Vehicle is now armable")

	vehicle.home_location=vehicle.location.global_frame # set to current location
	
	vehicle.mode = VehicleMode("GUIDED")
	
	while vehicle.mode != 'GUIDED':
		print("Waiting for drone to enter GUIDED flight mode")
		time.sleep(1)
	print("Vehicle now in GUIDED MODE. Have fun!!")

	if not manualArm:
		vehicle.armed = True
		while not vehicle.armed:
			print("Waiting for vehicle to become armed.")
			time.sleep(1)
	else:
		if not vehicle.armed:
			print("Exiting script. manualArm set to True but vehicle not armed.")
			print("Set manualArm to False if desiring script to arm the drone.")
			return None
	print("Look out! Props are spinning!!")
	
	vehicle.simple_takeoff(targetHeight)  # meters

	while True:
		print("Current Altitude: %d" % vehicle.location.global_relative_frame.alt)
		if vehicle.location.global_relative_frame.alt >= 0.95 * targetHeight:
			break
		time.sleep(1)
	print("Target altitude reached!!")

	return None

def go_to_location(lat, lon, alt):
	vehicle.simple_goto(LocationGlobalRelative(lat, lon, alt), groundspeed=5)


def set_home():
    vehicle.home_location = vehicle.location.global_frame


############ MAIN ###############

set_home()
arm_and_takeoff(targetAltitude)
print(vehicle.home_location)

# read points file and get the desired coordinates
with open("points.csv", "r") as file:
	points = file.readlines()
	
try:
	for i in range(1, len(points)):
		point, target_lat, target_long, alt = points[i].split(",")
		target_lat = float(target_lat)
		target_long = float(target_long)
		alt = float(alt[:-1])
		print(target_lat, target_long, targetAltitude)
		go_to_location(target_lat, target_long, targetAltitude)
		

		lat, long = vehicle.location.global_frame.lat, vehicle.location.global_frame.lon #, vehicle.location.global_frame.alt
		distance = math.sqrt( ( target_lat-lat ) ** 2 + ( target_long-long ) ** 2 )
		distance_m = 111139*distance
		
		while (distance_m > 1): # distance is greater than 1 meters
			lat, long = vehicle.location.global_frame.lat, vehicle.location.global_frame.lon #, vehicle.location.global_frame.alt
			distance = math.sqrt( ( target_lat-lat ) ** 2 + ( target_long-long ) ** 2 )
			distance_m = 111139*distance
			time.sleep(0.01)
			print(f"Distance to location: {distance_m}")

		set_home()
except Exception as e:
	print(e)

vehicle.mode = VehicleMode("LAND")

while vehicle.mode != 'LAND':
	time.sleep(1)
	print("Waiting for drone to land")
print("Drone in land mode. Exiting script.")