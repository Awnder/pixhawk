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

targetAltitude = 3
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

############ MAIN ###############


arm_and_takeoff(targetAltitude)

# read points file and get the desired coordinates
with open("points.csv", "r") as file:
	points = file.readlines()
	
for i in range(1, len(points)):
    point, lat, lon, alt = points[i].split(",")
    lat = float(lat)
    lon = float(lon)
    alt = float(alt[:-1])
    print(lat, lon, targetAltitude)
    go_to_location(lat, lon, targetAltitude)
    time.sleep(2)

vehicle.mode = VehicleMode("LAND")

while vehicle.mode != 'LAND':
	time.sleep(1)
	print("Waiting for drone to land")
print("Drone in land mode. Exiting script.")
