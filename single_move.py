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

targetAltitude = 5
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

def get_location_metres(original_location, dNorth, dEast):
    """
    Returns a LocationGlobal object containing the latitude/longitude `dNorth` and `dEast` metres from the 
    specified `original_location`. The returned Location has the same `alt` value
    as `original_location`.

    The function is useful when you want to move the vehicle around specifying locations relative to 
    the current vehicle position.
    The algorithm is relatively accurate over small distances (10m within 1km) except close to the poles.
    For more information see:
    http://gis.stackexchange.com/questions/2951/algorithm-for-offsetting-a-latitude-longitude-by-some-amount-of-meters
    """
    earth_radius=6378137.0 #Radius of "spherical" earth
    #Coordinate offsets in radians
    dLat = dNorth/earth_radius
    dLon = dEast/(earth_radius*math.cos(math.pi*original_location.lat/180))

    #New position in decimal degrees
    newlat = original_location.lat + (dLat * 180/math.pi)
    newlon = original_location.lon + (dLon * 180/math.pi)
    return LocationGlobal(newlat, newlon,original_location.alt)

def get_distance_metres(aLocation1, aLocation2):
    """
    Returns the ground distance in metres between two LocationGlobal objects.

    This method is an approximation, and will not be accurate over large distances and close to the 
    earth's poles. It comes from the ArduPilot test code: 
    https://github.com/diydrones/ardupilot/blob/master/Tools/autotest/common.py
    """
    dlat = aLocation2.lat - aLocation1.lat
    dlong = aLocation2.lon - aLocation1.lon
    return math.sqrt((dlat*dlat) + (dlong*dlong)) * 1.113195e5

def distance_to_current_waypoint():
    """
    Gets distance in metres to the current waypoint. 
    It returns None for the first waypoint (Home location).
    """
    nextwaypoint = vehicle.commands.next
    if nextwaypoint==0:
        return None
    missionitem=vehicle.commands[nextwaypoint-1] #commands are zero indexed
    lat = missionitem.x
    lon = missionitem.y
    alt = missionitem.z
    targetWaypointLocation = LocationGlobalRelative(lat,lon,alt)
    distancetopoint = get_distance_metres(vehicle.location.global_frame, targetWaypointLocation)
    return distancetopoint

def download_mission():
    """
    Download the current mission from the vehicle.
    """
    cmds = vehicle.commands
    cmds.download()
    cmds.wait_ready() # wait until download is complete.

def go_to_location():
	point0 = LocationGlobalRelative(vehicle.location.global_frame.lat, vehicle.location.global_frame.lon, vehicle.location.global_frame.alt)
	# point1 = LocationGlobalRelative(33.6540405, -117.8118813, targetAltitude)
	# vehicle.simple_goto(point1, groundspeed=5)
	print(vehicle.location.global_frame.lat, vehicle.location.global_frame.lon, vehicle.location.global_frame.alt)
	time.sleep(10)

############ MAIN ###############


arm_and_takeoff(targetAltitude)
go_to_location()

vehicle.mode = VehicleMode("LAND")


while vehicle.mode != 'LAND':
	time.sleep(1)
	print("Waiting for drone to land")
print("Drone in land mode. Exiting script.")
