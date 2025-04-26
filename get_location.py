import time
import os
import platform
import sys

import collections
import collections.abc
collections.MutableMapping = collections.abc.MutableMapping

from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative
from pymavlink import mavutil

vehicle = connect('/dev/ttyAMA0', baud=57600, wait_ready=True)

while not vehicle.is_armable:
    print("Waiting for vehicle to become armable.")
    time.sleep(1)
print("Vehicle is now armable")

vehicle.mode = VehicleMode("GUIDED")

while vehicle.mode != 'GUIDED':
    print("Waiting for drone to enter GUIDED flight mode")
    time.sleep(1)
print("Vehicle now in GUIDED MODE. Have fun!!")

with open("points.csv", "w") as file:
    file.write("point,lat,lon,alt\n")

point = 0
while True:
    input("press any key to get and save new coordinate")
    print(point, vehicle.location.global_frame.lat, vehicle.location.global_frame.lon, vehicle.location.global_frame.alt)
    with open("points.csv", "a") as file:
        file.write(f"{point},{vehicle.location.global_frame.lat},{vehicle.location.global_frame.lon},{vehicle.location.global_frame.alt}\n")
    point += 1