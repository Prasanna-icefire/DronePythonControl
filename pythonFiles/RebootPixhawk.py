from pymavlink import mavutil
from dronekit import connect, VehicleMode
import time
connection_string = '/dev/ttyS0'
print("Connecting to vehicle on: %s" % (connection_string,))
vehicle = connect(connection_string, wait_ready=True,baud=57600)
print ("Connected and now rebooting....!")
vehicle.reboot()
time.sleep(15)
print("Reconnecting")
vehicle = connect(connection_string, wait_ready=True,baud=57600)
print ("Connected!")