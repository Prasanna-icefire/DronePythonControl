from pymavlink import mavutil
from dronekit import connect, VehicleMode
import time
import RPi.GPIO as GPIO
import os
from gtts import gTTS 
language = 'en'
duration=0.7  #duration set for mobility

def speak(msg):
    myobj = gTTS(text=msg, lang=language, slow=False) 
    myobj.save("welcome.mp3") 
    os.system("mpg321 welcome.mp3")

def getDistance(a):
    GPIO.setup(a[0], GPIO.OUT)
    GPIO.setup(a[1], GPIO.IN)
    dist=distance(a[0],a[1])
    time.sleep(0.5) 
    return dist    
   
def distance(GPIO_TRIGGER,GPIO_ECHO):

    time.sleep(0.2)
    GPIO.output(GPIO_TRIGGER, True)
    time.sleep(0.00001)
    GPIO.output(GPIO_TRIGGER, False)
    StartTime = time.time()
    StopTime = time.time()
    while GPIO.input(GPIO_ECHO) == 0:
        StartTime = time.time()
    while GPIO.input(GPIO_ECHO) == 1:
        StopTime = time.time()
    TimeElapsed = StopTime - StartTime
    distance = (TimeElapsed * 34300) / 2
    return distance

def arm_and_takeoff(aTargetAltitude):
    df = getDistance(front)
    dl = getDistance(left)
    dr = getDistance(right)
    print("DistanceFront ",df,"DistanceLeft ",dl," DistanceRight ",dr)
    if df > minDist and dl > minDist and dr > minDist :
        print("Executing TakeOff...")
        time.sleep(1)
        print ("Arming motors")
        # Copter should arm in GUIDED mode
        vehicle.mode    = VehicleMode("GUIDED")
        vehicle.armed   = True
        # Confirm vehicle armed before attempting to take off
        while not vehicle.armed:
            print (" Arming Failed")
            speak("Arming Failure")
            vehicle.armed   = True
            time.sleep(0.5)
            print("Arming Re-ried")
            time.sleep(2)

        print ("Taking off!")
        vehicle.simple_takeoff(aTargetAltitude) # Take off to target altitude

        # Wait until the vehicle reaches a safe height before processing the goto (otherwise the command
        #  after Vehicle.simple_takeoff will execute immediately).
        while getDistance(front) > minDist:
            print (" Altitude: ", vehicle.location.global_relative_frame.alt)
            if(vehicle.armed == False):
                    print("Vehicle Disarmed")
                    exit()
            #Break and return from function just below target altitude.
            if vehicle.location.global_relative_frame.alt>=aTargetAltitude*0.95:
                print ("Reached target altitude")
                time.sleep(10)
                break
            time.sleep(1)
    else:
        print("TakeOff Failure, insufficient Space")  
        speak("TakeOff Failure, insufficient Space")       

def set_velocity_body(Vx, Vy, Vz):
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,
        0, 0,
        mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
        0b0000111111000111,
        0, 0, 0,
        Vx, Vy, Vz,
        0, 0, 0,
        0, 0)
    vehicle.send_mavlink(msg)
    vehicle.flush()    

def movements():
    speak("Movements Beginning")
    def forward():
        counter = 0
        while counter<1:
            #print("I am Moving Forward for duration second")
            set_velocity_body(1,0,0)
            time.sleep(duration)
            counter=counter+1
    def backward():
        counter = 0
        while counter<1:
            #print("Moving Backward for duration second")
            set_velocity_body(-1,0,0)
            time.sleep(duration)
            counter=counter+1 
    def leftward():
        counter = 0
        while counter<1:
            #print("Moving Backward for duration second")
            set_velocity_body(0,-1,0)
            time.sleep(duration)
            counter=counter+1 
    def rightward():
        counter = 0
        while counter<1:
            #print("Moving Backward for duration second")
            set_velocity_body(0,1,0)
            time.sleep(duration)
            counter=counter+1             


    if vehicle.armed == False:
        print("Vehicle Disarmed") 
    else:   
        while True:
            df = getDistance(front)

            if  df > minDist:
                print("Distance of OBSTACLE to FRONT SENSOR ",df)
                forward()
            else:
                backward()
            dl = getDistance(left)
            dr = getDistance(right)    
            if dl < 100 and dr < 100:
                break
            if dl < 100:
                rightward()
            if dr < 100:
                leftward()    

def landAndDisarm():
    print("Landing Initiated")
    speak("I will start landing now")
    vehicle.mode = VehicleMode("LAND")
    time.sleep(1)
    while True:
        print (" Altitude Landing: ", vehicle.location.global_relative_frame.alt)
        if(vehicle.armed == False):
                print("Vehicle Disarmed")
                GPIO.cleanup()
                exit()
        #Break and return from function just below target altitude.
        if vehicle.location.global_relative_frame.alt<=0.05:
            print ("Landed")
            time.sleep(10)
            print("Disarming Motors")
            vehicle.armed=False
            break
        time.sleep(1)

GPIO.setmode(GPIO.BCM)
print(" Initialising UltrasonicSensors")#Add other sensors here as well.
speak("Hi, I am Icefire.")
speak("I am Created by Prasanna.")
front = [23,24] #forward ultrasonic sensor [trigger,echo]
left = [17,27]
right = [18,22]
speak("  I will now initialise my Ultrasonic Sensors and will test if all connections are set and are good to go,Please wait")

minDist = 80

connection_string = '/dev/ttyS0'
print("Connecting to vehicle on: %s" % (connection_string,))
vehicle = connect(connection_string, wait_ready=True,baud=57600)
print ("Vehicle Connected!")
speak("All Connections are perfect,following are a few data you must know ")
print (" GPS: %s" % vehicle.gps_0)
speak("%s" % vehicle.gps_0)
print (" Battery: %s" % vehicle.battery)
print (" Last Heartbeat: %s" % vehicle.last_heartbeat)
print (" Is Armable?: %s" % vehicle.is_armable)
speak("a Is Armable?: %s" % vehicle.is_armable)
print (" System status: %s" % vehicle.system_status.state)
print (" Mode: %s" % vehicle.mode.name)    # settable
speak ("a Mode: %s" % vehicle.mode.name) 

if __name__ == '__main__':
    try:
        if vehicle.is_armable:
            arm_and_takeoff(0.8)
            movements()
            landAndDisarm()
            print("Mission Successful")
            speak("I have completed my Mission Successfully")
        else:
            speak("Sorry I could not complete my mission")    


    except KeyboardInterrupt:
        print("Measurement stopped by User")
        GPIO.cleanup()
        vehicle.close()



