import RPi.GPIO as GPIO
import os
import glob
import time
import board
import busio
import adafruit_ads1x15.ads1115 as ADS
from adafruit_ads1x15.analog_in import AnalogIn
import csv
from datetime import datetime
import schedule
import threading


start = "unknown"
stop = "unknown"


# temperature sensor specifications and set-up
os.system("modprobe w1-gpio")
os.system("modprobe w1-therm")

# dv settings
GPIO.cleanup()
GPIO.setmode(GPIO.BCM)
# Stop sending warnings
GPIO.setwarnings(False)

# valve 1
GPIO.setup(10, GPIO.OUT)  # open valve (IN6)
GPIO.setup(9, GPIO.OUT)  # close valve (IN5)
GPIO.output(10, GPIO.HIGH)  # set the output to low (relay off)
GPIO.output(9, GPIO.HIGH)  # set the output to low (relay off)
GPIO.setup(20, GPIO.IN)  # limit switch - fully open
GPIO.setup(16, GPIO.IN)  # limit switch - fully closed

# tank level sensor settings
TRIG = 23  # Associate pin 15 to TRIG
ECHO = 24  # Associate pin 14 to Echo

GPIO.setup(TRIG, GPIO.OUT)  # Set pin as GPIO out
GPIO.setup(ECHO, GPIO.IN)  # Set pin as GPIO in

# array to store data
data = []
destination = "/home/mine/Desktop/Irrigation system/Data/irrigation_scheduled_recent_reformatted.csv"
fields = [
    "Timestamp",
    "Tank level",
    "Temperature_1",
    "Flow rate 1 in L/Min",
    "Moisture 1 ADC",
    "Moisture 1 Voltage",
    "Valve open time",
    "Valve close time",
]

with open(destination, "a", newline="") as csvfile:
    dw = csv.DictWriter(csvfile, delimiter=",", fieldnames=fields)
    dw.writeheader()


def temperature():
    device0 = glob.glob("/sys/bus/w1/devices/" + "28*")[0] + "/w1_slave"
    f0 = open(device0, "r")
    lines0 = f0.readlines()
    f0.close()

    while lines0[0].strip()[-3:] != "YES":
        lines0 = f0.readlines()
    equals_pos = lines0[1].find("t=")
    if equals_pos != -1:
        temp_string0 = lines0[1][equals_pos + 2 :]
        temp1 = round(float(temp_string0) / 1000.0, 1)

    device1 = glob.glob("/sys/bus/w1/devices/" + "28*")[1] + "/w1_slave"
    f1 = open(device1, "r")
    lines1 = f1.readlines()
    f1.close()

    while lines1[0].strip()[-3:] != "YES":
        lines1 = f1.readlines()
    equals_pos = lines1[1].find("t=")
    if equals_pos != -1:
        temp_string1 = lines1[1][equals_pos + 2 :]
        temp2 = round(float(temp_string1) / 1000.0, 1)

    return temp1

valve_1_status = 0
def v1_open():
    global start
    # Open valve 1
    GPIO.output(10, GPIO.LOW)  # open the valve (activate the first relay)
    GPIO.output(9, GPIO.HIGH)  # keep the second relay off
    time.sleep(4)  # wait for 4 seconds
    GPIO.output(10, GPIO.HIGH)  # turn off the first relay

    v1_open_state = GPIO.input(20)  # read the status of v1 open limit switch input
    print(
        "Valve 1 Open Status: " + str(v1_open_state)
    )  # print the status of the limit switch input
    dt = datetime.now()
    start = dt
    print(start)


def v1_close():
    global stop
    # Close valve 1
    GPIO.output(10, GPIO.HIGH)  # keep the first relay off
    GPIO.output(9, GPIO.LOW)  # close the valve (activate the second relay)
    time.sleep(4)  # wait for 4 seconds
    GPIO.output(9, GPIO.HIGH)  # turn off the second relay

    v1_closed_state = GPIO.input(16)  # read the status of v1 closed limit switch input
    valve_1_status = "Valve 2 Closed Status: " + str(v1_closed_state)
    print(
        "Valve 1 Closed Status: " + str(v1_closed_state)
    )  # print the status of the limit switch
    dt = datetime.now()
    stop = dt


def moisture():
    # Initialize the I2C interface
    i2c = busio.I2C(board.SCL, board.SDA)

    # Create an ADS1115 object
    ads = ADS.ADS1115(i2c)

    # Define the analog input channel
    channel_0 = AnalogIn(ads, ADS.P0)
    percentage_0 = round(channel_0.value * -0.00911 + 199)
    # Loop to read the analog input continuously
    return percentage_0, round(channel_0.voltage, 2)


def tank_level():
    GPIO.output(TRIG, False)  # Set TRIG as LOW
    # print ("Waiting For Sensor To Settle")
    time.sleep(2)  # Delay of 2 seconds

    GPIO.output(TRIG, True)  # Set TRIG as HIGH
    time.sleep(0.00001)  # Delay of 0.00001 seconds
    GPIO.output(TRIG, False)  # Set TRIG as LOW

    while GPIO.input(ECHO) == 0:  # Check if Echo is LOW
        pulse_start = time.time()  # Time of the last  LOW pulse

    while GPIO.input(ECHO) == 1:  # Check whether Echo is HIGH
        pulse_end = time.time()  # Time of the last HIGH pulse

    pulse_duration = pulse_end - pulse_start  # pulse duration to a variable

    distance = pulse_duration * 17150  # Calculate distance
    distance = round(distance, 2)  # Round to two decimal points

    if distance > 20 and distance < 400:  # Is distance within range
        pass
        # print ("Distance:",distance - 0.5,"cm")  #Distance with calibration
    else:
        distance = -1
        # print ("Out Of Range" )                 #display out of range
    return distance


def data_read():
    print("saving")

    with open(destination, "a", newline="") as csvfile:
        writer = csv.writer(csvfile, quotechar='"')
        writer.writerow(data)  # Write data to CSV
        
#Schedule the functions to run periodically
schedule.every().day.at("06:00").do(v1_open)
schedule.every().day.at("06:15").do(v1_close)
schedule.every().day.at("18:00").do(v1_open)
schedule.every().day.at("18:15").do(v1_close)
schedule.every(30).seconds.do(data_read)

def run():
    global total_pulse_count,data

    FLOW_SENSOR_PIN = 25  # GPIO pin number where the flow sensor is connected
    
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(FLOW_SENSOR_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    
    total_pulse_count = 0
    
    def pulse_callback(channel):
        global total_pulse_count
        total_pulse_count += 1
    
    GPIO.add_event_detect(FLOW_SENSOR_PIN, GPIO.FALLING, callback=pulse_callback)
    while True:
        try:
            schedule.run_pending()
            start_counter = 0
            flow_1 = total_pulse_count *  60 * 2.25 / 1000
            total_pulse_count = 0

            global data
            dt = datetime.now()
            data = [
                dt,
                tank_level(),
                temperature(),
                flow_1,
                moisture()[0],
                moisture()[1],
                start,
                stop
            ]
            print(data)
        


        except Exception as e:
            print("Error:", e)
        
        except KeyboardInterrupt:
            print("\ncaught keyboard interrupt!, bye")
            GPIO.cleanup()
            sys.exit()


run()


