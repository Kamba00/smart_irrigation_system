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
import requests
import time

api_key = 'insert your  own token here'
city = 'insert location'
base_url = f'http://api.openweathermap.org/data/2.5/weather?q={city}&appid={api_key}&units=metric'

#tank features
tank_full = 30#cm
def calculate_dew_point(temperature, humidity):
    return temperature - (100 - humidity) / 5

def calculate_heat_index(temperature, humidity):
    c = [-42.379, 2.04901523, 10.14333127, -0.22475541, -6.83783e-03, -5.481717e-02, 1.22874e-03, 8.5282e-04, -1.99e-06]
    
    # Convert temperature to Fahrenheit
    temp_f = (temperature * 9/5) + 32
    
    # Calculate heat index
    heat_index = c[0] + (c[1] * temp_f) + (c[2] * humidity) + (c[3] * temp_f * humidity) + (c[4] * temp_f**2) + (c[5] * humidity**2) + (c[6] * temp_f**2 * humidity) + (c[7] * temp_f * humidity**2) + (c[8] * temp_f**2 * humidity**2)
    
    # Convert back to Celsius
    heat_index_c = (heat_index - 32) * 5/9
    
    return heat_index_c

def calculate_wind_chill(ta, v):
    # ta is the air temperature in Celsius
    # v is the wind speed in meters per second
    
    # Convert wind speed to km/h
    v_kmh = v * 3.6
    
    # Calculate wind chill index
    wci_celsius = 13.12 + 0.6215 * ta - 11.37 * v_kmh**0.16 + 0.3965 * ta * v_kmh**0.16
    
    return wci_celsius

def interpret_visibility(weather_data):
    if weather_data:
        main_weather = weather_data['weather'][0]['main']
        description = weather_data['weather'][0]['description']
        clouds = weather_data['clouds']['all']
        visibility = weather_data.get('visibility', None)

        print(f"Weather: {main_weather}, Description: {description}")
        print(f"Cloudiness: {clouds}%")
        print(f"Visibility: {visibility} meters" if visibility is not None else "Visibility data not available")
    else:
        print("No weather data available")

def interpret_uvIndex(weather_data):
    if weather_data:
        main_weather = weather_data['weather'][0]['main']
        description = weather_data['weather'][0]['description']
        uv_index = weather_data.get('uvIndex', None)

        print(f"Weather: {main_weather}, Description: {description}")
        print(f"UV Index: {uv_index}" if uv_index is not None else "UV Index data not available")
    else:
        print("No weather data available")

#reading from data base
global outdoor_temp,outdoor_humidity,feels_like,pressure,wind_chill,wind_direction,wind_gust,\
        cloudiness,rainfall_hour,daily_rainfall,dew_point,heat_index
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

#valve 2
GPIO.setup(17, GPIO.OUT) #open valve (IN2)
GPIO.setup(13, GPIO.OUT) #close valve (IN1)
GPIO.output(17, GPIO.HIGH) #set the output to low (relay off)
GPIO.output(13, GPIO.HIGH) #set the output to low (relay off)
GPIO.setup(5, GPIO.IN) #limit switch - fully open
GPIO.setup(6, GPIO.IN) #limit switch - fully closed
# tank level sensor settings
TRIG = 23  # Associate pin 15 to TRIG
ECHO = 24  # Associate pin 14 to Echo

GPIO.setup(TRIG, GPIO.OUT)  # Set pin as GPIO out
GPIO.setup(ECHO, GPIO.IN)  # Set pin as GPIO in

# array to store data
data = []
destination = "/home/mine/Desktop/Irrigation system/Data/irrigation_logic_recent.csv"
fields = [
    "Timestamp",
    "Tank level",
    "Temperature_1",
    "Flow rate 1 in L/Min",
    "Moisture 1 ADC",
    "Moisture 1 Voltage",
    "Valve open time",
    "Valve close time",
    "outdoor_temp",
    "outdoor_humidity",
    "feels_like",
    "pressure",
    "wind_chill",
    "wind_direction",
    "wind_gust",
    "cloudiness",
    "rainfall_hour",
    "daily_rainfall",
    "dew_point",
    "heat_index"
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

    return temp1,temp2

valve_2_status = 0
def v2_open():
    global start
    # Open valve 2
    GPIO.output(10, GPIO.LOW)  # open the valve (activate the first relay)
    GPIO.output(9, GPIO.HIGH)  # keep the second relay off
    time.sleep(4)  # wait for 4 seconds
    GPIO.output(10, GPIO.HIGH)  # turn off the first relay

    v2_open_state = GPIO.input(20)  # read the status of v2 open limit switch input
    print(
        "Valve 1 Open Status: " + str(v2_open_state)
    )  # print the status of the limit switch input
    dt = datetime.now()
    start = dt

def v2_close():

    #Close valve 2
    GPIO.output(17, GPIO.HIGH) #keep the first relay off
    GPIO.output(13, GPIO.LOW) #close the valve (activate the second relay)
    time.sleep(4) #wait for 4 seconds
    GPIO.output(13, GPIO.HIGH) #turn off the second relay

    v2_closed_state = GPIO.input(16)  # read the status of v1 closed limit switch input
    valve_2_status = "Valve 2 Closed Status: " + str(v2_closed_state)
    print(
        "Valve 2 Closed Status: " + str(v2_closed_state)
    )  # print the status of the limit switch
    dt = datetime.now()
    stop = dt

def moisture():
    global percentage_0,percentage_1
    # Initialize the I2C interface
    i2c = busio.I2C(board.SCL, board.SDA)

    # Create an ADS1115 object
    ads = ADS.ADS1115(i2c)

    # Define the analog input channel
    channel_0 = AnalogIn(ads, ADS.P0)
    percentage_0 = round(channel_0.value * -0.00911 + 199)
    # Define the analog input channel
    channel_1 = AnalogIn(ads, ADS.P1)
    percentage_1 = round(channel_1.value * -0.00911 + 199)
    # Loop to read the analog input continuously
    return percentage_0, round(channel_0.voltage, 2),percentage_1, round(channel_1.voltage, 2)

def moisture_temp():
    global soil_moisture,soil_temp
    soil_moisture = []
    soil_moisture.append(percentage_0,percentage_1)
    soil_temp = []
    soil_temp.append(temp1,temp2)
    return soil_temp,soil_moisture

def average():
    global avg_soil_moisture,avg_soil_temp
    avg_soil_moisture = sum(soil_moisture)/len(soil_moisture)
    avg_soil_temp = sum(soil_temp)/len(soil_temp)
    return avg_soil_moisture,avg_soil_temp

def check_day_night():
    global is_it_day,is_it_night
    is_it_day = False
    is_it_night = False
    mytime = time.localtime()
    if mytime.tm_hour > 6 or mytime.tm_hour < 18:
        print ('It is AM')
        is_it_day = True
        is_it_night = False
    else:
        print ('It is PM')
        is_it_night = True
        is_it_day = False
    return is_it_day,is_it_night


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

def weather_values():
    response = requests.get(base_url)
    weather_data = response.json()

    # daily_data = weather_data['rain']
    outdoor_temp = weather_data['main']['temp']
    outdoor_humidity = weather_data['main']['humidity']
    feels_like = weather_data['main']['feels_like']
    pressure = weather_data['main']['pressure']
    wind_speed = weather_data['wind']['speed']
    wind_direction = weather_data['wind']['deg']
    wind_gust = weather_data['wind']['gust']
    cloudiness = weather_data['clouds']['all']
    
    #calculate dewpoint,heat index,wind chill
    dew_point = calculate_dew_point(sum(temperature()[0],temperature()[1],outdoor_temp)/3, outdoor_humidity)
    heat_index = calculate_heat_index(sum(temperature()[0],temperature()[1],outdoor_temp)/3, outdoor_humidity)
    wind_chill = calculate_wind_chill(sum(temperature()[0],temperature()[1],outdoor_temp)/3, wind_speed)

    return outdoor_temp,outdoor_humidity,feels_like,pressure,wind_chill,wind_direction,wind_direction,wind_gust,\
        cloudiness,dew_point,heat_index,wind_speed
print(weather_values())

def run_valve():
    try:
        while is_it_day:
            if tank_level()>0.5*tank_full and \
                moisture[0] <65 and temperature[0] <29 and\
                    temperature[1]>26 :
                            v2_open()
            else:
                if moisture[0]  > 85 :
                    v2_close()
                break
        while is_it_night:
            if tank_level()>0.5*tank_full and \
                moisture[0] <65 and temperature[0] <25 and\
                    temperature[0]>20 :
                            v2_open()
            else:
                if moisture[0]  > 85 :
                    v2_close()       
                break     
    except Exception as e:
        print(e)

schedule.every(30).seconds.do(data_read)
schedule.every(6).hours.do(average)
schedule.every(1).seconds.do(check_day_night)
schedule.every(1).seconds.do(moisture_temp)
schedule.every(10).seconds.do(run_valve)

def run():
    schedule.run_pending()
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
                #avg_soil_moisture,
                #avg_soil_temp,
                weather_values()[0],
                weather_values()[1],
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

