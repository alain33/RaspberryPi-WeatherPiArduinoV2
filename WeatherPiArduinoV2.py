#!/usr/bin/env python
#
# WeatherPiArduino V2 Test File
# Version 2.0 May 9, 2016
#
# SwitchDoc Labs
# www.switchdoc.com
#
#

# imports

import sys
import time
from datetime import datetime
import random 

import config

import subprocess
import RPi.GPIO as GPIO


sys.path.append('./RTC_SDL_DS3231')
sys.path.append('./Adafruit_Python_BMP')
sys.path.append('./Adafruit_Python_GPIO')
sys.path.append('./SDL_Pi_WeatherRack')
sys.path.append('./SDL_Pi_FRAM')
sys.path.append('./RaspberryPi-AS3935/RPi_AS3935')


import SDL_DS3231
import Adafruit_BMP.BMP280 as BMP280
import SDL_Pi_WeatherRack as SDL_Pi_WeatherRack

import SDL_Pi_FRAM
from RPi_AS3935 import RPi_AS3935

################
# Device Present State Variables
###############

config.AS3935_Present = False
config.DS3231_Present = False
config.BMP280_Present = False
config.FRAM_Present = False
config.HTU21DF_Present = False
config.AM2315_Present = False
config.ADS1015_Present = False
config.ADS1115_Present = False


def returnStatusLine(device, state):

	returnString = device
	if (state == True):
		returnString = returnString + ":   \t\tPresent" 
	else:
		returnString = returnString + ":   \t\tNot Present"
	return returnString


###############df   

#WeatherRack Weather Sensors
#
# GPIO Numbering Mode GPIO.BCM
#

anemometerPin = 26
rainPin = 21

# constants

SDL_MODE_INTERNAL_AD = 0
SDL_MODE_I2C_ADS1015 = 1    # internally, the library checks for ADS1115 or ADS1015 if found

#sample mode means return immediately.  THe wind speed is averaged at sampleTime or when you ask, whichever is longer
SDL_MODE_SAMPLE = 0
#Delay mode means to wait for sampleTime and the average after that time.
SDL_MODE_DELAY = 1

weatherStation = SDL_Pi_WeatherRack.SDL_Pi_WeatherRack(anemometerPin, rainPin, 0,0, SDL_MODE_I2C_ADS1015)

weatherStation.setWindMode(SDL_MODE_SAMPLE, 5.0)
#weatherStation.setWindMode(SDL_MODE_DELAY, 5.0)


################

# DS3231/AT24C32 Setup
filename = time.strftime("%Y-%m-%d%H:%M:%SRTCTest") + ".txt"
starttime = datetime.utcnow()

ds3231 = SDL_DS3231.SDL_DS3231(1, 0x68)


try:

	#comment out the next line after the clock has been initialized
	ds3231.write_now()
	print "DS3231=\t\t%s" % ds3231.read_datetime()
	config.DS3231_Present = True
except IOError as e:
	#    print "I/O error({0}): {1}".format(e.errno, e.strerror)
	config.DS3231_Present = False

	
################

# BMP280 Setup

try:
	bmp280 = BMP280.BMP280()
	config.BMP280_Present = True

except IOError as e:

	#    print "I/O error({0}): {1}".format(e.errno, e.strerror)
	config.BMP280_Present = False

################

# HTU21DF Detection 
try:
	HTU21DFOut = subprocess.check_output(["htu21dflib/htu21dflib","-l"])
	config.HTU21DF_Present = True
except:
	config.HTU21DF_Present = False


################

# ad3935 Set up Lightning Detector

as3935 = RPi_AS3935(address=0x03, bus=1)

try:

	as3935.set_indoors(True)
	config.AS3935_Present = True

except IOError as e:

    #    print "I/O error({0}): {1}".format(e.errno, e.strerror)
    config.AS3935_Present = False


if (config.AS3935_Present == True):
	as3935.set_noise_floor(0)
	as3935.calibrate(tun_cap=0x0F)

as3935LastInterrupt = 0
as3935LightningCount = 0
as3935LastDistance = 0
as3935LastStatus = ""

def handle_as3935_interrupt(channel):
    time.sleep(0.003)
    global as3935, as3935LastInterrupt, as3935LastDistance, as3935LastStatus
    reason = as3935.get_interrupt()
    as3935LastInterrupt = reason
    if reason == 0x01:
	as3935LastStatus = "Noise Floor too low. Adjusting"
        as3935.raise_noise_floor()
    elif reason == 0x04:
	as3935LastStatus = "Disturber detected - masking"
        as3935.set_mask_disturber(True)
    elif reason == 0x08:
        now = datetime.now().strftime('%H:%M:%S - %Y/%m/%d')
        distance = as3935.get_distance()
	as3935LastDistance = distance
	as3935LastStatus = "Lightning Detected "  + str(distance) + "km away. (%s)" % now



as3935pin = 25

if (config.AS3935_Present == True):
	GPIO.setup(as3935pin, GPIO.IN)
	GPIO.add_event_detect(as3935pin, GPIO.RISING, callback=handle_as3935_interrupt)

###############

# Set up FRAM 

fram = SDL_Pi_FRAM.SDL_Pi_FRAM(addr = 0x50)
# FRAM Detection 
try:
	fram.read8(0)
	config.FRAM_Present = True
except:
	config.FRAM_Present = False

###############

# Detect AM2315 
try:
	from tentacle_pi.AM2315 import AM2315
	try:
		am2315 = AM2315(0x5c,"/dev/i2c-1")
		temperature, humidity, crc_check = am2315.sense()
		print "AM2315 =", temperature
		config.AM2315_Present = True
	except:
		config.AM2315_Present = False
except:
	config.AM2315_Present = False
	print "------> See Readme to install tentacle_pi"





# Main Loop - sleeps 10 seconds
# Tests all I2C devices on WeatherPiArduino 


# Main Program

print ""
print "WeatherPiArduino Demo / Test Version 1.0 - SwitchDoc Labs"
print ""
print ""
print "Program Started at:"+ time.strftime("%Y-%m-%d %H:%M:%S")
print ""

totalRain = 0


print "----------------------"
print returnStatusLine("DS3231",config.DS3231_Present)
print returnStatusLine("BMP280",config.BMP280_Present)
print returnStatusLine("FRAM",config.FRAM_Present)
print returnStatusLine("HTU21DF",config.HTU21DF_Present)
print returnStatusLine("AM2315",config.AM2315_Present)
print returnStatusLine("ADS1015",config.ADS1015_Present)
print returnStatusLine("ADS1115",config.ADS1115_Present)
print returnStatusLine("AS3935",config.AS3935_Present)
print "----------------------"







while True:




	print "---------------------------------------- "
	print "----------------- "
	print " WeatherRack Weather Sensors" 
	print "----------------- "
	#

 	currentWindSpeed = weatherStation.current_wind_speed()/1.6
  	currentWindGust = weatherStation.get_wind_gust()/1.6
  	totalRain = totalRain + weatherStation.get_current_rain_total()/25.4
  	print("Rain Total=\t%0.2f in")%(totalRain)
  	print("Wind Speed=\t%0.2f MPH")%(currentWindSpeed)
    	print("MPH wind_gust=\t%0.2f MPH")%(currentWindGust)
  	if (config.ADS1015_Present or config.ADS1115_Present):	
		print "Wind Direction=\t\t\t %0.2f Degrees" % weatherStation.current_wind_direction()
		print "Wind Direction Voltage=\t\t %0.3f V" % weatherStation.current_wind_direction_voltage()

	print "----------------- "
	print "----------------- "
	if (config.DS3231_Present == True):
		print " DS3231 Real Time Clock"
	else:
		print " DS3231 Real Time Clock Not Present"
	
	print "----------------- "
	#

	if (config.DS3231_Present == True):
		currenttime = datetime.utcnow()

		deltatime = currenttime - starttime
	 
		print "Raspberry Pi=\t" + time.strftime("%Y-%m-%d %H:%M:%S")
		
		print "DS3231=\t\t%s" % ds3231.read_datetime()
	
		print "DS3231 Temperature= \t%0.2f C" % ds3231.getTemp()
		# do the AT24C32 eeprom

		print "----------------- "
		print "----------------- "
		print " AT24C32 EEPROM"
		print "----------------- "
		print "writing first 4 addresses with random data"
		for x in range(0,4):
			value = random.randint(0,255)
			print "address = %i writing value=%i" % (x, value) 	
			ds3231.write_AT24C32_byte(x, value)
		print "----------------- "
	
		print "reading first 4 addresses"
		for x in range(0,4):
			print "address = %i value = %i" %(x, ds3231.read_AT24C32_byte(x)) 
		print "----------------- "



	print "----------------- "
	if (config.BMP280_Present == True):
		print " BMP280 Barometer"
	else:
		print " BMP280 Barometer Not Present"
	print "----------------- "

	if (config.BMP280_Present):
		print 'Temperature = \t{0:0.2f} C'.format(bmp280.read_temperature())
		print 'Pressure = \t{0:0.2f} KPa'.format(bmp280.read_pressure()/1000)
		print 'Altitude = \t{0:0.2f} m'.format(bmp280.read_altitude())
		print 'Sealevel Pressure = \t{0:0.2f} KPa'.format(bmp280.read_sealevel_pressure()/1000)
	print "----------------- "

	print "----------------- "
	if (config.AM2315_Present == True):
		print " AM2315 Temperature/Humidity Sensor"
	else:
		print " AM2315 Temperature/Humidity  Sensor Not Present"
	print "----------------- "

	if (config.AM2315_Present):
    		temperature, humidity, crc_check = am2315.sense()
    		print "AM2315 temperature: %0.1f" % temperature
    		print "AM2315 humidity: %0.1f" % humidity
    		print "AM2315 crc: %s" % crc_check
	print "----------------- "

	print "----------------- "
	if (config.HTU21DF_Present == True):
		print " HTU21DF Temp/Hum"
	else:
		print " HTU21DF Temp/Hum Not Present"
	print "----------------- "

	# We use a C library for this device as it just doesn't play well with Python and smbus/I2C libraries
	if (config.HTU21DF_Present):
		HTU21DFOut = subprocess.check_output(["htu21dflib/htu21dflib","-l"])
		splitstring = HTU21DFOut.split()

		HTUtemperature = float(splitstring[0])	
		HTUhumidity = float(splitstring[1])	
		print "Temperature = \t%0.2f C" % HTUtemperature
		print "Humidity = \t%0.2f %%" % HTUhumidity
	print "----------------- "

	print "----------------- "
	if (config.AS3935_Present):
		print " AS3935 Lightning Detector"
	else:
		print " AS3935 Lightning Detector Not Present"
	print "----------------- "

	if (config.AS3935_Present):
		print "Last result from AS3935:"

		if (as3935LastInterrupt == 0x00):
			print "----No Lightning detected---"
		
		if (as3935LastInterrupt == 0x01):
			print "Noise Floor: %s" % as3935LastStatus
			as3935LastInterrupt = 0x00

		if (as3935LastInterrupt == 0x04):
			print "Disturber: %s" % as3935LastStatus
			as3935LastInterrupt = 0x00

		if (as3935LastInterrupt == 0x08):
			print "Lightning: %s" % as3935LastStatus
			as3935LightningCount += 1
			as3935LastInterrupt = 0x00

		print "Lightning Count = ", as3935LightningCount
	print "----------------- "
	
	print "----------------- "
	if (config.FRAM_Present):
		print " FRAM Test"
	else:
		print " FRAM Not Present"
	print "----------------- "

        if (config.FRAM_Present):
		print "writing first 3 addresses with random data"
		for x in range(0,3):
			value = random.randint(0,255)
                	print "address = %i writing value=%i" % (x, value)
                	fram.write8(x, value)
        	print "----------------- "

        	print "reading first 3 addresses"
        	for x in range(0,3):
                	print "address = %i value = %i" %(x, fram.read8(x))
        print "----------------- "
	print


	print "Sleeping 10 seconds"
	time.sleep(10.0)


