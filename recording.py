#!/usr/local/bin/python
# -*- coding: utf-8 -*-
 
import time
import RPi.GPIO as GPIO
import shlex
import subprocess
import serial
from pynmea import nmea
import picamera
import csv
from datetime import datetime as dt
import os

GPIO_SHUTDOWN_BUTTON = 21
GPIO_START_BUTTON = 20
GPIO_LED = 26

def main():
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(GPIO_SHUTDOWN_BUTTON, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
    GPIO.setup(GPIO_START_BUTTON, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
    GPIO.setup(GPIO_LED, GPIO.OUT)

    beforeFlag = False
    
    while True:
        if GPIO.input(GPIO_START_BUTTON) and not beforeFlag:
            beforeFlag = True
            GPIO.output(GPIO_LED, True)
            recording();
        else:
            beforeFlag = False
            GPIO.output(GPIO_LED, False)
            if GPIO.input(GPIO_SHUTDOWN_BUTTON):
                print "shutdown"
                proc = subprocess.Popen(shlex.split('sudo shutdown -h now'))
        time.sleep(1)

    GPIO.cleanup()


def recording():
    target_dir = dt.now().strftime('%Y_%m_%d_%h_%m_%s')
    os.makedirs(target_dir)

    f = open(target_dir+'/data.csv', 'ab')
    csvWriter = csv.writer(f)
    
    beforeFlag = True;
    dataIndex = 1;
    # GPS
    ser = serial.Serial('/dev/ttyAMA0',9600)
    gpgga = nmea.GPGGA()
    # Camera
    camera = picamera.PiCamera()
    
    while True:
        # Camera
        camera.capture(target_dir+'/'+str(dataIndex)+'.jpg')
        # GPS
        gpsLat = -1
        gpsLong= -1
        data = ser.readline()
        if (data.startswith('$GPGGA')):
            gpgga.parse(data)
            gpggaLat = gpgga.latitude
            gpggaLong = gpgga.longitude
            if gpggaLat is not '':
                gpsLat = float(gpggaLat[0:1])  + float(gpggaLat[1:]) / 60
                gpsLong= float(gpggaLong[0:2]) + float(gpggaLong[2:]) / 60
        listData = [dataIndex, gpsLat, gpsLong]
        csvWriter.writerow(listData)
        dataIndex = dataIndex + 1;

        if GPIO.input(GPIO_START_BUTTON) and not beforeFlag:
            break;
        else:
            beforeFlag = False;
        time.sleep(1)

    camera.close()
    f.close()

    
if __name__ == "__main__":
    try:
        main()
    finally:
        print "clean up"
        GPIO.cleanup()

