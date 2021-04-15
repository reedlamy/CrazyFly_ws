#!/usr/bin/env python

import RPi.GPIO as GPIO
import time
from gpiozero import PWMOutputDevice

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)

DATA_PIN = 16
CLK_PIN = 20
LATCH_PIN = 21

GPIO.setup(DATA_PIN, GPIO.OUT)
GPIO.setup(LATCH_PIN, GPIO.OUT)
GPIO.setup(CLK_PIN, GPIO.OUT)

PWM1 = PWMOutputDevice(5)
PWM2 = PWMOutputDevice(6)
PWM3 = PWMOutputDevice(13)
PWM4 = PWMOutputDevice(19)

# M3A = 0
# M2A = 0
# M1A = 0
# M1B = 1
# M2B = 1
# M4A = 0
# M3B = 1
# M4B = 1


def setSpeed(l_speed, r_speed):
    PWM1.value = l_speed
    PWM2.value = r_speed
    PWM3.value = l_speed
    PWM4.value = r_speed


def shiftout(pins, sec):
    GPIO.output(LATCH_PIN, 0)
    for bit in pins:
        GPIO.output(DATA_PIN, bit)
        GPIO.output(CLK_PIN, 1)
        GPIO.output(CLK_PIN, 0)
    GPIO.output(LATCH_PIN, 1)
    time.sleep(sec)

