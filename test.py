
from machine import Pin, I2C
import machine
import time


a = machine.ADC(machine.Pin(33))
b = machine.ADC(machine.Pin(25))
                  
while True:
    print("Part A:",a.read(), "Port B:",b.read(),end='\r')
    time.sleep(2)

