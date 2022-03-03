# pip install pyserial  -> import serial
from printrun.printcore import printcore # I copied the printrun source into a local sub folder ...
from printrun import gcoder

import time, sys
COM = 'COM3'


# printcore('/dev/ttyUSB0', 115200) on Mac or p.printcore('COM3',115200) on Windows
try:
  p = printcore(COM, 115200)
except Exception as e:
  print("Error with serial port", COM, e)
  sys.exit("Fix this and try again!")

# wait until printer is online
while not p.online:
  time.sleep(1)
  print("Waiting for device to come online ... ", end="")
print("Device online!")

gcode=[i.strip() for i in open('setup_rectangle.gcode')]
gcode = gcoder.LightGCode(gcode)

p.startprint(gcode) # this will start a print

#If you need to interact with the printer:
p.send_now("M105") # get extruder temp
p.send_now("M501") # EEPROM info
#p.pause() # use these to pause/resume the current print
#p.resume()
#p.disconnect() # this is how you disconnect from the printer once you are done. This will also stop running prints.