# Generates gcode from a DEM and line data (shape file)
# Python 3
# charding@iastate.edu
# Jan. 21, 2022


from osgeo import gdal # for reading geotiffs
import matplotlib.pyplot as plt # for plotting
import numpy as np # to get access to the elevation values that were used to create the STL

# Mini state machine for visualizing tool path in matplotlib 
last_xy=() # global for storing the last point plotted
num_segs = 0
col = "black"
plt.axis("equal") # same x and y scale
def pl(p=None):
    """ plot to point p from last point plotted (state machine)
        if no last point is stored, store p as last point and return
        to erase the last point (reset), use no arg
        color can be set by arg as a valid color string (e.g. "red", "blue", "black", etc.)
    """
    global last_xy, num_segs, col # HACK! these are global variable to hold state

    if isinstance(p, type("str")): # interpret a string as color
        col = p
        return
    if last_xy == (): # if last_xy is still empty we're staring a new line
        last_xy = p
        return
    if p == None: # if no arg was given, start a new line
        last_xy = ()
        num_segs = 0
        return
    assert last_xy != None, "1"
    assert last_xy != (), "2"
    assert p != None, "3"
    assert p != (), "4"

    plt.plot((last_xy[0],p[0]), (last_xy[1],p[1]), color=col)
    #print(num_segs,last_xy, p)
    num_segs += 1
    #print((last_xy[0],p[0]), (last_xy[1],p[1]))
    last_xy = p
    #plt.show()  # this actually shows the plotting window, use this later when you want to preview gcode

'''
# test
pl((0,0))
pl((10,0))
pl((10,10))
pl((0,10))
pl((0,0))
pl() # reset (start new line)
pl((5,5))
pl((-2,5))
pl((0,0))
pl((5,5))
plt.show()
'''


# Read in geotiff DEM raster data with GDAL
dem_filename = "deming_5m_sm_with_DEM_1m_clipped.tif" # should later be user input from GUI
dem = gdal.Open(dem_filename)

# get bounding box coordinates
band = dem.GetRasterBand(1)
ulx, xres, xskew, uly, yskew, yres  = dem.GetGeoTransform() 
xres, yres = abs(xres), abs(yres)  # res can be negative(?) so need to use abs()
rwidth, rheight = dem.RasterXSize, dem.RasterYSize # number of pixels in x (width) and y (height)
rwidth_in_m, rheight_in_m = rwidth * xres, rheight * yres # pixel size in meters
lrx = ulx + rwidth_in_m # lower right
lry = uly - rheight_in_m
print("raster", rwidth,"x", rheight, "cells, w/h ratio", rwidth/rheight)

# real world width and height (measured) in mm (also user input)
mmwidth = 200
mmheight = 240
print("measured  model", mmwidth,"x", mmheight, "mm", "w/h ratio", mmwidth/mmheight)


# get the real scaling in x and y
wraster_to_model_scale = mmwidth / rwidth_in_m
hraster_to_model_scale = mmheight / rheight_in_m

# lets assume that the avg. xy scaling can be used for z
zraster_to_model_scale = 0.1 * (hraster_to_model_scale + wraster_to_model_scale) / 2

# make numpy array with the real-world elevation values
dem_ar = band.ReadAsArray()

# elevation scaling:  values are in the logfile, should be parsed automatically or be user input
base_thickness = 0.6 # mm
min_elev = 0.6 # mm
max_elev = 27.720467

print("model elev", min_elev, max_elev)
raster_to_model_elev_scale = (max_elev - min_elev) / (dem_ar.max() - dem_ar.min())
min_raster_elev = dem_ar.min()
print("raster elev", dem_ar.min(), dem_ar.max())
print("raster to model elevation scale:", raster_to_model_elev_scale)


# gcode parameters (user input?)
LINE_FEED_RATE = 120 # 600 mm/min = 10 mm/sec speed for drawing
MOVE_FEED_RATE = 600 # speed for moves
Z_SAFE_HEIGHT = round(dem_ar.max() * zraster_to_model_scale + 1, 3) # needs to be above the max model height

print(f"Drawing speed: {LINE_FEED_RATE} mm/min")
print(f"Move speed: {MOVE_FEED_RATE} mm/min")


# Connect to plotter

# pip install pyserial  to install the serial interface module which printrun needs!
from printrun.printcore import printcore # I copied the printrun source into a local sub folder ...
import time, sys
COM = 'COM3' # find this in the Device Manager - Ports

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

p.send_now("M501") # show EEPROM settings

# create gcode for drawing a rectangle to put the model into
print("I will now home x and y. Use the touchpad to lower z to where the pen touches the plate")
p.send_now("G28 X Y")
input("Hit Enter when done with Z adjustment")

p.send_now("M82 ;absolute extrusion mode")
p.send_now(f"G0 X{0} Y{0} Z{0} F{MOVE_FEED_RATE}")
pl((0,0))

def g1(x,y,f):
    p.send_now(f"G1 X{x} Y{y} Z{0} F{LINE_FEED_RATE}")
    pl((x,y))

# draw boundary with 0/0 in left lower corner 
g1(mmwidth, 0)
g1(mmwidth, mmheight)
g1(0, mmheight)
g1(0, 0)
#plt.show()

# lift head to safe height
print("moving to 0,0 at safe Z height")
p.send_now(f"G0 X{0} Y{0} Z{Z_SAFE_HEIGHT} F{MOVE_FEED_RATE}")
input("affix model and hit enter to start plotting lines")


# read in shapefile from shp folder
import shapefile
shpfilename = "demin_lines_1m_z.shp"
#shpfilename = "test.shp"
shpfile = "shp/" + shpfilename
print("Reading line data from", shpfile)
sf = shapefile.Reader(shpfile)

# check for line features (Z is ok but will be ignored)
assert sf.shapeTypeName == "POLYLINEZ" or sf.shapeTypeName == "POLYGON" , "Error: shapefile needs to be type POLYLINEZ or POLYGON"

shapes = sf.shapes() # get all shapes





print("3D pLotting ", shpfilename, "lines draped on ", dem_filename)
p.send_now(f"G0 X{0} Y{0} Z{Z_SAFE_HEIGHT} F{MOVE_FEED_RATE}")

# run though all features
for lidx, ln in enumerate(shapes):
    pl() # lift pen in viz

    # get value of first column as feature name
    ftname = sf.records()[lidx][0]

    print("Drawing line #",lidx, ftname)

    # Warn if line has multiple parts (should not happen) - will always use the first and only part for now
    if len(ln.parts) > 1: # single part lines will return [0]
        print("Warning: line has multiple parts, which is not supported at this time")

    # get x/y coordinates
    pts = ln.points

    # get 3D points
    pidx = 0 # can't use enumerate here b/c index 0 might lie outside the raster area!
    for p in pts:
        x_,y_  = p # 2D coords

        # Origin in UTM
        x_ -= ulx # x of upper left
        y_ -= lry # y of lower right

        # Skip if outside of raster area
        if x_ < 0 or y_ < 0: 
            continue
        if x_ > rwidth_in_m or y_ > rheight_in_m: 
            continue
        
        # x and y need to be set here to contain the last inside point
        x, y = x_, y_

        # raster cell coordinates for look up z in the DEM
        i,j = int(x / xres), int(y / yres)
        z = dem_ar[j, i] 

        x *= wraster_to_model_scale # scale to mm
        y *= hraster_to_model_scale
        z *= zraster_to_model_scale 

        x = round(x, 3)
        y = round(y, 3)
        z = round(z, 3)

        #print(x,y)
        pl((x,y))

        if pidx == 0: # need to move up and fly to first point
            p.send_now(f"G0 X{x} Y{y} Z{Z_SAFE_HEIGHT} F{MOVE_FEED_RATE}")
            #print(f"G0 X{x} Y{y} Z{Z_SAFE_HEIGHT} F{MOVE_FEED_RATE}")

        p.send_now(f"G1 X{x} Y{y} Z{z} F{LINE_FEED_RATE}")
        #print(f"G0 X{x} Y{y} Z{z} F{LINE_FEED_RATE}")
        pidx += 1

    p.send_now(f"G0 X{x} Y{y} Z{Z_SAFE_HEIGHT} F{MOVE_FEED_RATE}") # lift back to safe height
    #print(f"G0 X{x} Y{y} Z{Z_SAFE_HEIGHT} F{MOVE_FEED_RATE}\n") # lift back to safe height

p.send_now(f"G0 X{0} Y{0} Z{Z_SAFE_HEIGHT} F{MOVE_FEED_RATE}") # goto origin, lift back to safe height
#print(f"G0 X{0} Y{0} Z{Z_SAFE_HEIGHT} F{MOVE_FEED_RATE}")
print("Plotting done")

plt.show() # show matplotlib graph

