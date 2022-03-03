# Generates gcode from a DEM and line data (shape file)
# Python 3
# charding@iastate.edu
# Jan. 21, 2022


from sys import platform
from osgeo import gdal # for reading geotiffs
import matplotlib.pyplot as plt
import numpy as np


# For visualizing tool path in matplotlib (aka preview mode)
last_xy=() # global for last point
num_segs = 0 # global point (line segment) counter
col = "black" # global color
plt.axis("equal")
def pl(p=None):
    """ plot to point p from last point (state machine)
        if no last point is stored, store p as last point and return
        to erase the last point (reset), use no arg
        color can be set by arg as a valid color string ("red", "blue", etc.)
    """
    global last_xy, num_segs, col
    if isinstance(p, type("str")):  # set color
        col = p
        return
    if last_xy == (): 
        last_xy = p # just store p and wait for next point to plot a line
        return
    if p == None: # start new line (first point)
        last_xy = ()
        num_segs = 0
        return
    assert last_xy != None, "1"
    assert last_xy != (), "2"
    assert p != None, "3"
    assert p != (), "4"

    plt.plot((last_xy[0],p[0]), (last_xy[1],p[1]), color=col) #x1/x2  y1/y2  (weird ...)
    #print(num_segs,last_xy, p)
    num_segs += 1
    #print((last_xy[0],p[0]), (last_xy[1],p[1]))
    last_xy = p
    #plt.show()

'''
# test
pl((0,0))
pl((10,0))
pl((10,10))
pl((0,10))
pl((0,0))
pl() # start new line
pl((5,5))
pl((-2,5))
pl((0,0))
pl((5,5))
plt.show()
'''


# Read in geotiff with GDAL
# TODO user input
dem_filename = "deming_5m_sm_with_DEM_1m_clipped.tif"
dem = gdal.Open(dem_filename)

# get bounding box coordinates
band = dem.GetRasterBand(1)
ulx, xres, xskew, uly, yskew, yres  = dem.GetGeoTransform() 
xres, yres = abs(xres), abs(yres)  # res can be negative(?) 
rwidth, rheight = dem.RasterXSize, dem.RasterYSize # number of cells in x and y
rwidth_in_m, rheight_in_m = rwidth * xres, rheight * yres 
lrx = ulx + rwidth_in_m # lower right
lry = uly - rheight_in_m
print("raster", rwidth,"x", rheight, "cells, w/h ratio", rwidth/rheight)


# TODO user input
# real world width and height (measured) in mm
mmwidth = 200
mmheight = 240
print("measured  model", mmwidth,"x", mmheight, "mm", "w/h ratio", mmwidth/mmheight )


# get the real scaling in x and y
wraster_to_model_scale = mmwidth / rwidth_in_m
hraster_to_model_scale = mmheight / rheight_in_m

# lets assume that the avg. x/y scale can be used for z
zraster_to_model_scale = 0.1 * (hraster_to_model_scale + wraster_to_model_scale) / 2

# make numpy array with the real-world elevation values
dem_ar = band.ReadAsArray()

# TODO read base thickness and elev min max from logfile
# elevation scaling: 0.6 - 27.720467 <- from logfile
base_thickness = 0.6 # mm
min_elev = 0.6 # mm
max_elev = 27.720467
print("model elev", min_elev, max_elev)
raster_to_model_elev_scale = (max_elev - min_elev) / (dem_ar.max() - dem_ar.min())
min_raster_elev = dem_ar.min()
print("raster elev", dem_ar.min(), dem_ar.max())
print("raster to model elevation scale:", raster_to_model_elev_scale)

# TODO user input
# gcode parameters
LINE_FEED_RATE = 600 # 600 mm/min = 10 mm/sec speed for drawing
MOVE_FEED_RATE = 1200 # speed for moves
Z_SAFE_HEIGHT = round(dem_ar.max() * zraster_to_model_scale + 1, 3) # needs to be large enough to NOT touch the model anywhere!


# Set mode here!
# if False, will only create matplotlib preview, must be True to also send gcode to the printer
do_plot = False 

# Establish connection to printer
if do_plot:
    # pip install pyserial  -> import serial
    from printrun.printcore import printcore # BTW I copied the printrun source into a local sub folder instead doing a proper install
    import time, sys
    COM = 'COM3' # TODO: user input (use the printrun desktop app to verify the port!)

    try:
        p = printcore(COM, 115200) # printcore('/dev/ttyUSB0', 115200) on Mac or p.printcore('COM3',115200) on Windows
    except Exception as e:
        print("Error with serial port", COM, e)
        sys.exit("Fix this and try again!")

    # wait until printer is online
    while not p.online:
        time.sleep(1)
    print("Waiting for device to come online ... ", end="")
    print("Device online!")


# wrapper around send_now() to omit it for preview-only mode
def plot(gcode):
    global do_plot, p
    if do_plot:
        p.send_now("gcode")

if do_plot:
    print("I will home the x and y axis now. Manually lower z so that the pen makes contact with the bed.")
    input("Press enter when you're done with z homing")

    print("Drawing reference rectangle")

    print("M82 ;absolute extrusion mode")
    print("G28 X Y ;home x y only as z will trigger bedprobe and heating!")

# go to 0,0 
plot(f"G0 X{0} Y{0} Z{0} F{MOVE_FEED_RATE}")
pl((0,0))

def G0(x,y): # convenience function
    plot(f"G0 X{x} Y{y} Z{0} F{LINE_FEED_RATE}")
    pl((x,y))

# draw boundary with 0/0 in left lower corner 
G0(mmwidth, 0)
G0(mmwidth, mmheight)
G0(0, mmheight)
G0(0, 0)

# lift head to safe height
plot(f"G0 X{0} Y{0} Z{Z_SAFE_HEIGHT} F{MOVE_FEED_RATE}")
#plt.show() # this will show the rectangle only    
print("Drawing reference rectangle done")


# read in line shapefile from shp folder
import shapefile
shpfilename = "demin_lines_1m_z.shp"
#shpfilename = "test.shp"
shpfile = "shp/" + shpfilename
print("Reading line data from", shpfile)
sf = shapefile.Reader(shpfile)

# check for line features (Z is ok but will be ignored)
assert sf.shapeTypeName == "POLYLINEZ" or sf.shapeTypeName == "POLYGON" , "Error: shapefile needs to be type POLYLINEZ or POLYGON"

shapes = sf.shapes() # get all shapes

# Plot lines from shapefile
if do_plot:
    print("Match the terrain print on the bed with the reference rectangle and clamp it down")
    input("Press Enter to start 3D plotting")
    print("Plotting lines from", shpfilename, "draped on", dem_filename)
    plot(f"G0 X{0} Y{0} Z{Z_SAFE_HEIGHT} F{MOVE_FEED_RATE}") # Ensure (again) that we're above the model!)

# run though all features
for lidx, ln in enumerate(shapes):
    pl() # lift penn in viz

    # get value of first column as feature name
    ftname = sf.records()[lidx][0]

    print("Drawing",lidx, ftname)

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
            plot(f"G0 X{x} Y{y} Z{Z_SAFE_HEIGHT} F{MOVE_FEED_RATE}")
            #print(f"G0 X{x} Y{y} Z{Z_SAFE_HEIGHT} F{MOVE_FEED_RATE}")

        plot(f"G0 X{x} Y{y} Z{z} F{LINE_FEED_RATE}")
        #print(f"G0 X{x} Y{y} Z{z} F{LINE_FEED_RATE}")
        pidx += 1

    plot(f"G0 X{x} Y{y} Z{Z_SAFE_HEIGHT} F{MOVE_FEED_RATE}") # lift back to safe height
    #print(f"G0 X{x} Y{y} Z{Z_SAFE_HEIGHT} F{MOVE_FEED_RATE}\n") # lift back to safe height

plot(f"G0 X{0} Y{0} Z{Z_SAFE_HEIGHT} F{MOVE_FEED_RATE}") # goto origin, lift back to safe height
#print(f"G0 X{0} Y{0} Z{Z_SAFE_HEIGHT} F{MOVE_FEED_RATE}")

plt.show() # quite the preview app to continue
print("3D plot complete")


