import serial
import math
from mpl_toolkits.mplot3d.axes3d import Axes3D
import matplotlib.pyplot as plt

# Create a serial connection
s = serial.Serial('COM8', 115200)

s.reset_output_buffer()
s.reset_input_buffer()

# Create a figure
ax = plt.axes(projection = "3d")

# Create a list to store the data
x = []
y = []
z = []

x_val = 0
step = 200 #arbitrary value, to offset the 3d plot

    
input("Press Enter to Start")

s.readline() # ignore the first line
for i in range(256): # there will be 256 points per scan, transmitted via serial
    a = s.readline() # read the serial data
    distance = a.decode() # strip the newline character and decode the data
    print(distance, end = "") # print the data (for debugging)
    angle = float(i * math.pi / 128) # calculate the angle of the point
    z.append(int(distance) * math.cos(angle)) # convert to cartesian coordinates
    y.append(int(distance) * math.sin(angle)) # and append to the list
    x.append(x_val) # x is the same for all points in a scan

ax.plot3D(x, y, z) # plot the data

x_val += step

for i in range(256):
   z.append(z[i])
   y.append(y[i])
   x.append(x_val)  # offset the x value for the next scan

ax.plot3D(x, y, z) # plot the data

x_val += step

for i in range(256):
   z.append(z[i])
   y.append(y[i])
   x.append(x_val)  # offset the x value for the next scan

ax.plot3D(x, y, z) # plot the data

            
s.close() # close the serial connection

plt.show() # show the plot