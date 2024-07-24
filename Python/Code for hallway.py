import serial
import math
from mpl_toolkits.mplot3d.axes3d import Axes3D
import matplotlib.pyplot as plt

# Create a serial connection
s = serial.Serial('[your_COM_port_here]', 115200)

s.reset_output_buffer()
s.reset_input_buffer()

# Create a figure
ax = plt.axes(projection = "3d")

# Create a list to store the data
x = []
y = []
z = []

x_val = 0
step = [step_size_here] # by measurement, my step size while taking scans is about 25.5 mm

    
input("Press Enter to Start")

while (True): #i dont know how long the hallway is, so i just loop until the user quits
    inp = input("Press Enter to Start or 'q' to quit")
    if (inp == 'q'): # if the user wants to quit,
        break
    print ("Starting scan")
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
    x_val += step; # offset the x value for the next scan

            
s.close() # close the serial connection

plt.show() # show the plot