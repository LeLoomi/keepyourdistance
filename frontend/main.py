import sys
import serial
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib import style

import numpy as np
from numpy.fft import rfft, fft

# Configure serial connection (update port and settings accordingly)
ser = serial.Serial(
    port='/dev/tty.usbmodem103',
    baudrate=115200,
    parity=serial.PARITY_ODD,
    stopbits=serial.STOPBITS_TWO,
    bytesize=serial.EIGHTBITS,
    timeout=1  # important to avoid blocking read
)

style.use('fivethirtyeight')

fig, (ax1, ax2) = plt.subplots(1, 2)

data_plot = []  # store values to plot
X = [] # array for fft 
max_len = 200  # max number of points to display


# sampling rate
sr = 44000

# sampling interval
ts = 1.0/sr
t = np.arange(0,1,ts)


def animate(i):
    # Read all lines currently available from serial
    while ser.in_waiting:
        line = ser.readline().decode('utf-8').strip()  # read line and decode
        if line:
            # Assuming your serial device sends numbers line by line
            value = float(line)
            data_plot.append(value)
            if len(data_plot) > max_len:
                X = fft(data_plot)
                data_plot.pop(0)
                
    ax1.clear()
    ax1.plot(data_plot)

    N = len(X)
    n = np.arange(N)
    T = N/sr
    freq = n/T 

    #if X != []:
        #ax2.clear()
        #x2.plot(freq, np.abs(X), 'b')

    ax1.set_title('Live Serial Data')
    ax1.set_ylabel('Value')
    ax1.set_xlabel('Samples')

ani = animation.FuncAnimation(fig, animate, interval=100)

plt.show()