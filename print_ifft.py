import numpy as np
import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import seaborn as sns
import serial
import tkinter as tk
from tkinter import ttk
import json

# Parameters
SPEED_OF_SOUND = 343  
FRAME_LENGTH = 1024

ser = serial.Serial('/dev/tty.usbmodem11203', 115200, timeout=1)

sns.set_theme(style="darkgrid")
root = tk.Tk()
root.title("Akustische Distanzmessung")

distance_var = tk.StringVar()
ttk.Label(root, textvariable=distance_var, font=("Helvetica", 16)).pack(pady=5)

fig, axs = plt.subplots(3, 1, figsize=(12, 8), sharex=False)

canvas = FigureCanvasTkAgg(fig, master=root)
canvas.get_tk_widget().pack()

# Initialize plots 
axs[0].set_title("Raw 🦕 Signal")
axs[0].set_ylabel("Amplitude strength normalized")

axs[1].set_title("Fourier Transformed Signal")
axs[1].set_ylabel("Amplitude strength")

axs[2].set_title("Filtered Signal")
axs[2].set_ylabel("Amplitude")

plt.tight_layout()

def update_plot():
    raw_array = np.zeros(1024)
    fft_array = np.zeros(1024)
    filt_array = np.zeros(1024)
    ifft_array = np.zeros(1024)

    while ser.in_waiting:
        try:
            line = ser.readline().decode('utf-8').strip()
            if line:  
                    array_type = str(line)[0]
                    data_string = str(line)[2:]
                    data_array = np.fromstring(data_string, dtype=float, sep=",")

                    print("Received:", data_string)
                    print("Received:", len(data_array))

                    match array_type: 
                        case 'A':
                              raw_array = data_array[1:]
                        case 'T':
                              fft_array = data_array[1:]
                        case 'F':
                              filt_array = data_array[1:]
                        case 'I':
                              ifft_array = data_array[1:]
                        case _:
                              continue
                        
        except Exception as e:
            print(f"Fehler beim Lesen: {e}")


    # raw 🦕 signal
    axs[0].cla()
    axs[0].plot(raw_array, color='blue', linewidth=1)
    axs[0].set_ylim(-0.1,1)
    
    # FFT Signal 
    axs[1].cla()
    axs[1].plot(fft_array, color='green', linewidth=1)
    axs[1].set_ylim(-0.1,1)

    # FFT Signal 
    axs[2].cla()
    axs[2].plot(ifft_array, color='red', linewidth=1)

    canvas.draw()
    root.after(100, update_plot)

update_plot()
root.mainloop()

