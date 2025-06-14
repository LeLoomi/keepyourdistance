import numpy as np
import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import seaborn as sns
import serial
import tkinter as tk
from tkinter import ttk


# Parameters
SPEED_OF_SOUND = 343  
SENT_RATE = 44000
SAMPLING_RATE = 96000
FRAME_LENGTH = 1024

ser = serial.Serial('/dev/tty.usbmodem2103', 115200, timeout=1)

sns.set_theme(style="darkgrid")
root = tk.Tk()
root.title("Akustische Distanzmessung")

distance_var = tk.StringVar()
ttk.Label(root, textvariable=distance_var, font=("Helvetica", 16)).pack(pady=5)

fig, axs = plt.subplots(4, 1, figsize=(12, 8), sharex=False)
canvas = FigureCanvasTkAgg(fig, master=root)
canvas.get_tk_widget().pack()

# Set titles/labels
titles = ["Raw Signal", "Fourier Transformed Signal","Filtered Signal FFT", "Filtered Signal"]


lines = []
for ax, title in zip(axs, titles):
    ax.set_title(title)
    ax.set_ylabel("Amplitude")
    ax.set_ylim(-0.1,1)
    line, = ax.plot(np.zeros(FRAME_LENGTH))
    lines.append(line)


axs[1].set_xlim(-10, 44000)
axs[1].set_ylim(-5, 250)
axs[2].set_xlim(-10, 44000)
axs[2].set_ylim(-5, 250)
axs[3].set_xlim(-1, 1040)
axs[3].set_ylim(-1, 10)


def update_plot():
    try:
        while ser.in_waiting:
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            if not line or len(line) < 100:
                continue

            array_type = line[0]
            data_string = line[2:]

            try:
                float_list = [float(x) for x in data_string.split(",") if x.strip()]
                data_array = np.array(float_list)
            except ValueError:
                continue

            match array_type:
                case 'A': # raw audio signal
                    lines[0].set_ydata(data_array)
                case 'T': # FFT transformed signal
                    x_values = np.linspace(0, FRAME_LENGTH, FRAME_LENGTH)
                    x_freq = x_values * (SAMPLING_RATE/len(data_array))
                    lines[1].set_xdata(x_freq)

                    help_array = np.append(data_array, np.zeros(513))
                    lines[1].set_ydata(help_array)
                case 'F': # filtered signal
                    x_values = np.linspace(0, FRAME_LENGTH, FRAME_LENGTH)
                    x_freq = x_values * (SAMPLING_RATE/len(data_array))
                    lines[2].set_xdata(x_freq)

                    help_array = np.append(data_array, np.zeros(513))
                    lines[2].set_ydata(help_array)
                case 'I': # inverded filtered signal
                    lines[2].set_ydata(data_array)
                    #continue
                case 'C': # convoluted signal
                    lines[3].set_ydata(data_array)
                case _:
                    continue

    except Exception as e:
        print("Fehler beim Lesen:", e)

    canvas.draw()
    root.after(100, update_plot)

update_plot()
root.mainloop()