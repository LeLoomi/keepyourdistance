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
SAMPLING_RATE = 96000
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

# Set titles/labels
titles = ["Raw 🦕 Signal", "Fourier Transformed Signal", "Filtered Signal"]
ylims = [(-0.1, 1), (None, None), (-0.1, 1)]


lines = []
for ax, title, ylim in zip(axs, titles, ylims):
    ax.set_title(title)
    ax.set_ylabel("Amplitude")
    line, = ax.plot(np.zeros(FRAME_LENGTH))
    lines.append(line)
    if ylim[0] is not None and ylim[1] is not None:
        ax.set_ylim(*ylim)

plt.tight_layout()


def update_plot():
    try:
        while ser.in_waiting:
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            if not line or len(line) < 3:
                continue

            array_type = line[0]
            data_string = line[2:]

            try:
                data_array = np.fromstring(data_string, dtype=float, sep=",")
                if data_array.size < FRAME_LENGTH:
                    continue
            except ValueError:
                continue

            match array_type:
                case 'A':
                    lines[0].set_ydata(data_array)
                case 'T':
                    x_values = np.linspace(0, len(data_array), len(data_array))
                    x_freq = x_values * ((SAMPLING_RATE/2)/len(data_array))
                    lines[1].set_xdata(x_freq)
                    lines[1].set_ydata(data_array)
                case 'F':
                    filt_array = data_array[1:]
                case 'I':
                    lines[2].set_ydata(data_array)
                case _:
                    continue

    except Exception as e:
        print("Fehler beim Lesen:", e)

    canvas.draw()
    root.after(100, update_plot)

update_plot()
root.mainloop()