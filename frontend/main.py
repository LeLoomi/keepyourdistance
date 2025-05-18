import numpy as np
import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import seaborn as sns
import serial
import tkinter as tk
from tkinter import ttk
from get_ports import serial_ports

# Important Parameters
SR = 48000  # Sampling Rate a
CARRIER_FREQ = 20000
BAND_FREQ = 3000 
BURST_CYCLES = 5
SPEED_OF_SOUND = 343  # m/s
MAX_LEN = 2048

port_name = serial_ports()
if port_name is None:
    port_name = '/dev/tty.usbmodem103'

ser = serial.Serial(port_name, 115200, timeout=1)

# second signal for convolution
burst_len = (SR / CARRIER_FREQ)
one_cycle = np.concatenate([np.ones(burst_len), np.zeros(burst_len)])
burst_pattern = np.tile(one_cycle, BURST_CYCLES)

sns.set_theme(style="darkgrid")
root = tk.Tk()
root.title("Akustische Distanzmessung")

distance_var = tk.StringVar()
ttk.Label(root, textvariable=distance_var, font=("Helvetica", 16)).pack(pady=5)

fig, axs = plt.subplots(3, 1, figsize=(12, 8), sharex=False)
canvas = FigureCanvasTkAgg(fig, master=root)
canvas.get_tk_widget().pack()

data_plot = []

def update_plot():
    global data_plot

    # seriell data reading
    while ser.in_waiting:
        try:
            line = ser.readline().decode('utf-8').strip()
            val = float(line)
            data_plot.append(val)
            if len(data_plot) > MAX_LEN:
                # discarding values if array ull enoght for window
                data_plot.pop(0)
        except:
            pass

    # nomalize raw data to amplitude between [0,1]
    data_np = np.array(data_plot)
    norm_data = (data_np - data_np.min()) / (data_np.max() - data_np.min())

    # FFT calcualation
    fft_vals = np.fft.rfft(norm_data)
    fft_freqs = np.fft.rfftfreq(len(norm_data), 1/SR)

    # Bandpass filter
    band_mask = (fft_freqs >= CARRIER_FREQ - BAND_FREQ) & (fft_freqs <= CARRIER_FREQ + BAND_FREQ)
    filtered_fft = fft_vals[band_mask]
    filtered_freqs = fft_freqs[band_mask]

    convolution = np.convolve(norm_data,filtered_fft) 

    # plotting normalized data 
    axs[0].clear()
    axs[0].plot(norm_data, color='blue')
    axs[0].set_title("Normalisiertes empfangenes Signal (Amplitude)")
    axs[0].set_ylabel("Normierte Amplitude")
    axs[0].set_xlim(0, len(norm_data))

    # plotting fft 
    axs[1].clear()
    mask = fft_freqs <= CARRIER_FREQ #masking othe rvalues out because of niquist
    axs[1].plot(fft_freqs[mask], np.abs(fft_vals[mask]), color='green')
    axs[1].set_title("FFT (up to 20kHz)")
    axs[1].set_ylabel("Amplitude")
    axs[1].set_xlim(0, CARRIER_FREQ)

    # plotting convolution
    axs[2].clear()
    corr_time = np.arange(len(corr)) / SR
    axs[2].plot(corr_time, convolution, color='orange')
    #axs[2].axvline(time_delay, color='red', linestyle='--', label=f"Echo Peak bei {time_delay*1000:.2f} ms")
    axs[2].set_title("Convolution with referenz signal")
    axs[2].set_xlabel("time (s)")
    axs[2].set_ylabel("Convolution")
    axs[2].legend()
    axs[2].set_xlim(0, corr_time[-1])

    canvas.draw()

    time_peak = np.argmax(convolution) * 1/SR
    distance = time_peak * SPEED_OF_SOUND

    distance_var.set(f"Distanz: {distance:.3f} m")

    root.after(100, update_plot)

update_plot()
root.mainloop()
