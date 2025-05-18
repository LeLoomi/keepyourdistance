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

# Parameters
SR = 96000  
CARRIER_FREQ = 40000
BAND_FREQ = 50000 
BURST_CYCLES = 5
SPEED_OF_SOUND = 343  
MAX_LEN = 1000

port_name = serial_ports()
if port_name is None:
    port_name = '/dev/tty.usbmodem103'

ser = serial.Serial(port_name, 115200, timeout=1)

burst_len = int(SR/CARRIER_FREQ)
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

serial_data = []  # globale Liste für empfangene Daten

def calculate(data_np):
    fft_vals = np.fft.rfft(data_np)
    fft_freqs = np.fft.rfftfreq(len(data_np), 1/SR)

    
    # Bandpass filter (optional)
    band_mask = (fft_freqs >= CARRIER_FREQ - BAND_FREQ) & (fft_freqs <= CARRIER_FREQ + BAND_FREQ)

    # normalize
    max_val = np.max(np.abs(fft_vals[band_mask]))
    fft_vals_normalized = fft_vals/max_val


    filtered_fft_vals = np.zeros_like(fft_vals, dtype=complex)
    filtered_fft_vals[band_mask] = fft_vals[band_mask]

    # inverse FFT, to get filtered Signal
    data_filtered = np.fft.ifft(filtered_fft_vals)

    data_filtered = np.fft.ifft(data_filtered)
    convolution = np.convolve(data_filtered, burst_pattern, mode='same')

    start_index = data_np.index("X")
    time_peak = (np.argmax(convolution) - start_index) / SR
    distance = time_peak * SPEED_OF_SOUND
    distance_var.set(f"Distanz: {distance:.3f} m")

    # Plots updaten
    axs[1].cla()
    axs[1].set_ylim(0,1)
    axs[1].plot(fft_freqs, np.abs(fft_vals_normalized), color='green')
    axs[1].set_title("FFT")
    axs[1].set_ylabel("Amplitude")

    axs[2].cla()
    corr_time = np.arange(len(convolution)) / SR
    axs[2].plot(corr_time, convolution, color='orange')
    axs[2].set_title("Faltung mit Referenzsignal")
    axs[2].set_xlabel("Zeit (s)")
    axs[2].set_ylabel("Amplitude")

def update_plot():
    global serial_data

    while ser.in_waiting:
        try:
            line = ser.readline().decode('utf-8').strip()
            if line:  
                if line == "START_S":
                    serial_data.append("X")
                else:
                    val = float(line)
                    serial_data.append(val)
                    print(line)

                if len(serial_data) > MAX_LEN:
                    serial_data.pop(0)
        except Exception as e:
            print(f"Fehler beim Lesen: {e}")

    if len(serial_data) >= MAX_LEN:
        calculate(serial_data)

    axs[0].cla()
    axs[0].plot(serial_data, color='blue')
    axs[0].set_title("Normalisiertes empfangenes Signal (Amplitude)")
    axs[0].set_ylabel("Normierte Amplitude")

    canvas.draw()
    root.after(100, update_plot)

update_plot()
root.mainloop()