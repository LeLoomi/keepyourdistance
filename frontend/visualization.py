from scipy.signal import spectrogram
import numpy as np
import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import seaborn as sns
import serial
import tkinter as tk
from tkinter import ttk



def waterfall_3d(signal, sample_freq):
    # calculate spectrogram using stft
    freqs, time, Sxx = spectrogram(signal, fs=sample_freq)

    # waterfallplot as a 3d plot
    fig = plt.figure(figsize=(10, 6))
    ax = fig.add_subplot(111, projection='3d')

    for i, time_slice in enumerate(time):
        ax.plot(freqs, [time_slice]*len(freqs), Sxx[:, i])

    ax.set_xlabel('Frequency [Hz]')
    ax.set_ylabel('Time [sec]')
    ax.set_zlabel('Amplitude')
    ax.set_title('Waterfall')

    plt.tight_layout()
    plt.show(block=True)

    # spectrogram as 2d plot
    plt.pcolormesh(time, freqs, Sxx)
    plt.ylabel('Frequency [Hz]')
    plt.xlabel('Time [sec]')
    plt.show(block=True)

    pass

