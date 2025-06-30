from scipy.signal import spectrogram
from scipy.signal import chirp
import numpy as np
import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
from matplotlib import cm
from mpl_toolkits.mplot3d.art3d import Line3DCollection
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import tkinter as tk


class AnimatedWaterfall:
    def __init__(self, root):
        self.root = root
        self.root.title("Animated Waterfall Plot")

        self.fig = plt.figure(figsize=(14, 6))
        self.ax3d = self.fig.add_subplot(121, projection='3d')

        self.collections = []
        self.init = True
        self.cmap = cm.jet

        self.ax3d.set_box_aspect(aspect=(4, 8, 2))
        self.ax3d.set_ylabel('Frequency [Hz]')
        self.ax3d.set_xlabel('Time [s]')
        self.ax3d.set_zlabel('Amplitude [dB]')
        self.ax3d.set_title('3D Waterfall Spectrogram')

        self.ax3d.grid(True)

        # ---------------------
        self.ax1 = self.fig.add_subplot(322)
        self.raw_values = []
        # ---------------------
        self.ax2 = self.fig.add_subplot(324)
        self.fft_values = []
        # ---------------------
        self.ax3 = self.fig.add_subplot(326)
        self.conv_values = []

        self.fig.tight_layout()

        self.canvas = FigureCanvasTkAgg(self.fig, master=self.root)
        self.canvas_widget = self.canvas.get_tk_widget()
        self.canvas_widget.pack(fill=tk.BOTH, expand=True)

    def update_values3d(self, signal, sample_freq):
        signal_len = len(signal)
        nperseg = signal_len // 20
        noverlap = nperseg // 4
        # compute spectrogram
        freqs, times, Sxx = spectrogram(signal, fs=sample_freq,
                                        nperseg=nperseg, noverlap=noverlap)
        Sxx_dB = 10 * np.log10(Sxx + 1e-10)  # avoid log(0)
        # normalize amplitudes
        norm = plt.Normalize(Sxx_dB.min(), Sxx_dB.max())

        if self.init:
            for i, f in enumerate(freqs):
                x = times
                y = np.full_like(times, f)
                z = Sxx_dB[i, :]

                # color values
                points = np.array([x, y, z]).T.reshape(-1, 1, 3)
                segments = np.concatenate([points[:-1], points[1:]], axis=1)
                values = z[:-1]

                # init plot
                lc = Line3DCollection(segments, cmap=self.cmap, norm=norm)
                lc.set_array(values)
                lc.set_linewidth(1.5)
                self.ax3d.add_collection(lc)
                self.collections.append(lc)

                self.ax3d.set_xlim(times.min(), times.max())
                self.ax3d.set_ylim(freqs.min(), freqs.max())
                self.ax3d.set_zlim(Sxx_dB.min(), Sxx_dB.max())
        else:
            for i, f in enumerate(freqs):
                x = times
                y = np.full_like(times, f)
                z = Sxx_dB[i, :]

                points = np.array([x, y, z]).T.reshape(-1, 1, 3)
                segments = np.concatenate([points[:-1], points[1:]], axis=1)
                values = z[:-1]

                lc = self.collections[i]
                lc.set_segments(segments)
                lc.set_array(values)
                lc.norm = norm
            # update z-limits only if range has changed significantly
            z_min, z_max = Sxx_dB.min(), Sxx_dB.max()
            current_zlim = self.ax3d.get_zlim()
            if abs(current_zlim[0] - z_min) > 1 or abs(current_zlim[1] - z_max) > 1:
                self.ax3d.set_zlim(z_min, z_max)

    def update_values_raw(self, signal):
        if self.init:
            self.plot_raw,  = self.ax1.plot(range(len(signal)), signal)
        else:
            self.plot_raw.set_xdata(range(len(signal)))
            self.plot_raw.set_ydata(signal)
        
    def update_values_fft(self, fft):
        if self.init:
            self.plot_fft,  = self.ax2.plot(range(len(fft)), fft)
        else:
            self.plot_fft.set_xdata(range(len(fft)))
            self.plot_fft.set_ydata(fft)

    def update_values_conv(self, conv):
        if self.init:
            self.plot_conv, = self.ax3.plot(range(len(conv)), conv)
        else:
            self.plot_conv.set_xdata(range(len(conv)))
            self.plot_conv.set_ydata(range(len(conv)))


    def update_plot(self, signal, fft, conv, sample_freq):
        # calculate and update plot related values
        self.update_values3d(signal, sample_freq=sample_freq)
        self.update_values_raw(signal)
        self.update_values_fft(fft)
        self.update_values_conv(conv)

        self.canvas.draw()
        self.init = False
        pass
