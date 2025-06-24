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
import threading


class AnimatedWaterfall:
    def __init__(self, root, signal, sample_freq):
        self.root = root
        self.root.title("Animated Waterfall Plot")

        self.fig = plt.figure(figsize=(14, 6))
        self.ax3d = self.fig.add_subplot(111, projection='3d')

        self.collections = []
        self.init = True
        self.update_values(signal, sample_freq)

        self.ax3d.set_box_aspect(aspect=(4, 8, 1))
        self.ax3d.set_ylabel('Frequency [Hz]')
        self.ax3d.set_xlabel('Time [s]')
        self.ax3d.set_zlabel('Amplitude [dB]')
        self.ax3d.set_title('3D Waterfall Spectrogram')

        self.ax3d.grid(True)
        self.fig.tight_layout()
        self.ax3d.legend()

        self.canvas = FigureCanvasTkAgg(self.fig, master=self.root)
        self.canvas_widget = self.canvas.get_tk_widget()
        self.canvas_widget.pack(fill=tk.BOTH, expand=True)

    def update_values(self, signal, sample_freq):
        # compute spectrogram
        freqs, times, Sxx = spectrogram(signal, fs=sample_freq)
        Sxx_dB = 10 * np.log10(Sxx + 1e-10)  # avoid log(0)

        # normalize amplitudes
        norm = plt.Normalize(Sxx_dB.min(), Sxx_dB.max())
        cmap = cm.jet

        for i, f in enumerate(freqs):
            x = times
            y = np.full_like(times, f)
            z = Sxx_dB[i, :]

            # color values
            points = np.array([x, y, z]).T.reshape(-1, 1, 3)
            segments = np.concatenate([points[:-1], points[1:]], axis=1)
            values = z[:-1]

            if self.init:
                # init plot: add collections to the axis
                lc = Line3DCollection(segments, cmap=cmap, norm=norm)
                lc.set_array(values)
                lc.set_linewidth(1.5)
                self.ax3d.add_collection(lc)
            else:
                # manipulate existing collections
                self.ax3d.collections[i].set_segments(segments)
                self.ax3d.collections[i].set_array(values)
                self.ax3d.collections[i].cmap = cmap
                self.ax3d.collections[i].norm = norm

        self.ax3d.set_ylim(freqs.min(), freqs.max())
        self.ax3d.set_xlim(times.min(), times.max())
        self.ax3d.set_zlim(Sxx_dB.min(), Sxx_dB.max())
        self.init = False

    def update_plot(self, signal, sample_freq):
        # calculate and update plot related values
        self.update_values(signal, sample_freq)

        self.canvas.draw()


def animate(gui):
    fs = 1000  # sampling frequency in Hz
    t = np.linspace(0, 5, fs * 5)
    for i in range(1, 1000, 10):      
        signal = chirp(t, f0=10*i, f1=500, t1=5, method='linear')

        gui.update_plot(signal, fs)


if __name__ == '__main__':
    fs = 1000  # sampling frequency in Hz
    t = np.linspace(0, 5, fs * 5)
    signal = chirp(t, f0=10, f1=500, t1=5, method='linear')

    root = tk.Tk()
    gui = AnimatedWaterfall(root, signal, fs)

    threading.Thread(target=animate, args=(gui,), daemon=True).start()

    root.mainloop()
