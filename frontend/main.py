import serial_reader
import data_processing
import threading
import tkinter as tk

from visualization import AnimatedWaterfall
from serial_reader import get_data


def run():
    root = tk.Tk()
    gui = AnimatedWaterfall(root)
    threading.Thread(target=get_data, args=(gui,), daemon=True).start()
    root.mainloop()

if __name__ == '__main__':
    run()


