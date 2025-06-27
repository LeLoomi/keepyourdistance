import sys
import glob
import serial
import numpy as np

from data_processing import data_processing


# Parameters
SPEED_OF_SOUND = 343  

DATA_BUFFER_SIZE = 100

SENT_RATE = 41666
SAMPLING_RATE = 96000
FRAME_LENGTH = 1024

PLOT_OFFSET = 2


def get_port():
    """ Lists serial port names

        :raises EnvironmentError:
            On unsupported or unknown platforms
        :returns:
            A list of the serial ports available on the system if some ports to listen to are available
            otherwise return None
    """
    if sys.platform.startswith('win'):
        # windows
        ports = ['COM%s' % (i + 1) for i in range(256)]
    elif sys.platform.startswith('linux') or sys.platform.startswith('cygwin'):
        # linux
        # this excludes your current terminal "/dev/tty"
        ports = glob.glob('/dev/tty[A-Za-z]*')
    elif sys.platform.startswith('darwin'):
        # mac os x
        ports = glob.glob('/dev/tty.*')
    else:
        raise EnvironmentError('Unsupported platform')

    result = []
    for port in ports:
        try:
            s = serial.Serial(port)
            s.close()
            result.append(port)
            print(port)
        except (OSError, serial.SerialException):
            pass
    if len(result):
        return result[0]
    return None


def normalize(array, coef = 0):
    min_value = np.min(array)
    max_value = np.max(array)
    if coef == 0:
        coef = (max_value - min_value)
    for i in range(len(array)):
        array[i] = (array[i] - min_value)/ coef

    return coef, array


def get_data(gui):
    port = get_port()
    if port is None:
        port = '/dev/tty.usbmodem2103'

    ser = serial.Serial('/dev/tty.usbmodem2103', 115200, timeout=1)

    distance_array = []
    
    # data array for recieved data from serial port 
    # psoc_data_array = [[<Raw Signal>], [<FFT Transformed Signal - Frequnecy>], [<FFT Transformed Signal - Amplitude>], ...
    # ... [<Filtered Signal - Frequnecy>], [<Filtered Signal - Amplitude>], [Time Domain Filtered Signal], [Convoluted Signal]]
    psoc_data_array = np.empty(shape=(DATA_BUFFER_SIZE, 7, 1024), dtype=np.float32)
    cycle = 0
    start_var = False

    try:
        while ser.in_waiting:
            line = ser.readline().decode('utf-8', errors='ignore').strip()

            if line.startswith('S'):
                if start_var:
                    if cycle == 1:
                        gui.update_plot(psoc_data_array[1, 0, :], SAMPLING_RATE)   
                        cycle += 1  
                start_var = True


            if len(line) < 3:
                continue
            
            data_type = line[0]
            print(data_type)
            data_string = line[2:]

            try:
                if start_var:
                    if data_type in ['D']:
                        d = float(data_string)
                        distance_array.append(d)
                    if data_type in ['A', 'T', 'F', 'I', 'C']:
                        data_array = np.array([float(x) for x in data_string.split(",") if x.strip()])
                        if data_type != 'C':
                            _, norm_data_array = normalize(data_array)

                    # filling the psoc_data_array
                    match data_type:   
                        # raw audio signal                
                        case 'A':                        
                            psoc_data_array[cycle % DATA_BUFFER_SIZE, 0, :] = norm_data_array                                                    

                        # real fast fourir transformed signal
                        case 'T':                                                     
                            x_values = np.linspace(0, FRAME_LENGTH, FRAME_LENGTH)
                            x_freq = x_values * (SAMPLING_RATE/len(data_array))

                            padded_array = np.zeros(1024)
                            padded_array[:len(data_array)] = data_array

                            psoc_data_array[cycle % DATA_BUFFER_SIZE, 1, :] = x_freq
                            psoc_data_array[cycle % DATA_BUFFER_SIZE, 2, :] = padded_array

                        # frequency domain filtered signal
                        case 'F':                  
                            x_values = np.linspace(0, FRAME_LENGTH, FRAME_LENGTH)
                            x_freq = x_values * (SAMPLING_RATE/len(data_array))
                            
                            padded_array = np.zeros(1024)
                            padded_array[:len(data_array)] = data_array
                            
                            psoc_data_array[cycle % DATA_BUFFER_SIZE, 3, :] = x_freq
                            psoc_data_array[cycle % DATA_BUFFER_SIZE, 4, :] = padded_array

                        # time domain filtered signal (after inverse FFT)
                        case 'I': 
                            psoc_data_array[cycle % DATA_BUFFER_SIZE, 5, :] = norm_data_array

                        # convoluted signal
                        case 'C': 
                            _, norm_data_array = normalize(data_array[40:])

                            padded_array = np.zeros(1024)
                            padded_array[:len(norm_data_array)] = norm_data_array

                            psoc_data_array[cycle % DATA_BUFFER_SIZE, 6, :] = x_freq
                                                    
                        case _:
                            continue  

            except ValueError:
                continue
    

    except Exception as e:
            print("Error while reading the serial port:", e)
    
    
