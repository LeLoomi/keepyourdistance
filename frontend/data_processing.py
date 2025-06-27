import numpy as np

BURST_CYCLES = 5

def rfft_on_data(data_array):
    return  np.abs(np.fft.rfft(data_array))

# applies a bandpass filter on frequency domain
# input is an array of amplitudes for the frequencies resulting from a RFFT and the bandwidth around the SENT_RATE that is kept
def bandpass_filter(amplitudes, bandwidth):
    frequencies = np.fft.rfftfreq(len(amplitudes), d=1/SAMPLING_RATE)
    filtered_amplitudes = np.zeros_like(amplitudes)

    for i, freq in enumerate(frequencies):
        if (freq > (SENT_RATE - bandwidth/2)) and (freq < (SENT_RATE + bandwidth/2)):
            filtered_amplitudes[i] = amplitudes[i]

    return filtered_amplitudes

def ifft_on_data(data_array):
    return np.fft.irfft(data_array)


def convolute_data(data_array):
    n = 2
    one_cycle = np.concatenate([np.ones(n), np.zeros(n)])
    burst_pattern = np.tile(one_cycle, BURST_CYCLES)

    return np.convolve(data_array, burst_pattern, mode='same')


def data_processing(data_array):
    rfft_data = rfft_on_data(data_array)
    filtered_data = bandpass_filter(rfft_data, 5000)
    ifft_data = ifft_on_data(filtered_data)
    conv_data = convolute_data(data_array)

    return [rfft_data, filtered_data, ifft_data, conv_data]
