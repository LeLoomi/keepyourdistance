# Prototype for Blood Pressure Measuring
This prototype was created in the context of the Telekom MMS Health Hackathon for the estimation of blood pressure by using sound emissions.

## Motivation
Blood pressure measurements were done with complicated devices that require time and a cuff around the arm. Recently smarter devices have emerged on the market that measure the blood pressure by utilizing light (PPG). This approach is more prone to error, so a need for a better methodology arises.

Blood pressure measurements can be done with sound to calculate the thickness of the artery. The actual measurements in tissue require a higher frequency in the MHz range. This is unfeasible with our current equipment that operates in the KHz frequency. The idea is to develop a prototype that is capable of distance measurements in air using a microphone and a piezo element for sound emission.

## Idea
To calculate the distance to the wall, we utilize Time of Flight (ToF) measurements. This can be done by emitting a signal at a certain frequency (41.6 kHz) and listening for the reflection of said frequency with the microphone. Knowing the point of sending we can then infer the ToF for the signal to the wall and back, divide it by 2 and combine it with the speed of sound (approx. 343 m/s) to get an accurate distance to the wall. 

## Hardware
For our prototype we have a PSoC 6 AI from Infineon. The PSoC possesses a microphone for sampling up to 96 kHz. For our sound emission a piezo element is used, that is controlled by 2 GPIO pins.

## Software
The PSoC runs with a C program that can be flashed on the chip with Infineon's ModusToolbox. The PSoC has 2 cores: CM0P with a lower frequency and the CM4 with a higher frequency. The former runs the code that is responsible for sending the signal and the latter for processing the incoming signal captured by the microphone. The two cores can communicate with Inter Process Communication.

## Practical Signal Processing
Sending the 41.6 kHz is rather simple but is done in 5 bursts to facilitate later steps. The actual computationally heavy task is processing the incoming signal, filtering it and turning it into a form that allows ToF measurements. 

### Raw Signal
First of all, the signal is captured by the microphone at a frequency of 96 kHz. For the extraction of the data into the program 1024 samples are taken at once from the microphone. With the 96 kHz that equates to a total capture time $t_\text{Capture}$ for 1024 samples

$$
t_{\text{Capture}} = \frac{\text{Sample Count}}{\text{Sample Rate}} = \frac{1024}{96000 Hz} \approx 11 ms.
$$

### Fast Fourier Transformation
The FFT is applied to turn the raw audio signal into the frequency domain. This gives us an array of amplitudes and phases, where each amplitude and phase correspond to one frequency in the raw signal.

### Band Pass Filter
The Band Pass Filter is applied to filter all unwanted frequency that are not within a bandwidth around our desired sent frequency (41.6 kHz).

### Inverse FFT
With the signal now split up into frequencies and filtered we can apply the Inverse FFT (IFFT) to convert the signal back into the time domain. This allows us to get timing information for ToF calculation.

### Convolution
A convolution is used to highlight the exact position of the sent signal in the time domain. The sent signal is known (5 bursts of High-Low) and is convoluted with the IFFT signal. This gives us a spike at the reception of the reflected signal.

The 5 burst pattern make convoluting the signal less error prone as false positives are ruled out.

### Time of Flight / Distance Measurements
The Time of Flight can be estimated by looking at the amount of samples that were received until the convolutional peak was observed. The reflection of the signal is guaranteed to be in the 1024 samples of the audio buffer. This can be shown with the capture time $t_{\text{Capture}}$ derived earlier and calculating the maximum distance sound can travel in this time

$$
\text{Max Distance} = \text{Speed of Sound} \times t_{\text{Capture}} = 343 m/s \times 11 ms = 3.7 m.
$$

For measurements below 3.5 meters the sent signal is guaranteed to be inside the 1024 audio samples (Note: the effective distance that can be measured with our method is way below this value because of scatter).

The sample that contains the peak in the convolution is the one that can be used for time calculation. This give us

$$
\text{ToF} = \frac{\text{Sample Number}}{\text{96000 Hz}}.
$$

ToF is the round trip time for the signal. So for the distance measurements we are required to divide it by 2.
Therefore,

$$
\text{Distance} = \frac{\text{ToF} \times \text{Speed of Sound}}{2}.
$$

The actual distance contains a small but fixed offset that needs to be subtracted to get an accurate distance measurement.

## How to get it running
The setup is best tested on Windows and requires the ModusToolbox. Our program can be flashed onto the PSoC by using the `make program` command. The piezo element needs to be connected to GPIO `0` and `1`.

For a visual representation of the incoming data a python script can be launched.


