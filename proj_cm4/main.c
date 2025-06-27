/******************************************************************************
* File Name:   main.c
*
* Description: This is the source code for the PDM PCM Example
*              for ModusToolbox.
*
* Related Document: See README.md
*
*
*******************************************************************************
* Copyright 2021-2022, Cypress Semiconductor Corporation (an Infineon company) or
* an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
*
* This software, including source code, documentation and related
* materials ("Software") is owned by Cypress Semiconductor Corporation
* or one of its affiliates ("Cypress") and is protected by and subject to
* worldwide patent protection (United States and foreign),
* United States copyright laws and international treaty provisions.
* Therefore, you may use this Software only as provided in the license
* agreement accompanying the software package from which you
* obtained this Software ("EULA").
* If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
* non-transferable license to copy, modify, and compile the Software
* source code solely for use in connection with Cypress's
* integrated circuit products.  Any reproduction, modification, translation,
* compilation, or representation of this Software except as specified
* above is prohibited without the express written permission of Cypress.
*
* Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
* reserves the right to make changes to the Software without notice. Cypress
* does not assume any liability arising out of the application or use of the
* Software or any product or circuit described in the Software. Cypress does
* not authorize its products for use in any products where a malfunction or
* failure of the Cypress product may reasonably be expected to result in
* significant property damage, injury or death ("High Risk Product"). By
* including Cypress's product in a High Risk Product, the manufacturer
* of such system or application assumes all risk of such use and in doing
* so agrees to indemnify Cypress against all liability.
*******************************************************************************/

#include "cyhal.h"
#include "cybsp.h"
#include "cy_retarget_io.h"
#include "COMPONENT_CMSIS_DSP/Include/dsp/transform_functions.h"
#include "COMPONENT_CMSIS_DSP/Include/dsp/statistics_functions.h"
#include "COMPONENT_CMSIS_DSP/Include/dsp/filtering_functions.h"

#include "stdlib.h"
#include "assert.h"
#include "math.h"

#include "ipc_communication.h"


#define SEND_IPC_MSG(x) ipc_msg.cmd = x; \
                        Cy_IPC_Pipe_SendMessage(USER_IPC_PIPE_EP_ADDR_CM0, \
                                                USER_IPC_PIPE_EP_ADDR_CM4, \
                                                (void *) &ipc_msg, 0);



static volatile uint8_t msg_cmd = 0;

/* IPC structure to be sent to CM0+ */
/* static ipc_msg_t ipc_msg = {
    .client_id  = IPC_CM4_TO_CM0_CLIENT_ID,
    .cpu_status = 0,
    .intr_mask  = USER_IPC_PIPE_INTR_MASK,
    .cmd        = IPC_CMD_INIT,
}; */

/**
 * @brief Define how many samples in a frame 
 * 
 */
#define FRAME_SIZE                  (1024)

/** 
 * @brief Noise threshold hysteresis 
 * 
 */
#define THRESHOLD_HYSTERESIS        0u

/**
 * @brief Volume ratio for noise and print purposes
 * 
 */
#define VOLUME_RATIO                (4*FRAME_SIZE)

/**
 * @brief Desired sample rate. Typical values: 8/16/22.05/32/44.1/48kHz
 * 
 */
#define SAMPLE_RATE_HZ              96000u


/**
 * @brief Decimation Rate of the PDM/PCM block. Typical value is 64
 * 
 */
#define DECIMATION_RATE             64u

/**
 * @brief Audio Subsystem Clock. Typical values depends on the desire sample rate:
 * - 8/16/48kHz    : 24.576 MHz
 * - 22.05/44.1kHz : 22.579 MHz
 *
 */
#define AUDIO_SYS_CLOCK_HZ          24576000u


/**
 * @brief PDM/PCM Data Pin
 *
 */
#define PDM_DATA                    P10_5

/**
 * @brief PDM/PCM Clock Pin
 *
 */
#define PDM_CLK                     P10_4

/**
 * @brief Amount of results returned from the FFT
 * 
 */
#define FFT_SIZE                    1024

/**
 * @brief the amount of amplitudes that we get after the FFT.
 * Because we discard the phases, we get only half the amount of values that were
 * stored in the original audio frame
 *
 */
#define AMPLITUDE_SIZE              FFT_SIZE/2

/**
 * @brief The first two values after the FFT are discarded, leaving us with 
 * FFT_SIZE/2 - 1 complex numbers.
 */
#define MAGNITUDES_SIZE             FFT_SIZE/2-1

/**
 * @brief the FFT provides us with complex numbers, represented by 2 floats
 * 
 */
#define COMPLEX_SIZE                FFT_SIZE/2


/**
 * @brief Frequency of the timer, so one tick lasts 1/FFT_TIMER_HZ
 * 
 */
#define FFT_TIMER_HZ                10000000u   // 10 MHz

/**
 * @brief The bandwidth to filter around
 * 
 */
#define BANDWIDTH_HZ                3000u       // 3 kHz

/**
 * @brief Frequency of the sent signal
 * 
 */
#define SIGNAL_FREQUENCY_HZ         41666u      // 41.6 kHz


/**
 * @brief The size of the signal that is used for convolution
 * 
 */
#define SLIDING_SIGNAL_SIZE       20u

/**
 * @brief How many bursts are sent
 * 
 */
#define BURST_SIZE                5

/* speed of sound in 21 degrees celsius (room temperature) */
#define SPEED_OF_SOUND            343.72

#define MAX_GENERATED_SIGNAL_SIZE 32



typedef struct {
    float32_t amplitude;
    float32_t phase;
} complex_t;

void pdm_pcm_isr_handler(void *arg, cyhal_pdm_pcm_event_t event);
void clock_init(void);
static void cm4_msg_callback(uint32_t *msg);

/* Interrupt flags */
volatile bool pdm_pcm_flag = true;

/* Volume variables */
uint32_t noise_threshold = THRESHOLD_HYSTERESIS;

/* HAL Object */
cyhal_pdm_pcm_t pdm_pcm;
cyhal_clock_t   audio_clock;
cyhal_clock_t   pll_clock;
cyhal_clock_t   fft_clock;
cyhal_timer_t fft_timer;
cyhal_timer_cfg_t timer_cfg = {
    .compare_value = 0,         // Not used for simple ticking
    .period = 0xFFFFFFFF,       // Max period for free-running
    .direction = CYHAL_TIMER_DIR_UP,
    .is_compare = false,
    .is_continuous = true,
    .value = 0
};

/* HAL Config */
const cyhal_pdm_pcm_cfg_t pdm_pcm_cfg =
{
    .sample_rate     = SAMPLE_RATE_HZ,
    .decimation_rate = DECIMATION_RATE,
    .mode            = CYHAL_PDM_PCM_MODE_STEREO,
    .word_length     = 16,  /* bits */
    .left_gain       = 0,   /* dB */
    .right_gain      = 0,   /* dB */
};

/* Message variables */
static volatile bool msg_flag = false;
static volatile uint32_t msg_value;
static volatile uint32_t start_tick;

/**
 * @brief Callback function to execute when receiving a message from CM0+ to CM4.
 * 
 * @param[in] msg Received message 
 */
static void cm4_msg_callback(uint32_t *msg)
{
    ipc_msg_t *ipc_recv_msg;

    if (msg != NULL)
    {
        start_tick = cyhal_timer_read(&fft_timer);
        /* Cast received message to the IPC message structure */
        ipc_recv_msg = (ipc_msg_t *) msg;

        msg_cmd = ipc_recv_msg->cmd;

    }
}


/**
 * @brief Generates the sent signal
 * 
 * @param f_sent                Frequency of the sent signal
 * @param f_sample              Sample rate of the microphone
 * @param bursts                Amount of bursts
 * @param output_signal         The output signal
 * @param max_output_length     Capacity of output_signal
 * 
 * @return length of the generated signal
 */
static uint32_t generate_sent_signal(uint32_t f_sent, uint32_t f_sample, uint32_t bursts, 
    float32_t *output_signal, uint32_t max_output_length) {
    /* the signal lasts bursts / f_sent, this is then mutliplied with f_sample to get the amount of samples in that signal */
    uint32_t sample_count =  (uint32_t) (2 * bursts);

    assert(sample_count <= max_output_length); 

    for (uint32_t i = 0; i < sample_count; i++) {
        // generates an alternating 1 and 0 signal
        output_signal[i] = (uint32_t) (i + 1) % 2;
    }

    return sample_count;
}

static void print_array(const float32_t *array, uint32_t size) {
    for (int i = 0; i < size; i++) {
        printf("%f,", array[i]);
    }
    printf("\n");
}

/**
 * @brief 
 * 
 * @param[in] frames 
 * @param[in] rfft 
 * @param[in] irfft 
 */
static void print_arrays(const float32_t *frames, const float32_t *rfft, const float32_t* filtered_rfft, const float32_t *irfft) {
    for (int i = 0; i < FFT_SIZE; i++) {
        if ( i % 2 == 0) {
            printf("%f,%f,%f,%f\n", frames[i], rfft[i], filtered_rfft[i], irfft[i]);
        } else {
            printf("%f,%f,%f,%f\n", frames[i], 0.0, filtered_rfft[i], irfft[i]);
        }
    }
}


/**
 * @brief Used to access and array of complex values containing frequency and phase
 * 
 * @note This requires some extra logic as the FFT documentation states: "The implementation is using 
 * a trick so that the output buffer can be N float : the last real is packaged in the imaginary 
 * part of the first complex (since this imaginary part is not used and is zero)."
 * 
 * @param complex   Array of amplitudes and phases with size FFT_SIZE
 * @param index     Which amplitude to access, can't be bigger than FFT_SIZE/2 - 1
 * @return A pointer to the value in the array
 */
static inline float32_t* access_amplitude(float32_t *complex, uint32_t index) {
    assert(index >= 0 && index < COMPLEX_SIZE);
    return index == (COMPLEX_SIZE - 1) ? &complex[1] : &complex[(index * 2) + 2];
}

/**
 * @brief Get the frequency by the index
 *
 * @param[in] index
 * @param[in] sample_rate
 *
 * @return Frequency for the selected index
 */
static inline float32_t get_frequency_by_index(uint32_t index, uint32_t sample_rate) {
    assert(index < COMPLEX_SIZE);
    return index * (sample_rate/2) / (COMPLEX_SIZE-1);
}

/**
 * @brief Get the index by frequency
 *
 * @param[in] frequency
 * @param[in] sample_rate
 *
 * @return Index for the selected frequency
 */
static inline uint32_t get_index_by_frequency(uint32_t frequency, uint32_t sample_rate) {
    assert(frequency <= SAMPLE_RATE_HZ);
    assert(frequency >= 0);
    return (COMPLEX_SIZE-1) * frequency / (sample_rate/2);
}

/* static void print_fft_results(const float32_t *array) {
    for (int i = 1; i < FFT_SIZE; i+=2) {
        printf("%f\n", fabs(array[i]));
    }
    printf("%f\n", fabs(array[1]));
    printf("\n\n\n");
} */

/**
 * @brief Filters the FFT amplitudes by using the sent_frequency
 * 
 * @note All amplitude and phase values belonging to frequencies outside the bandwidth around our
 * sent_frequency are set to 0.
 * 
 * @param[in,out]   complex_array     Complex values to filter
 * @param[in]       bandwidth         Bandwidth around the sent_frequency
 * @param[in]       sample_rate       The sample rate of the capturing device
 * @param[in]       sent_frequency    The frequency of the ultra sonic device
 */
static void filter_fft(float32_t *complex_array, uint32_t bandwidth, 
                       uint32_t sample_rate, uint32_t sent_frequency) {
    // width of one bucket
    const uint32_t bucket_width = sample_rate/2 / (COMPLEX_SIZE - 1);

    // figure out which slot contains the sent frequency, so that we can get the bandwidth around said frequency
    uint32_t bucket_index = get_index_by_frequency(sent_frequency, sample_rate);

    // get buckets that are above/below our sent frequency
    uint32_t r = bandwidth / bucket_width / 2;

    const uint32_t upper_index = bucket_index + r;
    const uint32_t lower_index = bucket_index - r;

    //printf("Bucket index: %lu, Lower index: %lu, Upper index: %lu, Sent frequency: %lu, Sample rate: %lu, Bandwidth: %lu, Bucket width: %lu\n", 
        //bucket_index, lower_index, upper_index, sent_frequency, sample_rate, bandwidth, bucket_width);

    for (uint32_t i = 2; i <= FFT_SIZE - 2; i += 2) {
        // set everything to 0 that is outside of our bandwidth
        if (i < lower_index || i > upper_index) {
            complex_t *complex = (complex_t *) &complex_array[i];
            complex->amplitude = 0;
            complex->phase = 0;
        }
    }
}

static void normalize_audio(float32_t *audio_frame_f32) {
    uint32_t p_min_index;
    float32_t min_audio;
    arm_min_f32(audio_frame_f32, FFT_SIZE, &min_audio, &p_min_index);

    uint32_t p_max_index;
    float32_t max_audio;
    arm_max_f32(audio_frame_f32, FFT_SIZE, &max_audio, &p_max_index);

    if (max_audio != min_audio) {
        for (int i = 0; i < FFT_SIZE; i++) {
            audio_frame_f32[i] = (audio_frame_f32[i] - min_audio) / (max_audio - min_audio);
        }
    }
}

/**
 * @brief Calculates the magnitude of a complex number
 * 
 * @param complex Needs to be an array of two floats
 * @return The magnitude of the complex number
 */
static inline float32_t calculate_magnitude(const float32_t *complex) {
    return sqrtf(complex[0] * complex[0] + complex[1] * complex[1]);
}

/* return the time in seconds */
static inline float32_t calculate_time_from_bucket(uint32_t bucket_index) {
    return (float32_t) (bucket_index) / (float32_t) SAMPLE_RATE_HZ;
}

/* return distance in meters */
static inline float32_t calculate_distance(float32_t time) {
    return (time * SPEED_OF_SOUND) / 2;
}

/*******************************************************************************
* Function Name: main
********************************************************************************
* Summary:
* The main function for Cortex-M4 CPU does the following:
*  Initialization:
*  - Initializes all the hardware blocks
*  Do forever loop:
*  - Check if PDM/PCM flag is set. If yes, report the current volume
*  - Update the LED status based on the volume and the noise threshold
*  - Check if the User Button was pressed. If yes, reset the noise threshold
*
* Parameters:
*  void
*
* Return:
*  int
*
*******************************************************************************/
int main(void)
{
    cy_rslt_t result;
    cy_en_ipc_pipe_status_t ipc_status;
    int16_t  audio_frame[FRAME_SIZE] = {0};

    setup_ipc_communication_cm4();

    /* Register the Message Callback */
    ipc_status = Cy_IPC_Pipe_RegisterCallback(USER_IPC_PIPE_EP_ADDR,
                    cm4_msg_callback,
                    IPC_CM0_TO_CM4_CLIENT_ID);
    if (ipc_status != CY_IPC_PIPE_SUCCESS)
    {
        CY_ASSERT(0);
    }

    /* Initialize the device and board peripherals */
    result = cybsp_init() ;
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    /* Enable global interrupts */
    __enable_irq();

    /* Init the clocks */
    clock_init();

    /* Initialize retarget-io to use the debug UART port */
    cy_retarget_io_init(CYBSP_DEBUG_UART_TX, CYBSP_DEBUG_UART_RX, CY_RETARGET_IO_BAUDRATE);

    /* Initialize the PDM/PCM block */
    cyhal_pdm_pcm_init(&pdm_pcm, PDM_DATA, PDM_CLK, &audio_clock, &pdm_pcm_cfg);
    cyhal_pdm_pcm_register_callback(&pdm_pcm, pdm_pcm_isr_handler, NULL);
    cyhal_pdm_pcm_enable_event(&pdm_pcm, CYHAL_PDM_PCM_ASYNC_COMPLETE, CYHAL_ISR_PRIORITY_DEFAULT, true);
    cyhal_pdm_pcm_start(&pdm_pcm);

    // Initialize the sync timer
    result = cyhal_timer_init(&fft_timer, NC, NULL);
    if (result != CY_RSLT_SUCCESS)
    {
        printf("Timer init failed");
        CY_ASSERT(0);
    }

    result = cyhal_timer_configure(&fft_timer, &timer_cfg);
        if (result != CY_RSLT_SUCCESS)
    {
        printf("Timer configure failed");
        CY_ASSERT(0);
    }
    result = cyhal_timer_set_frequency(&fft_timer, 10000000); // Match fft_clock frequency (10 MHz)
        if (result != CY_RSLT_SUCCESS)
    {
        printf("Timer set freq failed");
        CY_ASSERT(0);
    }
    result = cyhal_timer_start(&fft_timer);
        if (result != CY_RSLT_SUCCESS)
    {
        printf("Timer start failed");
        CY_ASSERT(0);
    }

    arm_rfft_fast_instance_f32 rfft_instance;
    arm_rfft_fast_init_f32(&rfft_instance, FFT_SIZE);

    float32_t generated_signal[MAX_GENERATED_SIGNAL_SIZE] = {0};
    uint32_t generated_signal_length = generate_sent_signal(SIGNAL_FREQUENCY_HZ, SAMPLE_RATE_HZ, 5,
        generated_signal, MAX_GENERATED_SIGNAL_SIZE);

    float32_t fft_results[FFT_SIZE] = {0};
    float32_t ifft_results[FFT_SIZE] = {0};
    float32_t audio_frame_f32[FFT_SIZE] = {0};

    /* we don't exactly know the exact length of the convolution as it depends on the generated signal, set it to max */ 
    float32_t convoluted_signal[FRAME_SIZE + MAX_GENERATED_SIGNAL_SIZE - 1] = {0};

    const uint32_t convoluted_signal_length = FRAME_SIZE + generated_signal_length - 1;

    // uint32_t start_t = 0;
    // uint32_t end_t = 0;

    for(;;)
    {
        switch (msg_cmd) {
            case IPC_START_S:  
                cyhal_pdm_pcm_clear(&pdm_pcm);  
                printf("\nS\n");
                /* Check if any microphone has data to process */
                // if (pdm_pcm_flag) 
                // {
                /* Clear the PDM/PCM flag */


                /* Setup to read the next frame */          

                /* 
                uint32_t total_read_audio_frames = 0;
                while (total_read_audio_frames < 1024) {
                    uint32_t read_audio_frames = FRAME_SIZE - total_read_audio_frames;
                    cyhal_pdm_pcm_read(&pdm_pcm, audio_frame+total_read_audio_frames, &read_audio_frames);
                    total_read_audio_frames += read_audio_frames;
                } 
                */

                // reset the flag to zero
                pdm_pcm_flag = 0;

                // do busy wait
                cyhal_pdm_pcm_read_async(&pdm_pcm, audio_frame, FRAME_SIZE);
                Cy_SysLib_Delay(10);

                while (pdm_pcm_flag == 0) {}

 
                // Convert to 32-bit float

                for (size_t i = 0; i < FFT_SIZE; i++)
                {
                    audio_frame_f32[i] = (float32_t)audio_frame[i];
                }

                normalize_audio(audio_frame_f32);

                #ifdef DEBUG
                // FOR DEBUGGING ONLY
                float32_t audio_frame_f32_to_print[FFT_SIZE];
                float32_t fft_to_print[FFT_SIZE];
                float32_t filtered_fft_to_print[FFT_SIZE];

                memcpy(&audio_frame_f32_to_print, &audio_frame_f32, FFT_SIZE * sizeof(float32_t));
                #endif
                
                // split the signal into its individual frequencies
                arm_rfft_fast_f32(&rfft_instance, audio_frame_f32, fft_results, 0);


                float32_t fft_magnitudes[511] = {0};
                uint32_t j = 0;
                for (int i = 2; i <= FFT_SIZE - 2; i += 2) {
                    assert(i != (FFT_SIZE - 1));
                    assert(j < FFT_SIZE/2 - 1);
                    fft_magnitudes[j] = calculate_magnitude(&fft_results[i]);
                    j += 1;
                }

                #ifdef DEBUG
                memcpy(&fft_to_print, &fft_results, FFT_SIZE * sizeof(float32_t));
                #endif

                // zero all unwanted frequencies
                filter_fft(fft_results, BANDWIDTH_HZ, SAMPLE_RATE_HZ, SIGNAL_FREQUENCY_HZ);

                float32_t filtered_magnitudes[511] = {0};
                uint32_t k = 0;
                for (int i = 2; i <= FFT_SIZE - 2; i += 2) {
                    assert(i != (FFT_SIZE - 1));
                    assert(k < FFT_SIZE/2 - 1);
                    filtered_magnitudes[k] = calculate_magnitude(&fft_results[i]);
                    k += 1;
                }

                #ifdef DEBUG
                memcpy(&filtered_fft_to_print, &fft_results, FFT_SIZE * sizeof(float32_t));
                #endif

                // do inverse FFT on the filtered signal
                arm_rfft_fast_f32(&rfft_instance, fft_results, ifft_results, 1);

                // do a convolution on the signal
                arm_conv_partial_f32(
                    generated_signal, generated_signal_length,
                    ifft_results, FFT_SIZE, 
                    convoluted_signal,
                    generated_signal_length - 1,
                    FFT_SIZE);


                float32_t conv_max = 0;
                uint32_t conv_max_index = 0;
                // find the maximum index of the convoluted signal
                arm_max_f32(convoluted_signal, convoluted_signal_length, &conv_max, &conv_max_index);
                float32_t time_of_flight = calculate_time_from_bucket(conv_max_index - generated_signal_length);
                float32_t distance = calculate_distance(time_of_flight);
                
                
                //printf("Time of flight: %f\n", time_of_flight);
                printf("D, %f\n", distance);
                // printf("Max Index: %d\n", conv_max_index);

                #ifdef DEBUG
                // print_arrays(audio_frame_f32_to_print, fft_to_print, filtered_fft_to_print, ifft_results);

                
                // RAW AUDIO
                printf("A,");
                print_array(audio_frame_f32_to_print, FFT_SIZE);

                // FFT output
                printf("T,");
                for (int i = 0; i < MAGNITUDES_SIZE; i++) {
                    printf("%f,", fft_magnitudes[i]);
                }
                printf("\n");

                // Filtered FFT
                printf("F,");
                for (int i = 0; i < MAGNITUDES_SIZE; i++) {
                    printf("%f,", filtered_magnitudes[i]);
                }
                printf("\n");


                // IFFT signal
                printf("I,");
                print_array(ifft_results, FFT_SIZE);
                

                // Convoluted signal
                printf("C,");
                print_array(convoluted_signal, 1024);
                
                
                #endif
                // printf("current time: %f\n", (float32_t) cyhal_timer_read(&fft_timer) / (float32_t) FFT_TIMER_HZ);

                // SEND_IPC_MSG(IPC_END_R);
            //}
        
        msg_cmd = 0;
        }
    }
}

/*******************************************************************************
* Function Name: pdm_pcm_isr_handler
********************************************************************************
* Summary:
*  PDM/PCM ISR handler. Set a flag to be processed in the main loop.
*
* Parameters:
*  arg: not used
*  event: event that occurred
*
*******************************************************************************/
void pdm_pcm_isr_handler(void *arg, cyhal_pdm_pcm_event_t event)
{
    (void) arg;
    (void) event;

    pdm_pcm_flag = true;
}

/*******************************************************************************
* Function Name: clock_init
********************************************************************************
* Summary:
*  Initialize the clocks in the system.
*
*******************************************************************************/
void clock_init(void)
{
    /* Initialize the PLL */
    cyhal_clock_reserve(&pll_clock, &CYHAL_CLOCK_PLL[0]);
    cyhal_clock_set_frequency(&pll_clock, AUDIO_SYS_CLOCK_HZ, NULL);
    cyhal_clock_set_enabled(&pll_clock, true, true);

    /* Initialize the audio subsystem clock (CLK_HF[1])
     * The CLK_HF[1] is the root clock for the I2S and PDM/PCM blocks */
    cyhal_clock_reserve(&audio_clock, &CYHAL_CLOCK_HF[1]);

    /* Source the audio subsystem clock from PLL */
    cyhal_clock_set_source(&audio_clock, &pll_clock);
    cyhal_clock_set_enabled(&audio_clock, true, true);
}

/* [] END OF FILE */