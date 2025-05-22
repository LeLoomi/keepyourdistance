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

#include "stdlib.h"
#include "assert.h"

#include "ipc_communication.h"


#define SEND_IPC_MSG(x) ipc_msg.cmd = x; \
                        Cy_IPC_Pipe_SendMessage(USER_IPC_PIPE_EP_ADDR_CM0, \
                                                USER_IPC_PIPE_EP_ADDR_CM4, \
                                                (void *) &ipc_msg, 0);

static void cm4_msg_callback(uint32_t *msg);

static volatile uint8_t msg_cmd = 0;

/* IPC structure to be sent to CM0+ */
static ipc_msg_t ipc_msg = {
    .client_id  = IPC_CM4_TO_CM0_CLIENT_ID,
    .cpu_status = 0,
    .intr_mask  = USER_IPC_PIPE_INTR_MASK,
    .cmd        = IPC_CMD_INIT,
};

/*******************************************************************************
* Macros
********************************************************************************/
/* Define how many samples in a frame */
#define FRAME_SIZE                  (1024)
/* Noise threshold hysteresis */
#define THRESHOLD_HYSTERESIS        0u
/* Volume ratio for noise and print purposes */
#define VOLUME_RATIO                (4*FRAME_SIZE)
/* Desired sample rate. Typical values: 8/16/22.05/32/44.1/48kHz */
#define SAMPLE_RATE_HZ              96000u
/* Decimation Rate of the PDM/PCM block. Typical value is 64 */
#define DECIMATION_RATE             64u
/* Audio Subsystem Clock. Typical values depends on the desire sample rate:
- 8/16/48kHz    : 24.576 MHz
- 22.05/44.1kHz : 22.579 MHz */
#define AUDIO_SYS_CLOCK_HZ          24576000u
/* PDM/PCM Pins */
#define PDM_DATA                    P10_5
#define PDM_CLK                     P10_4

#define FFT_SIZE                    1024

/**
 * @brief the amount of amplitudes that we get after the FFT.
 * Because we discard the phases, we get only half the amount of values that were
 * stored in the original audio frame
 */
#define AMPLITUDE_SIZE              FFT_SIZE/2

#define FFT_TIMER_HZ                10000000u   // 10 mHz

#define BANDWIDTH_HZ                3000u       // 3 kHz

/*******************************************************************************
* Function Prototypes
********************************************************************************/
void pdm_pcm_isr_handler(void *arg, cyhal_pdm_pcm_event_t event);
void clock_init(void);
void clock_init_fft(void);

/*******************************************************************************
* Global Variables
********************************************************************************/
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

/*******************************************************************************
* Function Name: cm4_msg_callback
********************************************************************************
* Summary:
*   Callback function to execute when receiving a message from CM0+ to CM4.
*
* Parameters:
*   msg: message received
*
*******************************************************************************/
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
 * @brief Get the frequency by the index
 *
 * @param[in] index
 * @param[in] sample_rate
 *
 * @return Frequency for the selected index
 */
static inline float32_t get_frequency_by_index(uint32_t index, uint32_t sample_rate) {
    assert(index < FFT_SIZE/2);
    return index * sample_rate / (FFT_SIZE/2);
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
    return (FFT_SIZE/2) * frequency / sample_rate;
}

static void print_fft_results(const float32_t *array) {
    for (int i = 1; i < FFT_SIZE; i+=2) {
        printf("%f\n", fabs(array[i]));
    }
    printf("%f\n", fabs(array[1]));
    printf("\n\n\n");
}

/**
 * @brief Filters the FFT amplitudes by using the sent_frequency
 * 
 * @note All frequency values outside the bandwidth are set to 0
 * 
 * @param[in,out]   amplitudes        Amplitudes to filter
 * @param[in]       bandwidth         Bandwidth around the sent_frequency
 * @param[in]       sample_rate       The sample rate of the capturing device
 * @param[in]       sent_frequency    The frequency of the ultra sonic device
 */
static void filter_fft(const float32_t *amplitudes, uint32_t bandwidth, 
                       uint32_t sample_rate, uint32_t sent_frequency) {
    // width of one bucket
    const uint32_t bucket_width = sample_rate/(AMPLITUDE_SIZE);

    // figure out which slot contains the sent frequency, so that we can get the bandwidth around said frequency
    uint32_t bucket_index = get_index_by_frequency(sent_frequency, sample_rate);

    // get buckets that are above/below our sent frequency
    uint32_t r = bandwidth / bucket_width / 2;

    uint32_t upper_index = bucket_index + r;
    uint32_t lower_index = bucket_index - r;

    for (uint32_t i = 0; i < AMPLITUDE_SIZE; i++) {
        // set everything to 0 that is outside of our bandwidth
        if (i < lower_index && i > upper_index) {
            amplitudes[i] = 0.0f;
        }
    }
}

/**
 * @brief Converts the fft_results containing signed amplitudes and frequencies to
 * absolute amplitudes only
 *
 * @param[in]   fft_results  Array of amplitudes and phases
 * @param[out]  amplitudes   Is filled with only amplitudes, it must have half of the capacity of fft_results
 *
 * @note The FFT documentation states: The implementation is using a trick so that the output
 * buffer can be N float : the last real is packaged in the imaginary part of the first
 * complex (since this imaginary part is not used and is zero).
 */
static void convert_to_amplitudes(const float32_t *fft_results, float32_t *amplitudes) {
    for (size_t i = 2; i < FFT_SIZE; i += 2) {
        amplitudes[i] = fabs(fft_results[i]);
    }

    // the last real is packaged in the imaginary part of the first complex (since this imaginary part is not used and is zero).
    amplitudes[FFT_SIZE/2 - 1] = fabs(fft_results[1]);

    // ftt_results[0] is the DC
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

    int v_index = 0;

    arm_rfft_fast_instance_f32 rfft_instance;
    arm_rfft_fast_init_f32(&rfft_instance, FFT_SIZE);

    float32_t results[FFT_SIZE] = {0};
    // uint32_t result_ticks[FFT_SIZE] = {0};

    for(;;)
    {
        switch (msg_cmd) {
            case IPC_START_S:
            printf("START_S\n");
                /* Check if any microphone has data to process */
                if (pdm_pcm_flag)
                {
                    /* Clear the PDM/PCM flag */
                    pdm_pcm_flag = 0;

                    /* Setup to read the next frame */
                    cyhal_pdm_pcm_read_async(&pdm_pcm, audio_frame, FRAME_SIZE);

                    // Copy input so FFT doesn't modify original
                    float32_t audio_frame_f32[FFT_SIZE] = {0};
                    for (size_t i = 0; i < FFT_SIZE; i++)
                    {
                        audio_frame_f32[i] = (float32_t)audio_frame[i];
                    }

                    uint32_t p_min_index;
                    float32_t min_audio;
                    arm_min_f32(audio_frame_f32, FFT_SIZE, &min_audio, &p_min_index);

                    uint32_t p_max_index;
                    float32_t max_audio;
                    arm_max_f32(audio_frame_f32, FFT_SIZE, &max_audio, &p_max_index);

                    if (max_audio != min_audio) {
                        for (int i = 0; i < FFT_SIZE; i++) {
                            audio_frame_f32[i] = audio_frame_f32[i] / (max_audio - min_audio);
                        }
                    }

                    arm_rfft_fast_f32(&rfft_instance, audio_frame_f32, results, 0);

                    float32_t amplitudes[FFT_SIZE/2];
                    convert_to_amplitudes(results, amplitudes);

                    print_fft_results(results);

                    // printf("current time: %f\n", (float32_t) cyhal_timer_read(&fft_timer) / (float32_t) FFT_TIMER_HZ);

                    // SEND_IPC_MSG(IPC_END_R);
                    v_index++;

                    if (v_index >= FFT_SIZE) v_index = 0;
                }

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