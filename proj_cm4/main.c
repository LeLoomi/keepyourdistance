/******************************************************************************
* File Name:   main.c
*
* Description: This is the source code for CM4 in the the Dual CPU Empty 
*              Application for ModusToolbox.
*
* Related Document: See README.md
*
*
*******************************************************************************
* Copyright 2020-2024, Cypress Semiconductor Corporation (an Infineon company) or
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

#include "cy_pdl.h"
#include "cyhal.h"
#include "cybsp.h"
#include "cy_retarget_io.h"

#define AUDIO_BUFFER_SIZE 512

cyhal_pdm_pcm_t     pdm_pcm;
cyhal_pdm_pcm_cfg_t cfg =
{
    .sample_rate     = 44000,
    .decimation_rate = 64,
    .mode            = CYHAL_PDM_PCM_MODE_LEFT,
    .word_length     = 16,
    .left_gain       = 3,  // +0.5 db gain
    .right_gain      = 0, // -1.0 db gain
};

int main(void)
{
    cy_rslt_t result;

    // 1. BSP initialisieren
    result = cybsp_init();
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    // 2. UART initialisieren für printf
    result = cy_retarget_io_init_fc(CYBSP_DEBUG_UART_TX, CYBSP_DEBUG_UART_RX,
                                   CYBSP_DEBUG_UART_CTS, CYBSP_DEBUG_UART_RTS, CY_RETARGET_IO_BAUDRATE);
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    __enable_irq();

    printf("PDM Microphone Example Started\r\n");

    // 3. PDM/PCM initialisieren
    result = cyhal_pdm_pcm_init(&pdm_pcm, P10_5, P10_4, NULL, &cfg);
    if (result != CY_RSLT_SUCCESS)
    {
        printf("Failed to initialize PDM/PCM\r\n");
        CY_ASSERT(0);
    }

    // 4. PDM/PCM starten
    result = cyhal_pdm_pcm_start(&pdm_pcm);
    if (result != CY_RSLT_SUCCESS)
    {
        printf("Failed to start PDM/PCM\r\n");
        CY_ASSERT(0);
    }

    int16_t audio_buffer[AUDIO_BUFFER_SIZE] = {0};
    size_t length = AUDIO_BUFFER_SIZE;

    cyhal_pdm_pcm_start(&pdm_pcm);

    for (;;)
    {
        cy_rslt_t read_result = cyhal_pdm_pcm_read(&pdm_pcm, audio_buffer, &length);
        printf("%u\n", length);

        if (read_result == CY_RSLT_SUCCESS)
        {
            printf("Audio Samples: ");
            for (int i = 0; i < 10; i += 2)
            {
                printf("L:%d R:%d\n", audio_buffer[i], audio_buffer[i+1]);
            }
            printf("\r\n");

            cyhal_system_delay_ms(200);
        }
        else printf("PDM read error: 0x%lu\r\n", read_result);
        
    }

    return 0;
}