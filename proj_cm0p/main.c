/******************************************************************************
* File Name:   main.c
*
* Description: This is the source code for CM0+ in the the Dual CPU Empty 
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
#include "cycfg.h"
#include "cybsp.h"
#include "cy_retarget_io.h"

#include "ipc_communication.h"

#define CYHAL_GET_PORTADDR(pin)    (Cy_GPIO_PortToAddr(CYHAL_GET_PORT(pin)))
#define CYHAL_GET_PORT(pin)         ((uint8_t)(((uint8_t)pin) >> 3U))

#define SEND_IPC_MSG(x) ipc_msg.cmd = x; \
                        Cy_IPC_Pipe_SendMessage(USER_IPC_PIPE_EP_ADDR_CM4, \
                                                USER_IPC_PIPE_EP_ADDR_CM0, \
                                                (void *) &ipc_msg, 0);  

static volatile uint8_t msg_cmd = 0;

static ipc_msg_t ipc_msg = {              /* IPC structure to be sent to CM4  */
    .client_id  = IPC_CM0_TO_CM4_CLIENT_ID,
    .cpu_status = 0,
    .intr_mask   = USER_IPC_PIPE_INTR_MASK,
    .cmd        = IPC_CMD_STATUS,
    .value      = 0
};

static void cm0p_msg_callback(uint32_t *msg);

int main(void)
{
    /* Enable global interrupts */
    __enable_irq();

    cy_en_ipc_pipe_status_t ipc_status;

    /* Init the IPC communication for CM0+ */
    setup_ipc_communication_cm0();
    
    cy_rslt_t result;

    GPIO_PRT_Type *CYBSP_PIEZO_0_PORT = CYHAL_GET_PORTADDR(P9_0);
    uint8_t CYBSP_PIEZO_0_PIN = CYHAL_GET_PIN(P9_0);

    GPIO_PRT_Type *CYBSP_PIEZO_1_PORT = CYHAL_GET_PORTADDR(P9_1);
    uint8_t CYBSP_PIEZO_1_PIN = CYHAL_GET_PIN(P9_1);

    /* Initialize the device and board peripherals */
    result = cybsp_init() ;
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    /* Enable CM4. CY_CORTEX_M4_APPL_ADDR must be updated if CM4 memory layout is changed. */
    Cy_SysEnableCM4(CY_CORTEX_M4_APPL_ADDR);

    Cy_GPIO_Pin_FastInit(CYBSP_PIEZO_0_PORT, CYBSP_PIEZO_0_PIN, CY_GPIO_DM_STRONG, 0UL, HSIOM_SEL_GPIO);
    Cy_GPIO_Pin_FastInit(CYBSP_PIEZO_1_PORT, CYBSP_PIEZO_1_PIN, CY_GPIO_DM_STRONG, 0UL, HSIOM_SEL_GPIO);


    msg_cmd = IPC_END_R;
    for (;;)
    {
        Cy_SysLib_DelayUs(25);
        Cy_GPIO_Write(CYBSP_PIEZO_0_PORT, CYBSP_PIEZO_0_PIN, 1UL);
        Cy_GPIO_Write(CYBSP_PIEZO_1_PORT, CYBSP_PIEZO_1_PIN, 0UL);

        Cy_SysLib_DelayUs(25);
        Cy_GPIO_Write(CYBSP_PIEZO_0_PORT, CYBSP_PIEZO_0_PIN, 0UL);
        Cy_GPIO_Write(CYBSP_PIEZO_1_PORT, CYBSP_PIEZO_1_PIN, 1UL);
        msg_cmd = 0;
    }
}

/*******************************************************************************
* Function Name: cm0p_msg_callback
********************************************************************************
* Summary:
*   Callback function to execute when receiving a message from CM4 to CM0+.
*
* Parameters:
*   msg: message received
*
*******************************************************************************/
static void cm0p_msg_callback(uint32_t *msg)
{
    ipc_msg_t *ipc_recv_msg;

    if (msg != NULL)
    {
        /* Cast the message received to the IPC structure */
        ipc_recv_msg = (ipc_msg_t *) msg;

        /* Extract the command to be processed in the main loop */
        msg_cmd = ipc_recv_msg->cmd;
    }
}

/* [] END OF FILE */
