/*****************************************************************************
* File Name      : main.c
*
* Description    : This is the source code for DEEPCRAFT Deploy Motion Example.
*
* Related Document : See README.md
*
******************************************************************************
* Copyright 2025, Cypress Semiconductor Corporation (an Infineon company) or
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
*****************************************************************************/

/*******************************************************************************
* Header File
*******************************************************************************/

#include "cybsp.h"

#ifdef ML_DEEPCRAFT_CM55
#include "stdlib.h"
#include "retarget_io_init.h"

#include "imu.h"
#endif /* ML_DEEPCRAFT_CM55 */

/*******************************************************************************
* Function Prototypes
*******************************************************************************/
#ifdef ML_DEEPCRAFT_CM55
static cy_rslt_t system_init(void);
static void cm55_ml_deepcraft_task(void);
#endif /* ML_DEEPCRAFT_CM55 */

/*******************************************************************************
* Function Name: main
********************************************************************************
* Summary:
*  This is the main function. It initializes the system on the CM55 CPU.
*  If the model inferencing is set to CM55 + U55, it starts the Deploy
*  Motion application, else, it enters Deepsleep mode.
*
* Parameters:
*  None
*
* Return:
*  int
*
*******************************************************************************/
int main(void)
{
    cy_rslt_t result;

    /* Initialize the device and board peripherals */
    result = cybsp_init();

    /* Board init failed. Stop program execution */
    if (CY_RSLT_SUCCESS != result)
    {
        CY_ASSERT(0);
    }

    /* Enable global interrupts */
    __enable_irq();

#ifdef ML_DEEPCRAFT_CM55
    /* If ML_DEEPCRAFT_CPU is set as CM55, start the task */
    cm55_ml_deepcraft_task();
#else

    for (;;)
    {
        /* If ML_DEEPCRAFT_CPU is set as CM33, put the CM55 to sleep */
        Cy_SysPm_CpuEnterDeepSleep(CY_SYSPM_WAIT_FOR_INTERRUPT);
    }
#endif /* ML_DEEPCRAFT_CM55 */
}

#ifdef ML_DEEPCRAFT_CM55
/*******************************************************************************
* Function Name: system_init
********************************************************************************
* Summary:
*  Initializes the neural network based on the DEEPCRAFT model and the
*  DEEPCRAFT pre-processor and initializes the IMU sensor.
*
* Parameters:
*  None
*
* Returns:
*  The status of the initialization.
*
*******************************************************************************/
static cy_rslt_t system_init(void)
{
    cy_rslt_t result;

    /* Initialize DEEPCRAFT pre-processing library */
    IMAI_init();

    /* Initialize the IMU and related interrupt handling code */
    result = imu_init();

    return result;
}

/*******************************************************************************
* Function Name: cm55_ml_deepcraft_task
********************************************************************************
* Summary:
*  Contains the main loop for the application. It sets up the UART for
*  logs and initialises the system (DEEPCRAFT pre-processor and IMU for input
*  data). It then invokes the IMU Data Processing function that sends the data
*  for pre-processing, inferencing, and prints in the results when enough data
*  data is received.
*
* Parameters:
*  None
*
* Returns:
*  None
*
*******************************************************************************/
static void cm55_ml_deepcraft_task(void)
{
    cy_rslt_t result;

    /* Initialize retarget-io middleware */
    init_retarget_io();

    /* \x1b[2J\x1b[;H - ANSI ESC sequence for clear screen */
    printf("\x1b[2J\x1b[;H");

    /* Initialize inference engine and sensors */
    result = system_init();

    /* Initialization failed */
    if(CY_RSLT_SUCCESS != result)
    {
        /* Failed to initialize properly */
        printf("System initialization fail\r\n");
        while(1);
    }

    for (;;)
    {
        /* Invoke the IMU Data Processing function that sends the data for
         * pre-processing, inferencing, and print the results when enough data
         * is received.
         */
        imu_data_process();
    }
}
#endif /* ML_DEEPCRAFT_CM55 */

/* [] END OF FILE */
