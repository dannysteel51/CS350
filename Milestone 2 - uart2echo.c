/*
 * Copyright (c) 2020, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 *  ======== uart2echo.c ========
 */
#include <stdint.h>
#include <stddef.h>
#include <stdio.h>
#include <ctype.h>

/* Driver Header files */
#include <ti/drivers/GPIO.h>
#include <ti/drivers/UART2.h>

/* Driver configuration */
#include "ti_drivers_config.h"

enum LED_States {INIT, WAIT1, WAIT2, OFF, ON}; // The states used in the in/else checks for user input

/*
 *  ======== mainThread ========
 */
void *mainThread(void *arg0)
{
    char         input;
    unsigned char state;
    const char   echoPrompt[] = "Echoing characters:\r\n";
    UART2_Handle uart;
    UART2_Params uartParams;
    size_t       bytesRead;
    size_t       bytesWritten;

    /* Call driver init functions */
    GPIO_init();

    /* Configure the LED pin */
    GPIO_setConfig(CONFIG_GPIO_LED_0, GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW);

    /* Create a UART where the default read and write mode is BLOCKING */
    UART2_Params_init(&uartParams);
    uartParams.baudRate = 115200;

    uart = UART2_open(CONFIG_UART2_0, &uartParams);

    if (uart == NULL) {
        /* UART2_open() failed */
        while (1);
    }

    UART2_write(uart, echoPrompt, sizeof(echoPrompt), &bytesWritten);

    // The initial state for the program
    state = INIT;

    /* Loops forever */
    while (1) {
        UART2_read(uart, &input, 1, &bytesRead);

        input = toupper(input);  // Make user input is upper case for uniformity

        // State Machine to turn LED ON/OFF
        switch(input){

            case 'O':   // If the user types 'O' for on or off
                state = WAIT1; // Waits for the next letter to be entered
                break;


            case 'F':   // 'F' fof off                      // IF the user presses 'F' for off
                if (state == WAIT1 || state == WAIT2) {      // Checks if the state is either Wait1 or Wait2
                    if (state == WAIT1) {                    // If current state is Wait1 and the user enters 'F', state is now Wait2
                        state = WAIT2;
                    }
                    else {                                   // If state is Wait2 and F is pressed, state is now OFF
                        state = OFF;
                    }
                }
                else {
                    state = INIT;                           // If 'F' is pressed while state is still INIT, it stays in INIT
                }
                break;

            case 'N':   // User presses 'N' for ON
                if (state == WAIT1) {                       // If the state is Wait1 and the user presses N, state is now ON
                    state = ON;
                }
                else {
                    state = INIT;                           // If the state is INIT and the user presses N, it stays in INIT
                }
                break;

            default:
                state = INIT;  // Initial state
                break;
         }

        // Turns on LED, next clears state
        if (state == ON){
            GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_ON);
            state = INIT;
        }
        // Turns off LED, next clears state
        else if (state == OFF){
            GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_OFF);
            state = INIT;
        }

        UART2_write(uart, &input, 1, &bytesWritten);

    }
}
