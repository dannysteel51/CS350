/*
 * Copyright (c) 2015-2020, Texas Instruments Incorporated
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
 *  ======== gpiointerrupt.c ========
 */
#include <stdint.h>
#include <stddef.h>
#include <stdio.h>

/* Driver Header files */
#include <ti/drivers/GPIO.h>
#include <ti/drivers/I2C.h>
#include <ti/drivers/UART.h>
#include <ti/drivers/Timer.h>


/* Driver configuration */
#include "ti_drivers_config.h"

// SM Global Variables
volatile unsigned char btn_WarmFlag = 0;
volatile unsigned char btn_CoolFlag = 0;
volatile unsigned char heatOn_g;
unsigned char setPoint_g;
unsigned char roomTemp_g;
unsigned short mSPeriod_g = 100;



// UART Global Variables
char output[64];
int bytesToSend;

// Driver Handles - Global variables
UART_Handle uart;

// I2C Global Variables
static const struct {
    uint8_t address;
    uint8_t resultReg;
    char *id;
} sensors[3] = {
    { 0x48, 0x0000, "11X" },
    { 0x49, 0x0000, "116" },
    { 0x41, 0x0001, "006" }
};
uint8_t txBuffer[1];
uint8_t rxBuffer[2];
I2C_Transaction i2cTransaction;

// Driver Handles - Global variables
I2C_Handle i2c;

// Driver Handles - Global variables
Timer_Handle timer0;

volatile unsigned char TimerFlag = 0;
void timerCallback(Timer_Handle myHandle, int_fast16_t status)
{
    TimerFlag = 1;
}

#define DISPLAY(x) UART_write(uart, &output, x);


void initUART(void)
{
    UART_Params uartParams;

    // Init the driver
    UART_init();

    // Configure the driver
    UART_Params_init(&uartParams);
    uartParams.writeDataMode = UART_DATA_BINARY;
    uartParams.readDataMode = UART_DATA_BINARY;
    uartParams.readReturnMode = UART_RETURN_FULL;
    uartParams.baudRate = 115200;

    // Open the driver
    uart = UART_open(CONFIG_UART_0, &uartParams);

    if (uart == NULL) {
    /* UART_open() failed */
        while (1);
    }
}


// Make sure you call initUART() before calling this function.
void initI2C(void)
{
    int8_t i, found;
    I2C_Params i2cParams;

    DISPLAY(snprintf(output, 64, "Initializing I2C Driver - "))

    // Init the driver
    I2C_init();

    // Configure the driver
    I2C_Params_init(&i2cParams);
    i2cParams.bitRate = I2C_400kHz;

    // Open the driver
    i2c = I2C_open(CONFIG_I2C_0, &i2cParams);
    if (i2c == NULL)
    {
        DISPLAY(snprintf(output, 64, "Failed\n\r"))
        while (1);
    }

    DISPLAY(snprintf(output, 32, "Passed\n\r"))

    /* Common I2C transaction setup */
    i2cTransaction.writeBuf = txBuffer;
    i2cTransaction.writeCount = 1;
    i2cTransaction.readBuf = rxBuffer;
    i2cTransaction.readCount = 0;

    found = false;
    for (i=0; i<3; ++i)
    {
        i2cTransaction.slaveAddress = sensors[i].address;
        txBuffer[0] = sensors[i].resultReg;

        DISPLAY(snprintf(output, 64, "Is this %s? ", sensors[i].id))
        if (I2C_transfer(i2c, &i2cTransaction))
        {
            DISPLAY(snprintf(output, 64, "Found\n\r"))
            found = true;
            break;
        }
        DISPLAY(snprintf(output, 64, "No\n\r"))
    }

    if(found)
    {
        DISPLAY(snprintf(output, 64, "Detected TMP%s I2C address:"
                "%x\n\r", sensors[i].id, i2cTransaction.slaveAddress))
    }
    else
    {
        DISPLAY(snprintf(output, 64, "Temperature sensor not found,"
                "contact professor\n\r"))
    }
}

int16_t readTemp(void)
{
//    int j;
    int16_t temperature = 0;
    i2cTransaction.readCount = 2;
    if (I2C_transfer(i2c, &i2cTransaction))
    {
        /*
         * * Extract degrees C from the received data;
         * * see TMP sensor datasheet
         * */
        temperature = (rxBuffer[0] << 8) | (rxBuffer[1]);
        temperature *= 0.0078125;

        if (rxBuffer[0] & 0x80)
        {
            temperature |= 0xF000;
        }
    }
    else
    {
        DISPLAY(snprintf(output, 64, "Error reading temperature sensor(%d)\n\r",i2cTransaction.status))
        DISPLAY(snprintf(output, 64, "Please power cycle your board by unplugging USB and plugging back in.\n\r"))
    }

    return temperature;
}

void initTimer(void)
{
    Timer_Params params;

    // Init the driver
    Timer_init();

    // Configure the driver
    Timer_Params_init(&params);
    params.period = mSPeriod_g * 1000;  // 100000 microseconds or 100mS
    params.periodUnits = Timer_PERIOD_US;
    params.timerMode = Timer_CONTINUOUS_CALLBACK;
    params.timerCallback = timerCallback;

    // Open the driver
    timer0 = Timer_open(CONFIG_TIMER_0, &params);
    if (timer0 == NULL) {
        /* Failed to initialized timer */
        while (1) {}
    }

    if (Timer_start(timer0) == Timer_STATUS_ERROR) {
        /* Failed to start timer */
        while (1) {}
    }
}






// SM Poll SetPoint Command States
enum PS_States { PS_SMStart, PS_Init, PS_1, PS_Decrease, PS_Increase } PS_State;


// Call PollSetPoint State Machine before HeatControl and ThermostatOutput SMs
void TickFct_PollSetPoint()
{
    // Local variables
    static unsigned char inc;  // Temperature setPoint increase flag
    static unsigned char dec;  // Temperature setPoint decrease flag


    switch(PS_State) {  // Transitions
        case PS_SMStart:
            PS_State = PS_Init; // Initial State
            break;

        case PS_Init:
            PS_State = PS_1;
            break;

        case PS_1:  // Stay in this state until button press is flagged for setPoint change (warmer/cooler)
            if(inc) { // Transition to setPoint increase
                PS_State = PS_Increase;
                break;
            }
            else if(dec) {  // Transition to setPoint decrease
                PS_State = PS_Decrease;
                break;
            }
            else if(!(inc || dec)) {  // No change
                PS_State = PS_1;
                break;
            }

        case PS_Decrease:
            if(inc) { // Transition to setPoint increase
                PS_State = PS_Increase;
                break;
            }
            else if(dec) {  // Transition to setPoint decrease
                PS_State = PS_Decrease;
                break;
            }
            else if(!(inc || dec)) {  // No change to setPoint
                PS_State = PS_1;
                break;
            }

        case PS_Increase:
            if(inc) { // Transition to setPoint increase
                PS_State = PS_Increase;
                break;
            }
            else if(dec) {  // Transition to setPoint decrease
                PS_State = PS_Decrease;
                break;
            }
            else if(!(inc || dec)) {  // No change to setPoint
                PS_State = PS_1;
                break;
            }

        default:
            PS_State = PS_SMStart;
            break;

    }

    switch(PS_State) {  // State actions
        case PS_Init:   // Initialize variables
            PS_State = PS_1;
            btn_WarmFlag = 0;
            btn_CoolFlag = 0;
            setPoint_g = 24;
            inc = 0;
            dec = 0;
            break;
        case PS_1:    // No change in setPoint, Poll SetPoint Buttons
            inc = btn_WarmFlag; // Poll warmer button
            dec = btn_CoolFlag; // Poll cooler button
            btn_WarmFlag = 0;   // Lower flag
            btn_CoolFlag = 0;   // Lower flag
            break;
        case PS_Decrease: // Lower setPoint 1°C, Poll SetPoint Buttons
            if(setPoint_g > 0) {   // Minimum setPoint is 0
                --setPoint_g;
            }
            inc = btn_WarmFlag; // Poll warmer button
            dec = btn_CoolFlag; // Poll cooler button
            btn_WarmFlag = 0;   // Lower flag
            btn_CoolFlag = 0;   // Lower flag
            break;
        case PS_Increase: // Raise setPoint 1°C, Poll SetPoint Buttons
            if(setPoint_g < 99) {   // Maximum setPoint is 99
                ++setPoint_g;
            }
            inc = btn_WarmFlag; // Poll warmer button
            dec = btn_CoolFlag; // Poll cooler button
            btn_WarmFlag = 0;   // Lower flag
            btn_CoolFlag = 0;   // Lower flag
            break;
        default:
            break;
    }
}


// Call HeatControl State Machine before ThermostatOutput SM
enum HC_States { HC_SMStart, HC_Init, HC_HeatOn, HC_HeatOff } HC_State;

void TickFct_HeatControl()
{

    switch(HC_State) {  // Transitions
        case HC_SMStart:
            HC_State = HC_Init; // Initial State
            break;

        case HC_Init:
            if(roomTemp_g < setPoint_g) {   // Transition to heat on
                HC_State = HC_HeatOn;
                break;
            }
            else {  // Transition to heat off
                HC_State = HC_HeatOff;
                break;
            }

        case HC_HeatOn:
            if(!(roomTemp_g > setPoint_g)) {   // Stay in heat on until room is warmer than setPoint
                HC_State = HC_HeatOn;
                break;
            }
            else if(roomTemp_g > setPoint_g) {  // Transition to heat off
                HC_State = HC_HeatOff;
                break;
            }

        case HC_HeatOff:
            if(!(roomTemp_g < setPoint_g)) {   // Stay in heat off until room is cooler than setPoint
                HC_State = HC_HeatOff;
                break;
            }
            else if(roomTemp_g < setPoint_g) {  // Transition to heat on
                HC_State = HC_HeatOn;
                break;
            }

        default:
            HC_State = HC_SMStart;
            break;

    }

    switch(HC_State) {  // State actions
        case HC_Init:   // Initialize variables
            if(readTemp()) { roomTemp_g = readTemp(); } // Poll on-board thermistor
            heatOn_g = 0;   // Heat indicator to server is off
            GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_OFF); // Turn off red LED
            break;
        case HC_HeatOn:    // Turn on heat
            if(readTemp()) { roomTemp_g = readTemp(); } // Poll on-board thermistor
            heatOn_g = 1;   // Heat indicator to server is on
            GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_ON); // Turn on red LED
            break;
        case HC_HeatOff:    // Turn off heat
            if(readTemp()) { roomTemp_g = readTemp(); } // Poll on-board thermistor
            heatOn_g = 0;   // Heat indicator to server is off
            GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_OFF); // Turn off red LED
            break;
        default:
            break;
    }
}


// SM Thermostat Output
enum TO_States { TO_SMStart, TO_Init, TO_1 } TO_State;

void TickFct_ThermostatOutput()
{

    // Local variables
    static unsigned int seconds;  // Temperature setPoint increase flag

    switch(TO_State) {  // Transitions
        case TO_SMStart:
            TO_State = TO_Init; // Initial State
            break;

        case TO_Init:
            TO_State = TO_1;
            break;

        case TO_1:
            TO_State = TO_1;
            break;

        default:
            TO_State = TO_SMStart;
            break;

    }

    switch(TO_State) {  // State actions
        case TO_Init:   // Initialize variables
            seconds = 0;
            break;
        case TO_1:    // Turn on heat
            ++seconds;
            /*   Transmit Data to Server   */
            // "<%02d, %02d, %d, %04d>, temperature, setPoint, heat, seconds
            DISPLAY( snprintf(output, 64, "<%02d, %02d, %d, %04d>\n\r", roomTemp_g, setPoint_g, heatOn_g, seconds))
            break;
        default:
            break;
    }
}



/*
 *  ======== gpioButtonFxn0 ========
 *  Callback function for the GPIO interrupt on CONFIG_GPIO_BUTTON_0.
 *
 *  Note: GPIO interrupts are cleared prior to invoking callbacks.
 */
void gpioButtonFxn0(uint_least8_t index)
{

    // Right button [SW2] - set Warm Flag on and set Cool Flag off (increase setPoint mode)
    btn_WarmFlag = 1;
    btn_CoolFlag = 0;
}

/*
 *  ======== gpioButtonFxn1 ========
 *  Callback function for the GPIO interrupt on CONFIG_GPIO_BUTTON_1.
 *  This may not be used for all boards.
 *
 *  Note: GPIO interrupts are cleared prior to invoking callbacks.
 */
void gpioButtonFxn1(uint_least8_t index)
{
    // Left button [SW3] - set Cool Flag on and set Warm Flag off (decrease setPoint mode)
    btn_WarmFlag = 0;
    btn_CoolFlag = 1;
}

/*
 *  ======== mainThread ========
 */
void *mainThread(void *arg0)
{
    /* Call driver init functions */
    GPIO_init();

    /* Configure the LED and button pins */
    GPIO_setConfig(CONFIG_GPIO_LED_0, GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW);
//    GPIO_setConfig(CONFIG_GPIO_LED_1, GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW);
    GPIO_setConfig(CONFIG_GPIO_BUTTON_0, GPIO_CFG_IN_PU | GPIO_CFG_IN_INT_FALLING);

    /* Turn on user LED */
    GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_ON);

    /* Install Button callback */
    GPIO_setCallback(CONFIG_GPIO_BUTTON_0, gpioButtonFxn0);

    /* Enable interrupts */
    GPIO_enableInt(CONFIG_GPIO_BUTTON_0);

    /*
     *  If more than one input pin is available for your device, interrupts
     *  will be enabled on CONFIG_GPIO_BUTTON1.
     */
    if (CONFIG_GPIO_BUTTON_0 != CONFIG_GPIO_BUTTON_1) {
        /* Configure BUTTON1 pin */
        GPIO_setConfig(CONFIG_GPIO_BUTTON_1, GPIO_CFG_IN_PU | GPIO_CFG_IN_INT_FALLING);

        /* Install Button callback */
        GPIO_setCallback(CONFIG_GPIO_BUTTON_1, gpioButtonFxn1);
        GPIO_enableInt(CONFIG_GPIO_BUTTON_1);
    }

    initUART(); // The UART must be initialized before calling initI2C()
    initI2C();
    initTimer();
    PS_State = PS_SMStart;  // Initial state
    HC_State = HC_SMStart;  // Initial state
    TO_State = TO_SMStart;  // Initial state

    // Timer period (mSPeriod_g) set to 100ms
    unsigned long PS_elapsedTime = 200;
    unsigned long HC_elapsedTime = 500;
    unsigned long TO_elapsedTime = 1000;

    // Loop Forever
    while(1)
    {
        // Every 200ms check the button flags
        if(PS_elapsedTime >= 200) { // 200ms Period
            TickFct_PollSetPoint(); // Execute one tick of PollSetPoint SM
            PS_elapsedTime = 0;
        }

        // Every 500ms read the temperature and update the LED
        if(HC_elapsedTime >= 500) { // 500ms Period
            TickFct_HeatControl(); // Execute one tick of HeatControl SM
            HC_elapsedTime = 0;
        }

        // Every second output the following to UART
        if(TO_elapsedTime >= 1000) { // 1000ms or 1 second Period
            TickFct_ThermostatOutput(); // Execute one tick of ThermostatOutput SM
            TO_elapsedTime = 0;
        }

        while (!TimerFlag){}    // Wait for the timer period
        TimerFlag = 0;          // Lower flag raised by timer

        PS_elapsedTime += mSPeriod_g;   // Increment by 100mS (mSPeriod_g) every tick
        HC_elapsedTime += mSPeriod_g;   // Increment by 100mS (mSPeriod_g) every tick
        TO_elapsedTime += mSPeriod_g;   // Increment by 100mS (mSPeriod_g) every tick
    }



//    return (NULL);
}
