/*
 * WHCS_NRF.c
 *
 * Created: 3/4/2015 1:46:59 PM
 *  Author: Jimmy
 */ 

#include <avr/io.h>
#include "nRF24L01.h"

//Default length for the NRF Payload
#ifndef DEF_PAYLOAD_SIZE
#define DEF_PAYLOAD_SIZE 5
#endif

//Default channel used by the NRF, each step is 1 MHz starting at 2.4 GHZ
#ifndef DEF_CHANNEL
#define DEF_CHANNEL 0x36
#endif

#ifndef DEF_RETRIES
#define DEF_RETRIES 15
#endif

int function(void)
{
    //TODO:: Please write your application code

    return 0;
}