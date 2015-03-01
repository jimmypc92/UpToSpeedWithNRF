/*
 * LibCircBuff.c
 *
 * Created: 3/1/2015 11:53:10 AM
 *  Author: Jimmy
 */ 

#include <avr/io.h>
#include "LibCircBuff.h"

/*
* Initializes the head and tail state for the ring buffer
*/
void initCircBuff(volatile CircBuff_t *cBuff)
{
	cBuff->Head = 0; //Points to the first valid byte in the buffer
	cBuff->Tail = 1; //points to next empty byte
	cBuff->OverRun = 0; //true false overrun
}

//This reads the value that is pointed to by head
//Then head is incremented to the next spot
//Wrapping is handled with modulus of BUFF_SIZE
//NO OVERFLOW CHECKING IMPLEMENTED
unsigned char readCircbuff(volatile CircBuff_t *cBuff)
{
	unsigned char readVal = cBuff->Data[cBuff->Head];
	cBuff->Head= (cBuff->Head+1)%BUFF_SIZE;
	return readVal;
}

//This writes to the next available space which
//is pointed to by tail
//Then tail is incremented by 1
//Wrapping is handled by modulus of BUFF_SIZE
//NO OVERFLOW CHECKING IMPLEMENTED
void writeCircBuff(volatile CircBuff_t *cBuff, unsigned char val)
{
	cBuff->Data[cBuff->Head] = val;
	cBuff->Tail = (cBuff->Tail+1)%BUFF_SIZE;
}