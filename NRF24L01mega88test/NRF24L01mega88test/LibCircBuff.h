/*
 * LibCircBuff.h
 *
 * Created: 3/1/2015 11:55:13 AM
 *  Author: Jimmy
 */ 


#ifndef LIBCIRCBUFF_H_
#define LIBCIRCBUFF_H_

#ifndef BUFF_SIZE
#define BUFF_SIZE 64
#endif

#ifdef __cplusplus
extern "C" {
#endif

/*
* Ring Buffer with size of BUFF_SIZE (recommend 64)
* Head points to the first valid byte in the buffer
* Tail points to the next open space.
* At first head points to an uninitialized byte so
* A write must be done.
*/
typedef struct {
	unsigned char Data[BUFF_SIZE];
	unsigned char Head;
	unsigned char Tail;
	unsigned int OverRun;
} CircBuff_t;

/*
* Initializes the head and tail state for the ring buffer
*/
void initCircBuff(volatile CircBuff_t *cBuff);

//This reads the value that is pointed to by head
//Then head is incremented to the next spot
//Wrapping is handled with modulus of BUFF_SIZE
//NO OVERFLOW CHECKING IMPLEMENTED
unsigned char readCircbuff(volatile CircBuff_t *cBuff);

//This writes to the next available space which
//is pointed to by tail
//Then tail is incremented by 1
//Wrapping is handled by modulus of BUFF_SIZE
//NO OVERFLOW CHECKING IMPLEMENTED
void writeCircBuff(volatile CircBuff_t *cBuff, unsigned char val);

#ifdef __cplusplus
}
#endif

#endif /* LIBCIRCBUFF_H_ */