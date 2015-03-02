#define F_CPU 1000000UL  // 8 MHz

/*Very Important - change F_CPU to match target clock 
  Note: default AVR CLKSEL is 1MHz internal RC
  This program transmits continously on USART. Interrupt is used for 
	Receive charactor, which is then transmitted instead. LEDs are used 
	as a test. Normal RX routine is included but not used.
  Change USART_BAUDRATE constant to change Baud Rate
*/

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

// Define baud rate
#define USART_BAUDRATE 9600
#define BAUD_PRESCALE (((F_CPU / (USART_BAUDRATE * 8UL))) - 1) //use 16UL for asynchronous normal, 8UL for asynchronous double speed
#define BUFF_SIZE 64

volatile unsigned char value;  
/* This variable is volatile so both main and RX interrupt can use it.
It could also be a uint8_t type */

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

volatile CircBuff_t rBuff;

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

/* Interrupt Service Routine for Receive Complete 
NOTE: vector name changes with different AVRs see AVRStudio -
Help - AVR-Libc reference - Library Reference - <avr/interrupt.h>: Interrupts
for vector names other than USART_RXC_vect for ATmega32 */

ISR(USART_RX_vect){
 
   cli(); // Turn off interrupts temporarily to make sure the data gets stored in the ring buffer
   value = UDR0;             //read UART register into value
   writeCircBuff(&rBuff,value); //Write the data into the ring buffer, this isn't interrupted because ints are off
   sei();  //Reenable interrupts
}

void USART_Init(void){
   // Set baud rate
   //UBRR0L = BAUD_PRESCALE;// Load lower 8-bits into the low byte of the UBRR register
   //UBRR0H = (BAUD_PRESCALE >> 8); 
	 /* Load upper 8-bits into the high byte of the UBRR register
    Default frame format is 8 data bits, no parity, 1 stop bit
  to change use UCSRC, see AVR datasheet*/ 

  // Enable receiver and transmitter and receive complete interrupt 
  //UCSR0B = ((1<<TXEN0)|(1<<RXEN0) | (1<<RXCIE0));
  
  	UCSR0A = (1 << U2X0);   //U2X0 = 1: asynchronous double speed mode, U2X = 0: asynchronous normal mode

  	/* Turn on the transmission and reception circuitry. */
  	UCSR0B = (1 << RXEN0) | (1 << TXEN0);
  	/* Use 8-bit character sizes. */
  	//UCSR0C = (1 << UCSZ00) | (1 << UCSZ01);

  	/* BAUD prescale */
  	UBRR0 = BAUD_PRESCALE;

  	/* Load upper 8-bits of the baud rate value into the high byte of the UBRR register. */
  	//UBRR0H = (BAUD_PRESCALE >> 8);
  	/* Load lower 8-bits of the baud rate value into the low byte of the UBRR register. */
  	//UBRR0L = BAUD_PRESCALE;

  	UCSR0B |= (1 << RXCIE0);

  	sei();
  
  
}


void USART_SendByte(uint8_t u8Data){

  // Wait until last byte has been transmitted
  while((UCSR0A &(1<<UDRE0)) == 0);

  // Transmit data
  UDR0 = u8Data;
}


// not being used but here for completeness
      // Wait until a byte has been received and return received data 
uint8_t USART_ReceiveByte(){
  while((UCSR0A &(1<<RXC0)) == 0);
  return UDR0;
}

//This assumes that an led is attached to Port B pin 0
//+ side on pin 0, - side connected to resistor connecting to ground.
void Led_init(void){
   //outputs, PB0 on
	 DDRB =0x01;       
   PORTB = 0x01;        
}


int main(void){
   initCircBuff(&rBuff);
   USART_Init();  // Initialise USART
   Led_init();    // init LEDs for testing
   value = 'A'; //0x41;    
   PORTB = 0x01; // 0 = LED on
   
   for(;;){    // Repeat indefinitely
             
	if(value == 0x41)
	{
		PORTB=0x01;
	}
	else
	{
		PORTB=0x00;
	}
			 
	if((rBuff.Head+1)%BUFF_SIZE != rBuff.Tail)
	{
		USART_SendByte(readCircbuff(&rBuff));  // send value
		//_delay_ms(250);
		// delay just to stop Hyperterminal screen cluttering up
	}
     //USART_SendByte(value);  // send value 
     //_delay_ms(250);         
		         // delay just to stop Hyperterminal screen cluttering up    
   }
}