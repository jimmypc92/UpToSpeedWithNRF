/*
 * NRF24L01mega88test.cpp
 *
 * Created: 2/20/2015 3:07:31 PM
 *  Author: Jimmy
 */ 

#define F_CPU 1000000UL  // 1 MHz

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

#include "nRF24L01.h"

// Define baud rate
#define USART_BAUDRATE 9600
#define BAUD_PRESCALE (((F_CPU / (USART_BAUDRATE * 8UL))) - 1)
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
  
  	UCSR0A = (1 << U2X0);

  	/* Turn on the transmission and reception circuitry. */
  	UCSR0B = (1 << RXEN0) | (1 << TXEN0);
  	/* Use 8-bit character sizes. */
  	//UCSR0C = (1 << UCSZ00) | (1 << UCSZ01);

  	/* BAUD prescale */
  	UBRR0 = 12;

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
	DDRB |= 0x01;       
	PORTB |= 0x01;        
}

/*
* NRF usage functions come after this.
*/

void InitSPI(void)
{
	//Set SCK (PB5), MOSI (PB3) , CSN (SS & PB2) & C  as outport
	//OBS!!! Has to be set before SPI-Enable below
	DDRB |= (1<<DDB5) | (1<<DDB3) | (1<<DDB2) |(1<<DDB1);
	DDRB &= (~(1<<DDB4));
	
	// Enable SPI, Master, set clock rate fck/16 .. clock rate not to important..
	SPCR |= (1<<SPE)|(1<<MSTR) |(1<<SPR0); // |(1<<SPR1);
	
	//PORTB |= (1 <<2);
	SETBIT(PORTB, 2);	//CSN IR_High to start with, nothing to be sent to the nRF yet!
	//PORTB &= (~(1<<1)); 
	CLEARBIT(PORTB, 1);	//CE low to start with, nothing to send/receive yet!
}

//Sends and receives a byte through SPI
unsigned char WriteByteSPI(unsigned char cData)
{
	//Load byte to Data register
	SPDR = cData;
	
	/* Wait for transmission complete */
	while(!(SPSR)&(1<<SPIF));
	
	//Return what's received from the nRF
	return SPDR;
}

//Read a register from the nRF
uint8_t GetReg(uint8_t reg)
{
	//make sure last command was a while ago
	_delay_us(15);
	//PORTB &= (~(1<<2)); 
	CLEARBIT(PORTB, 2);	//CSN low - nRF starts to listen for command
	_delay_us(15);
	WriteByteSPI(R_REGISTER + reg);	//R_Register = set the nRF to reading mode, "reg" = this register well be read back
	_delay_us(15);
	reg = WriteByteSPI(NOP);	//Send NOP (dummy byte) once to receive back the first byte in the "reg" register
	_delay_us(15);
	//PORTB |= (1<<2); 
	SETBIT(PORTB, 2);	//CSN Hi - nRF goes back to doing nothing
	return reg;	// Return the read register
}

uint8_t *WriteToNrf(uint8_t ReadWrite, uint8_t reg, uint8_t *val, uint8_t antVal)	//tar in "ReadWrite" (W el R), "reg" (ett register), "*val" (en array) & "antVal" (antal integer i variabeln)
{
	cli();	//disable global interrupt
	
	if (ReadWrite == W)	//if "W" then you want to write to the nRF (read mode "R" == 0x00, so skipping that one)
	{
		reg = W_REGISTER + reg;	//ex: reg = EN_AA: 0b0010 0000 + 0b0000 0001 = 0b0010 0001
	}
	
	//Create an array to be returned at the end.
	//Static uint8_t is needed to be able to return an array
	static uint8_t ret[32];	
	
	_delay_us(10);		//make sure the last command was a while ago
	CLEARBIT(PORTB, 2);	//CSN low = nRF starts to listen for command
	_delay_us(10);
	WriteByteSPI(reg);	//set the nRF to Write or read mode of "reg"
	_delay_us(10);
	
	int i;
	for(i=0; i<antVal; i++)
	{
		if (ReadWrite == R && reg != W_TX_PAYLOAD)
		{
			ret[i]=WriteByteSPI(NOP);	//send dummy bytes to read out the data
			_delay_us(10);
		}
		else
		{
			WriteByteSPI(val[i]);	//Send the commands to the nRF once at a time
			_delay_us(10);
		}
	}
	SETBIT(PORTB, 2);	//CSN Hi - nRF goes back to doing nothing.
	
	sei(); //enable global interrupt
	
	return ret;	//returnerar en array
}

//Initializes the nRF.
void nrf24L01_init(void)
{
	_delay_ms(100);	//allow radio to reach power down if shut down
	
	uint8_t val[5];	//An array of integers to send to the *WriteToNrf function
	
	//EN_AA - (auto-acknowledgements) - Transmitter gets automatic response from receiver when successful transmission! (lovely function!)
	//Only works if Transmitter has identical RF_Address on its channel ex: RX_ADDR_P0 = TX_ADDR
	val[0]=0x01;	//Set value
	WriteToNrf(W, EN_AA, val, 1);	//W=write mode, EN_AA=register to write to, val=data to write, 1 = number of data bytes.
	
	//Sets number of retries and retry delay
	val[0]=0x2F;	//0b0010 00011 "2" sets it up to 750uS delay between every retry (at least 500us at 250kbps and if payload >5bytes in 1Mbps, and if payload >15byte in 2Mbps) "F" is number of retries (1-15, now 15)
	WriteToNrf(W, SETUP_RETR, val, 1);
	
	////Choose number of enabled data pipes (1-5)
	val[0]=0x01;
	WriteToNrf(W, EN_RXADDR, val, 1); //enable data pipe 0
	
	//RF_Address width setup (how many bytes is the receiver address, the more the merrier 1-5)
	val[0]=0x03;
	WriteToNrf(W, SETUP_AW, val, 1); //0b0000 00011 5 bytes RF_Address
	
	//RF channel setup - choose frequency 2,400 - 2,527GHz 1MHz/step
	val[0]=0x01;
	WriteToNrf(W, RF_CH, val, 1); //RF channel registry 0b0000 0001 = 2,401GHz (same on TX and RX)
	
	//RF setup	- choose power mode and data speed. Here is the difference with the (+) version!!!
	val[0]=0x07;
	WriteToNrf(W, RF_SETUP, val, 1); //00000111 bit 3="0" 1Mbps=longer range, bit 2-1 power mode ("11" = -0db; 00 = -18db)
	
	//RX RF_Adress setup 5 byte - Set Receiver Address (set RX_ADDR_P0 = TX_ADDR if EN_AA is enabled!!!)
	int i;
	for(i=0; i<5; i++)
	{
		val[i]=0x12;	//0x12 x 5 to get a long secure address
	}
	WriteToNrf(W, RX_ADDR_P0, val, 5); //0b0010 1010 write registry
	//Since we chose pipe 0 on EN_RXADDR we give this address to that channel.
	//Here you can give different addresses to different channels (if they are enabled in EN_RXADDR) to listen on several different transmitters
	
	//TX RF_Adress setup 5 byte -  Set Transmitter address (not used in a receiver but can be set anyway)
	//int i; //återanvänder föregående i...
	for(i=0; i<5; i++)
	{
		val[i]=0x12;	//0x12 x 5 same on the receiver chip and the RX-RF_Address above if EN_AA is enabled!!!
	}
	WriteToNrf(W, TX_ADDR, val, 5);
	
	// payload width setup - 1-32byte (how many bytes to send per transmission)
	val[0]=5;		//Send 5 bytes per package this time (same on receiver and transmitter)
	WriteToNrf(W, RX_PW_P0, val, 1);
	
	//CONFIG reg setup - Now it's time to boot up the nRF and choose if it's supposed to be a transmitter or receiver
	val[0]=0x1E;  //0b0000 1110 - bit 0="0":transmitter bit 0="1":Receiver, bit 1="1"power up,
					//bit 4="1": mask_Max_RT i.e. IRQ-interrupt is not triggered if transmission failed.
	WriteToNrf(W, CONFIG, val, 1);
	
	//device need 1.5ms to reach standby mode
	_delay_ms(100);
	
	//sei();
}


void SendHexiByte(uint8_t byte)
{
	char upper = (byte & 0b11110000 );
	char lower = (byte & 0b00001111 );
	
	upper = ((upper >> 4) & 0b00001111);
	
	USART_SendByte(upper + 'A');
	USART_SendByte(lower + 'A');
}







int main(void){
   initCircBuff(&rBuff);
   USART_Init();  // Initialize USART
   Led_init();    // init LEDs for testing
   InitSPI();
   nrf24L01_init();
   value = 'A'; //0x41;    
   PORTB |= 0x01; // 0 = LED on
   
   for(;;){    // Repeat indefinitely
             
	if(value == 0x41)
	{
		PORTB |= 0x01;
		SendHexiByte(GetReg(EN_AA));
		value = 0x43;
	}
	else if(value == 0x45)
	{
		PORTB |= 0x01;
		SendHexiByte(GetReg(STATUS));
		value = 0x43;
	}
	else
	{
		PORTB &= (~(0x01));
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