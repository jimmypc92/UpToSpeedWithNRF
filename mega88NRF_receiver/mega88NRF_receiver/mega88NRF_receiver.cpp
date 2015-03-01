/*
 * mega88NRF_receiver.cpp
 *
 * Created: 2/22/2015 2:01:43 PM
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

// Array used for storing received data on NRF reception interrupt.
uint8_t *data;


/*
* Initializes LED pins for output
*/
void InitIO()
{
	DDRC |= (1<<DDC5) | (1<<DDC4);
}



/*
* NRF usage functions come after this.
*/

void InitSPI(void) {
	//Clear port B SPI pins for init
	DDRB &= ~((1<<DDB3)|(1<<DDB4)|(1<<DDB2)|(1<<DDB5));
	
	//Set SCK (PB5), MOSI (PB3) , CSN (SS & PB2) & C  as outport
	//OBS!!! Has to be set before SPI-Enable below
	DDRB |= (1<<DDB5) | (1<<DDB3) | (1<<DDB2) |(1<<DDB1);
	
	// Enable SPI, Master, set clock rate fck/16 .. clock rate not to important..
	//SPCR |= (1<<SPE)|(1<<MSTR) |(1<<SPR0); // |(1<<SPR1);
	
	SPCR = ((1<<SPE)|               // SPI Enable
	(0<<SPIE)|              // SPI Interrupt Enable
	(0<<DORD)|              // Data Order (0:MSB first / 1:LSB first)
	(1<<MSTR)|              // Master/Slave select
	(0<<SPR1)|(0<<SPR0)|    // SPI Clock Rate
	(0<<CPOL)|              // Clock Polarity (0:SCK low / 1:SCK hi when idle)
	(0<<CPHA));             // Clock Phase (0:leading / 1:trailing edge sampling)
	
	SPSR |= (1<<SPI2X);
	
	//PORTB |= (1 <<2);
	SETBIT(PORTB, 2);	//CSN IR_High to start with, nothing to be sent to the nRF yet!
	//PORTB &= (~(1<<1));
	CLEARBIT(PORTB, 1);	//CE low to start with, nothing to send/receive yet!
}

//Sends and receives a byte through SPI
uint8_t WriteByteSPI(uint8_t cData)
{
	//Load byte to Data register
	SPDR = cData;
	
	/* Wait for transmission complete */
	while((SPSR & (1<<SPIF))==0);
	
	//Return what's received from the nRF
	return SPDR;
}

//Read a register from the nRF
uint8_t GetReg(uint8_t reg)
{
	//make sure last command was a while ago
	_delay_us(10);
	//PORTB &= (~(1<<2));
	CLEARBIT(PORTB, 2);	//CSN low - nRF starts to listen for command
	_delay_us(10);
	WriteByteSPI(R_REGISTER + reg);	//R_Register = set the nRF to reading mode, "reg" = this register well be read back
	_delay_us(10);
	reg = WriteByteSPI(NOP);	//Send NOP (dummy byte) once to receive back the first byte in the "reg" register
	_delay_us(10);
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
	
	_delay_us(15);		//make sure the last command was a while ago
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
	
	return ret;	//return an array
}

//Initializes the nRF.
void nrf24L01_init(void) {
	_delay_ms(100);	//allow radio to reach power down if shut down
	
	uint8_t val[5];	//An array of integers to send to the *WriteToNrf function
	
	//RF channel setup - choose frequency 2,400 - 2,527GHz 1MHz/step
	val[0]=0x18;
	WriteToNrf(W, RF_CH, val, 1); //RF channel registry 0b0000 0001 = 2,401GHz (same on TX and RX)
	
	//EN_AA - (auto-acknowledgements) - Transmitter gets automatic response from receiver when successful transmission! (lovely function!)
	//Only works if Transmitter has identical RF_Address on its channel ex: RX_ADDR_P0 = TX_ADDR
	val[0]=0x3F;	//Set value
	WriteToNrf(W, EN_AA, val, 1);	//W=write mode, EN_AA=register to write to, val=data to write, 1 = number of data bytes.
	
	// payload width setup - 1-32byte (how many bytes to send per transmission)
	val[0]=5;		//Send 5 bytes per package this time (same on receiver and transmitter)
	WriteToNrf(W, RX_PW_P0, val, 1);
	
	// payload width setup - 1-32byte (how many bytes to send per transmission)
	val[0]=5;		//Send 5 bytes per package this time (same on receiver and transmitter)
	WriteToNrf(W, RX_PW_P1, val, 1);
	
	//Sets number of retries and retry delay
	val[0]=0x2F;	//0b0010 00011 "2" sets it up to 750uS delay between every retry (at least 500us at 250kbps and if payload >5bytes in 1Mbps, and if payload >15byte in 2Mbps) "F" is number of retries (1-15, now 15)
	WriteToNrf(W, SETUP_RETR, val, 1);
	
	////Choose number of enabled data pipes (1-5)
	val[0]=0x3F;
	WriteToNrf(W, EN_RXADDR, val, 1); //enable data pipe 0
	
	//RF_Address width setup (how many bytes is the receiver address, the more the merrier 1-5)
	val[0]=0x03;
	WriteToNrf(W, SETUP_AW, val, 1); //0b0000 00011 5 bytes RF_Address
	
	//RF setup	- choose power mode and data speed. Here is the difference with the (+) version!!!
	val[0]=0x07;
	WriteToNrf(W, RF_SETUP, val, 1); //00000111 bit 3="0" 1Mbps=longer range, bit 2-1 power mode ("11" = -0db; 00 = -18db)
	
	//RX RF_Adress setup 5 byte - Set Receiver Address (set RX_ADDR_P0 = TX_ADDR if EN_AA is enabled!!!)
	int i;
	for(i=0; i<5; i++)
	{
		val[i]=0x74;	//0x58 x 5 to get a long secure address
	}
	WriteToNrf(W, RX_ADDR_P0, val, 5); //0b0010 1010 write registry
	//Since we chose pipe 0 on EN_RXADDR we give this address to that channel.
	
	//RX RF_Adress setup 5 byte - Set Receiver Address (set RX_ADDR_P0 = TX_ADDR if EN_AA is enabled!!!)
	for(i=0; i<5; i++)
	{
		val[i]=0x12;	//0x58 x 5 to get a long secure address
	}
	WriteToNrf(W, RX_ADDR_P1, val, 5); //0b0010 1010 write registry
	//Since we chose pipe 0 on EN_RXADDR we give this address to that channel.
	
	//TX RF_Adress setup 5 byte -  Set Transmitter address (not used in a receiver but can be set anyway)
	for(i=0; i<5; i++)
	{
		val[i]=0x74;	//0x58 x 5 same on the receiver chip and the RX-RF_Address above if EN_AA is enabled!!!
	}
	WriteToNrf(W, TX_ADDR, val, 5);
	
	//CONFIG reg setup - Now it's time to boot up the nRF and choose if it's supposed to be a transmitter or receiver
	val[0]=0x1F;  //0b0000 1111 - bit 0="0":transmitter bit 0="1":Receiver, bit 1="1"power up,
	//bit 4="1": mask_Max_RT i.e. IRQ-interrupt is not triggered if transmission failed.
	WriteToNrf(W, CONFIG, val, 1);
	
	//device need 1.5ms to reach standby mode
	_delay_ms(100);
	
	//sei();
}


void INT0_interrupt_init(void)
{
	DDRD &= ~(1<<DDD2);	//Extern interrupt on INT0, make sure it is input
	
	EICRA |=  (1<<ISC01);// INT0 falling edge	PD2
	EICRA  &=  ~(1<<ISC00);// INT0 falling edge	PD2
	
	EIMSK |=  (1<<INT0);	//enable int0 interrupt
	//sei();              // Enable global interrupts do later
}

void receive_payload(void)
{
	//sei();		//Enable global interrupt
	
	SETBIT(PORTB, 1);	//CE IR_High = "Lyssnar"
	_delay_ms(1000);	//lyssnar i 1s och om mottaget går int0-interruptvektor igång
	CLEARBIT(PORTB, 1); //ce låg igen -sluta lyssna
	
	//cli();	//Disable global interrupt
}


/*
* End of code used for NRF.
*/

int main(void)
{
	InitIO();
	InitSPI();
	nrf24L01_init();
	
	//Toggle on PortC5 LED for status indication
	PORTC |= (1<<DDC5);
	
    while(1)
    {
		//Wait 1 second
        _delay_ms(1000);
		
		//Makes sure that the NRF is connected successfully through SPI
		if(GetReg(STATUS) == 0x0E) {
			int j;
			//Use the LED to let the user know that the nrf is successfully connected.
			for(j=0;j<8;j++)
			{
				PORTC ^= (1<<DDC5);
				//Wait half a second
				_delay_ms(500);
			}
			
			//Attempt to receive data.
			SETBIT(PORTB, 1);
			while(1)
			{
				PORTC ^= (1<<DDC5);
				_delay_ms(400);
				PORTC ^= (1<<DDC5);
				_delay_ms(400);
				//polls whether or not data has been received.
				if(GetReg(STATUS) != 0x0E)
				{
					CLEARBIT(PORTB,1);
					//toggles led PC4 if data has been received.
					PORTC |= (1<<DDC4);
					while(true);
				}
				
			}
		}		
    }
}