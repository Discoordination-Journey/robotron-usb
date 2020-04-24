#define F_CPU 16000000UL
#define BAUD 9600

// robotron's pins
#define ROBO_DDR DDRD
#define ROBO_PORT PORTD
#define ROBO_PIN PIND
#define ROBO 3

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdint.h>

uint8_t volatile data_pointer = 0;
uint8_t robotron_data = 0;
uint8_t modificator = 0;

void UART(unsigned int ubrr)
{
	UBRRH |= (unsigned char)(ubrr>>8);
	UBRRL |= (unsigned char)ubrr;
	UCSRB |= (1<<RXEN)|(1<<TXEN)|(1<<RXCIE);
	UCSRC |= (1<<URSEL)|(3<<UCSZ0);
}

void UART_Transmit(unsigned char data)
{
	while(!(UCSRA & (1<<UDRE)));
	UDR = data;
}

void bufferWrite(void)
{
	for(int8_t i = 3; i >= 0; i--)
	{
		if(modificator & (1<<i)) UART_Transmit(0x31);
		else UART_Transmit(0x30);
	}
	
	UART_Transmit(0x20); // space
	
	for(int i = 7; i >= 0; i--)
	{
		if(robotron_data & (1<<i)) UART_Transmit(0x31);
		else UART_Transmit(0x30);
	}
	
	UART_Transmit(0xD); // CR
	UART_Transmit(0xA); // NL
}

ISR(INT0_vect) // key is pressed
{
	if(data_pointer > 2 && data_pointer < 7)
	{
		// modificator
		if(ROBO_PIN & (1 << ROBO)) modificator |= (1 << (data_pointer-3));
		else  modificator &= ~(1 << (data_pointer-3));
	}
	else if(data_pointer > 15 && data_pointer < 25)
	{
		// data
		if(ROBO_PIN & (1 << ROBO)) robotron_data |= (1 << (data_pointer-16));
		else robotron_data &= ~(1 << (data_pointer-16));
	}
	
	if(data_pointer == 25) // end of data
	{
		data_pointer = 0;
		bufferWrite();
	}
	else data_pointer++;
}

void setup(void)
{
	MCUCR |= (1<<ISC00)|(1<<ISC01)|(1<<INT0); // int0
	GICR |= (1<<INT0);
	UART(F_CPU/16/BAUD-1);
	
	PORTB |= (1<<PB1);
	_delay_ms(1000);
	PORTB &= ~(1<<PB1);
	sei();
}

int main(void)
{
	setup();
	bufferWrite();
    while(1) {}
}