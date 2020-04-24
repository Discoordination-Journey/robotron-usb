#define F_CPU 12000000UL
#define TEST if(!(PINB & (1<<0))) PORTB |= (1<<PB0); else PORTB &= ~(1<<PB0);

// robotron's pins
#define ROBO_DDR DDRB
#define ROBO_PORT PORTB
#define ROBO_PIN PINB
#define ROBO 2

#include <stdio.h>
#include <avr/io.h>
#include <avr/wdt.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <avr/eeprom.h>
#include <avr/pgmspace.h>
#include "usbdrv/usbdrv.h"

uint8_t volatile data_pointer = 0;
uint8_t volatile robotron_data = 0;
uint8_t volatile robotron_data_old = 0;
uint8_t volatile modificator = 0;
uint32_t volatile robo_tmp = 0;

uint8_t key[101] = {0xBE, 0xBD, 0xBC, 0xBB, 0xBA, 0xB9, 0xB8, 0xB7, 0xB6, 0xB5, 0xB4, 0xB3, 0xB2, 0xB1, 0xB0, 0xAF, 0xAE, 0xAD, 0xAC, 0xAB, 0xAA, 0xA9, 0xA8, 0xA7, 0xA6, 0xA5};

// USB interface
const PROGMEM char usbHidReportDescriptor[64] = 
{
    0x05, 0x01,                    // USAGE_PAGE (Generic Desktop)
    0x09, 0x06,                    // USAGE (Keyboard)
    0xa1, 0x01,                    // COLLECTION (Application)
    0x75, 0x01,                    //   REPORT_SIZE (1)
    0x95, 0x08,                    //   REPORT_COUNT (8)
    0x05, 0x07,                    //   USAGE_PAGE (Keyboard)
    0x19, 0xe0,                    //   USAGE_MINIMUM (Keyboard LeftControl)
    0x29, 0xe7,                    //   USAGE_MAXIMUM (Keyboard Right GUI)
    0x15, 0x00,                    //   LOGICAL_MINIMUM (0)
    0x25, 0x01,                    //   LOGICAL_MAXIMUM (1)
    0x81, 0x02,                    //   INPUT (Data,Var,Abs)
    0x95, 0x01,                    //   REPORT_COUNT (1)
    0x75, 0x08,                    //   REPORT_SIZE (8)
    0x81, 0x01,                    //   INPUT (Cnst,Ary,Abs)
    0x95, 0x05,                    //   REPORT_COUNT (5)
    0x75, 0x01,                    //   REPORT_SIZE (1)
    0x05, 0x08,                    //   USAGE_PAGE (LEDs)
    0x19, 0x01,                    //   USAGE_MINIMUM (Num Lock)
    0x29, 0x05,                    //   USAGE_MAXIMUM (Kana)
    0x91, 0x02,                    //   OUTPUT (Data,Var,Abs)
    0x95, 0x01,                    //   REPORT_COUNT (1)
    0x75, 0x03,                    //   REPORT_SIZE (3)
    0x91, 0x01,                    //   OUTPUT (Cnst,Ary,Abs)
    0x95, 0x06,                    //   REPORT_COUNT (6)
    0x75, 0x08,                    //   REPORT_SIZE (8)
    0x15, 0x00,                    //   LOGICAL_MINIMUM (0)
    0x26, 0xff, 0x00,              //   LOGICAL_MAXIMUM (255)
    0x05, 0x07,                    //   USAGE_PAGE (Keyboard)
    0x19, 0x00,                    //   USAGE_MINIMUM (Reserved (no event indicated))
    0x29, 0xe7,                    //   USAGE_MAXIMUM (Keyboard Right GUI)
    0x81, 0x00,                    //   INPUT (Data,Ary,Abs)
    0xc0                           // END_COLLECTION
};

// USB keyboard report buffer
static struct REPORT_BUF_STRUCT 
{
	uchar modKey;
	uchar reserved;
	uchar keys[6];
} reportBuffer;

static uchar idleRate;    		// in 4 ms units
static uchar protocolVer = 1;	//  0 = boot protocol, 1 = report protocol
static uchar expectReport;		// flag to indicate if we expect an USB-report
static uchar keyDidChange;		// key status was changed

static void hardwareInit(void)
{
	// PD0..1 -> RX/TX
	// PD2..3 -> D+/D-
	// PD4    -> USB PULLUP CTRL
	// PD6..7 -> (NOT USED) 
	   
	PORTD |= 0xE3;		// 1110 0011 : activate pull-ups excepts on USB lines
	DDRD  |= 0x10;	  	// 0001 0000 : all pins input excepts USB reset
    //TCCR0B |= (1<<CS02); // timer 0 prescaler: 256
	
	PCMSK0 |= (1<<PCINT1);// interrupt
	PCICR |= (1<<PCIE0); 
	DDRB |= (1<<PB0);
}

uchar usbFunctionWrite(uchar *data, uchar len)
{
    expectReport = 0;
    return 1;
}

usbMsgLen_t usbFunctionSetup(uchar data[8])
{
	usbRequest_t *rq = (void*)data;

    usbMsgPtr = (uchar*)&reportBuffer;
    if((rq->bmRequestType & USBRQ_TYPE_MASK) == USBRQ_TYPE_CLASS) // class request type
	{    
        if(rq->bRequest == USBRQ_HID_GET_REPORT) return sizeof(reportBuffer); // wValue: ReportType (highbyte), ReportID (lowbyte)  
		else if(rq->bRequest == USBRQ_HID_SET_REPORT)
		{ 
            if (rq->wLength.word == 1) 
			{
                expectReport = 1;
                return USB_NO_MSG;
            }
        }
		else if(rq->bRequest == USBRQ_HID_GET_IDLE)
		{
            usbMsgPtr = &idleRate;
            return 1;
        }
		else if(rq->bRequest == USBRQ_HID_SET_IDLE) idleRate = rq->wValue.bytes[1];
		else if(rq->bRequest == USBRQ_HID_GET_PROTOCOL) 
		{
			if(rq->wValue.bytes[1] < 1) protocolVer = rq->wValue.bytes[1];
		}		
		else if(rq->bRequest == USBRQ_HID_SET_PROTOCOL) 
		{
            usbMsgPtr = &protocolVer;
            return 1;
        }
    }
	return 0;
}

int main(void)
{
	uchar idleCounter = 0;
	hardwareInit();
    wdt_enable(WDTO_1S);
	
    usbInit();
    usbDeviceDisconnect();  // enforce re-enumeration, do this while interrupts are disabled!
	
	// fake USB disconnect for > 250 ms 
	uchar i=250;
    while(--i) 
	{            
        wdt_reset();
        _delay_ms(1);
    }
	
    usbDeviceConnect();

	uchar cnt;
	for(cnt = 0; cnt < sizeof(reportBuffer.keys); cnt++) reportBuffer.keys[cnt] = 0;
	keyDidChange = 1;

    sei();
	
    for(;;)
	{
        wdt_reset();
        usbPoll();

		/*if(TIFR0 & _BV(TOV0)) // 4ms period expired
		{		
			TIFR0 = _BV(TOV0);
			if(idleRate != 0)
			{
				if(idleCounter > 0) idleCounter--; 
				else
				{
					idleCounter = idleRate;
					keyDidChange = 1;
				}
			}
		}
		*/
		
		if(!data_pointer && robo_tmp != 0) // data processing
		{	
			for(uint8_t j = 0; j < 26; j++)
			{
				// modificator
				if(j > 2 && j < 7)
				{
					if(robo_tmp & (1UL<<j)) modificator |= (1<<(j-3));
					else  modificator &= ~(1<<(j-3));
				}
				
				// key data
				if(j > 15 && j < 25)
				{
					if(robo_tmp & (1UL<<j)) robotron_data |= (1<<(j-16));
					else robotron_data &= ~(1<<(j-16));
				}
			}
			
			robo_tmp = 0;
			bufferWrite();
		}

		if(keyDidChange && usbInterruptIsReady()) // USB
		{
			keyDidChange = 0;
			usbSetInterrupt((uchar*)&reportBuffer, sizeof(reportBuffer));
			
			for(cnt = 0; cnt < sizeof(reportBuffer.keys); cnt++) reportBuffer.keys[cnt] = 0;
			keyDidChange = 1;
		}
	}    
    return 0;
}

void bufferWrite(void) // write buffer from data
{	
	for(uint8_t i = 0; i < 26; i++)
	{
		if(key[i] == robotron_data) 
		{
			reportBuffer.keys[0] = i+4;
			keyDidChange = 1;
			break;
		}
		else 
		{
			data_pointer = 0;
		}
	}
}

ISR(PCINT0_vect) // key is pressed 
{
	if(PINB & (1<<PB1))
	{
		if(ROBO_PIN & (1 << ROBO)) robo_tmp |= (1UL<<data_pointer);
		else robo_tmp &= ~(1UL<<data_pointer);
		
		if(data_pointer == 25)
		{
			 data_pointer = 0; // end of data			
			 TEST
		}		
		else data_pointer++;
	}
}