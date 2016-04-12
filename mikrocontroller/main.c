//Bibliotheken und Methoden aus Versuch 1 ergaenzenb
#define F_CPU 8000000UL
#define FOSC 8000000
#define BAUD 19200
#include <stdlib.h>
#include <stdio.h>
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include "display.c"
#include "bluetooth.c"
#include "sensoren.c"
volatile uint8_t TFlag;
volatile uint8_t EFlag;
volatile uint8_t CFlag;
volatile uint8_t SFlag;

int zustand;

int main(void) {
	//Setze Output/Inputs
	DDRB = 0xFF;
	DDRD = 0xFF;
	DDRB &= ~(1 << DDB0);
	PIND &= ~(1 << PD5);
	//Aktiviere PinChange Interrupt
	PCICR |= (1 << PCIE0);
    PCMSK0 |= (1 << PCINT0);
    //Initialisiere ADC
	ADC_Init();
	//Aktiviere Timer Interrupt
	TIMSK0 = _BV(OCIE0A);
	TCCR0A = _BV(WGM01);
	TCCR0B = _BV(CS02) | _BV(CS00);
	OCR0A = 255;
	TIMSK1 = _BV(OCIE1A);
	TCCR1B = _BV(CS12) | _BV(CS10);
	OCR1A = 20;
	//OCR1H = (1<<1) | (1<<2)
	//OCR1L = (1<<3) | (1<<4)

	//Initialisiere SPI
	SPI_Init();
	//Initisalisere Display
	Display_Init();
	//Initialisiere USART
	USART_Init(FOSC/16/BAUD-1, 1);
	//Schalte Interrupt scharf
	sei();
	while(1){
	if (EFlag==0){
		//Wenn PinChange Interrupt
		draw_Graph();
		if (TFlag==1){
			//Wenn Timer -> Frequenz 30/Hz
			TFlag=0;
			CFlag=0;
			insertDataGraph(ADC_Read(0));
		}
	}
	else if (EFlag==1){
		//Ansonsten
		if (CFlag==0){
			//Mache nur einmal Clear Screen
			clr_Screen();
			CFlag=1;
		}
		if (TFlag==1){
			//Zeichne ADC Werte
			USART_Transmit(0xFF);
			//USART_Transmit_16Bit(0x3A);
			//USART_Transmit_16Bit(0x025A);

			USART_Transmit_16Bit(ADC_Read(0));
			USART_Transmit_16Bit(ADC_Read(1));

			TFlag=0;
			draw_Number(ADC_Read(0));
			//draw_Text("Test");
		}
		}
}
}
ISR (TIMER0_COMPA_vect)
{
	TFlag=1;
}
ISR (TIMER1_COMPA_vect)
{
	SFlag=1;
}


ISR(PCINT0_vect)
{
	if(zustand == 0 && !(PINB & (1<<PINB0)))   //Taster wird gedrueckt (steigende Flanke)
    {
        zustand = 1;
    }

    else if (((zustand == 1) || (zustand == 2)) && !(PINB & (1<<PINB0)))   //Taster wird gehalten
    {
         zustand = 2;

         if (EFlag == 0)
			EFlag = 1;
		 else if (EFlag == 1)
			EFlag = 0;
    }

    else if (zustand == 2 && (PINB & (1<<PINB0)))   //Taster wird losgelassen (fallende Flanke)
    {
        zustand = 3;
    }

    else if (zustand == 3 && (PINB & (1<<PINB0)))   //Taster losgelassen
    {
        zustand = 0;
    }
}
