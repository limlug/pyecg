/* initialisiert die USART-Schnittstelle */
void USART_Init(uint8_t ubrr, uint8_t speed)
{
	UBRR0H = (unsigned char)(ubrr>>8);
	UBRR0L = (unsigned char)ubrr;
	UCSR0B = (1<<RXEN0)|(1<<TXEN0);
	UCSR0C = (3<<UCSZ00);

}

/* uertraegt 8Bit Frames an USART */
void USART_Transmit(uint8_t data)
{
while ( !( UCSR0A & (1<<UDRE0)) )
;
UDR0 = data;
}

/* empfaengt 8Bit Frames von USART */
uint8_t USART_Receive(void)
{
while ( !(UCSR0A & (1<<RXC0)) )
;
return UDR0;
}
/*
void USART_Transmit_16Bit_uniq(uint16_t data)
{
	//USART 16Bit Transmit über 9bit Flag
	//Here be Dragons
	while ( !( UCSR0A & (1<<UDRE0))) )
	;
	UCSR=B &= ~(1<<TXB8);
	if ( data & 0x0100 )
		UCSR0B |= (1<<TXB8);
	UDR0 = data;
	
}*/

void USART_Transmit_16Bit(uint16_t data)
{
	USART_Transmit(data >> 8);
	USART_Transmit(data);
}
/* empfaengt 16Bit Frames von USART */
uint16_t USART_Receive_16Bit(void)
{
	uint8_t firstReceived = USART_Receive();
	uint8_t secondReceived = USART_Receive();
	uint16_t bothReceived;
	bothReceived = firstReceived << 8;
	bothReceived |= secondReceived;
	return bothReceived;
}
