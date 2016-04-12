#include "8x8_v_33i.h"

#define CS PB2
#define CD PD2
#define RESET PD4

int nextDataPointPosition = 1;

void SPI_Init(void)
{	
	SPCR = (1<<SPE)|(1<<MSTR)|(1<<CPOL)|(1<<CPHA)|(1<<SPR0);
	SPSR = (1<<SPI2X);
}

/* SPI Transfer*/
void SPI_Transfer(uint8_t byte)
{
	SPDR = byte;
	while(!(SPSR & (1<<SPIF)))
		;
}

/* uertragt ein 8Bit-Befehl an das Display */
void sendCommand(uint8_t command)
{
	PORTB &= ~(1<<CS);
	PORTD &= ~(1<<CD);
	//PORTB &= ~(1<<CD); 

	SPDR = command;
	while(!(SPSR & (1<<SPIF)))
		;
	PORTB |= (1<<CS);

}

/* uertragt eine 8Bit-Spalte an das Display */
void sendData(uint8_t data)
{
	PORTB &= ~(1<<CS);
	//PORTB |= (1<<CD);
	PORTD |= (1<<CD);
	SPDR = data;
	while(!(SPSR & (1<<SPIF)))
		;
	PORTB |= (1<<CS);
}



/* Initialisiert das Display */
void Display_Init(void)
{
	//RESET
	PORTD &= ~(1<<RESET);
	_delay_ms(2);
	//RESET Los Lassen
	PORTD |= (1<<RESET);
	_delay_ms(5);
	//Initalisierungsbefehle
	sendCommand(0x40);
	sendCommand(0xA1);
	sendCommand(0xC0);
	sendCommand(0xA4);
	sendCommand(0xA6);
	sendCommand(0xA2);
	sendCommand(0x2F);
	_delay_ms(120);
	sendCommand(0x27);
	sendCommand(0x81);
	sendCommand(0x10);
	sendCommand(0xFA);
	sendCommand(0x90);
	sendCommand(0xAF);
	sendCommand(0b10110000);
	sendCommand(0b00000000);
	sendCommand(0b00010000);
	//Initialisierung Beendet
	//Mache Screen sauber
	for(uint8_t i=0; i<8; i++) {
		sendCommand(0b10110000 | i);
		for(uint8_t j=0; j<102; j++) {
			sendCommand(0b00001111 & j);
			sendCommand((0b00001111 & (j >> 4)) | (1 << 4));
			sendData(0x00);
		}
	
	}
}


/* zeichnet die hinterlegten Daten zu einem Char */
void draw_Char(uint8_t myChar)
{
	for(int i=0; i<9; i++){
		sendData(font[myChar][i]);
	}
}
void clr_Screen(void){
	//Mache Screen sauber
	for(uint8_t i=0; i<8; i++) {
		sendCommand(0b10110000 | i);
		for(uint8_t j=0; j<102; j++) {
			sendCommand(0b00001111 & j);
			sendCommand((0b00001111 & (j >> 4)) | (1 << 4));
			sendData(0x00);
		}
	
	}
}
/* Aufgabe 2.8 */
void draw_Square(void)
{	
	//Zeichne Ein Quadrat
	sendCommand(0b10110011);
	for(int j=16; j<24; j++) {
		sendCommand(0b00001111 & j);
		sendCommand((0b00001111 & (j >> 4)) | (1 << 4));
		sendData(0xFF);
	}
}
void draw_Number(uint16_t printNumber){
	//Zeichne eine arbiträre Zahl in Page 1
	int rest = printNumber;
	sendCommand(0b10110001);
	int i=0;
	while (rest != 0 ){
		draw_Char(rest % 10);
		i++;
		sendCommand(0b00001111 & (i*8));
		sendCommand((0b00001111 & ((i*8) >> 4)) | (1 << 4));
		rest = rest / 10;
	}
}
void draw_Graph(void){
	//Zeichne einen Graphen
	sendCommand(0b10110000);
	sendCommand(0b00000000);
	sendCommand(0b00010000);
	sendData(0xFF);
	sendCommand(0b10110001);
	sendCommand(0b00000000);
	sendCommand(0b00010000);
	sendData(0xFF);
	sendCommand(0b10110010);
	sendCommand(0b00000000);
	sendCommand(0b00010000);
	sendData(0xFF);
	sendCommand(0b10110011);
	sendCommand(0b00000000);
	sendCommand(0b00010000);
	sendData(0xFF);
	sendCommand(0b10110100);
	sendCommand(0b00000000);
	sendCommand(0b00010000);
	sendData(0xFF);
	sendCommand(0b10110101);
	sendCommand(0b00000000);
	sendCommand(0b00010000);
	sendData(0xFF);
	sendCommand(0b10110110);
	sendCommand(0b00000000);
	sendCommand(0b00010000);
	sendData(0xFF);
	sendCommand(0b10110111);
	sendCommand(0b00000000);
	sendCommand(0b00010000);
	sendData(0xFF);
	for(int j=1; j<102; j++) {
		sendCommand(0b00001111 & j);
		sendCommand((0b00001111 & (j >> 4)) | (1 << 4));
		sendData(0b10000000);
	}

}
void insertDataGraph(uint16_t data){
	//Zeichne Datenpunkte mit Faktor 20 gekürzt
	for(int i=0; i<8; i++) {
		sendCommand(0b10110000 | i);
		sendCommand(0b00001111 & nextDataPointPosition);
		sendCommand((0b00001111 & (nextDataPointPosition >> 4)) | (1 << 4));
		if (i==7){
			sendData(0b10000000);
		}
		else {
			sendData(0b00000000);
		}
	}
	sendCommand(0b10110000 | ((data/20)/8));
	sendCommand(0b00001111 & nextDataPointPosition);
	sendCommand((0b00001111 & (nextDataPointPosition >> 4)) | (1 << 4));
	sendData((1<<((data/20)%8)));
	nextDataPointPosition++;
	if (nextDataPointPosition>100) {
		nextDataPointPosition=1;
	}
}

void draw_Text(char* printtext){
	//Zeichne einen arbitären String 
	//Here be Dragons
	sendCommand(0b10110000);
	for(int j=0; j<=strlen(printtext); j++){
		char chartodraw;
		int chart;
		unsigned char ctd;
        sprintf(chartodraw, "%02X", printtext[j]);
        sscanf(chartodraw,"%x",&chart);
        ctd = chart & 0xFF;
		draw_Char(ctd);
	}
}

/* Zeichentabelle

Id	Hex	Wert
0	00	0
1	01	1
2	02	2
3	03	3
4	04	4
5	05	5
6	06	6
7	07	7
8	08	8
9	09	9
10	0A	A
11	0B	B
12	0C	C
13	0D	D
14	0E	E
15	0F	F
16	10	G
17	11	H
18	12	I
19	13	J
20	14	K
21	15	L
22	16	M
23	17	N
24	18	O
25	19	P
26	1A	Q
27	1B	R
28	1C	S
29	1D	T
30	1E	U
31	1F	V
32	20	W
33	21	X
34	22	Y
35	23	Z
36	24	.
37	25	:
38	26	(
39	27	)
40	28	-
41	29	+
42	2A	_
43	2B	SPACE
*/


