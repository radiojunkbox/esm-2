//-----------------------------------------------
// Firmware for ESM-2 Synthesizer Board
// Sapporo City Standard
// Author   : RJB
// Device   : ATmega88 / ATmega88P
// Clock    : 20MHz
// FUSES    : LOW=0xE6 HIGH=0xDF EXTENDED=0xF9
// Version  : 0.1 Sep. 06 2009
//            0.2 Sep. 26 2009
// Thanks to Motohiko Takeda (Bug fix for running status)
//-----------------------------------------------
// include

#include <avr\io.h>
#include <avr\interrupt.h>
#include "tables.h"

//-----------------------------------------------
// define

#define LED_GATE	PORTD1
#define LED_LFO		PORTD2
#define GATE_IN		PORTB4	
#define GATE_OUT	PORTB0

#define LED_GATE_ON		(PORTD |= (1 << LED_GATE))
#define LED_GATE_OFF	(PORTD &= ~(1 << LED_GATE))
#define LED_LFO_ON		(PORTD |= (1 << LED_LFO))
#define LED_LFO_OFF		(PORTD &= ~(1 << LED_LFO))
#define GATE_ON			(PORTB &= ~(1 << GATE_OUT))
#define GATE_OFF		(PORTB |= (1 << GATE_OUT))

#define	TRUE			1
#define	FALSE			0
#define ON				1
#define OFF				0

#define MAX_NOTE_CNT	8
#define	INTRCOUNT		178		// Timer0 interrupt interval 1ms
#define	BUFSIZE			16		// Size of ring buffer

// MIDI Messages
#define MIDI_NoteOff			0x80
#define MIDI_NoteOn				0x90
#define MIDI_PolykeyPressure	0xA0
#define MIDI_ControlChange		0xB0
#define MIDI_ProgramChange		0xC0
#define MIDI_ChannelPressure	0xD0
#define MIDI_PitchBend			0xE0

#define MIDI_StartSysEx			0xF0
#define MIDI_TuneRequest		0xF6
#define MIDI_EndSysEx			0xF7

#define MIDI_TimingClock		0xF8
#define MIDI_Start				0xFA
#define MIDI_Continue			0xFB
#define MIDI_Stop				0xFC
#define MIDI_ActiveSensing		0xFE
#define MIDI_SystemReset		0xFF

#define MIDI_CC_Moduration		0x01
#define MIDI_CC_DataEntry		0x06
#define MIDI_CC_RPN_LSB			0x64
#define MIDI_CC_RPN_MSB			0x65

#define MIDI_MM_AllSoundOff		0x78
#define MIDI_MM_ResetAllControl	0x79
#define MIDI_MM_AllNoteOff		0x7B

//-----------------------------------------------
// Global Variables

volatile unsigned char RxByte;
volatile unsigned char PC;
volatile unsigned char SysEx;
volatile unsigned char Rch;
volatile unsigned char MByte[2];
volatile unsigned char ch_bit;

volatile unsigned char PBBuf;
volatile unsigned char MWBuf;
volatile unsigned char RPNMsbBuf;
volatile unsigned char RPNLSBBuf;
volatile unsigned char NoteCnt;
volatile unsigned char NoteBuf[MAX_NOTE_CNT];
volatile unsigned char PlayNoteBuf;

volatile unsigned char MidiGate;
volatile unsigned char MidiTrig;
volatile unsigned char GateIn;

// for EG
volatile unsigned char EGState; // 1:ATK 2:DEC 3:SUS 0:REL
volatile unsigned int EGValue; // Q15 Format
volatile unsigned int EGAttack;
volatile unsigned int EGDecay;
volatile unsigned int EGSustain;
volatile unsigned int EGRelese;

// for ring buffer
volatile unsigned char RxBuf[BUFSIZE];
volatile int Ptr_buf_in, Ptr_buf_out; // pointer

// for LFO DDS
#define DDS_ACCU_MAX  0x10000
volatile unsigned int dds_diff;
volatile unsigned int dds_accu;
volatile unsigned int lfo_rate;
volatile unsigned int lfo_form;

//-----------------------------------------------
// Handler for Interrupt USART Rx Complete 

SIGNAL(USART_RX_vect)
{
	Ptr_buf_in++;
	Ptr_buf_in &= (BUFSIZE - 1);
	RxBuf[Ptr_buf_in] = UDR0;
}

//-----------------------------------------------
// Handler for Interrupt Timer0 Overflow

SIGNAL(TIMER0_OVF_vect)
{
	unsigned char	ph;
	int				lfo;
	int				mod;
	unsigned int	etmp;

	TCNT0 = INTRCOUNT;

	// Set ADC Channel
	ch_bit++;
	if( ch_bit > 5) ch_bit = 0; 
	ADMUX = ch_bit;
	ADCSRA = 0xCF;

	// Increment DDS Accumlator
	ph = dds_accu >> 8;
	dds_accu += dds_diff; 

	// Generate LFO Wave
	switch (lfo_form){
		case 0: // sin wave
			if(ph < 64)			lfo = LfoSinTbl[ph];
			else if( ph < 128)	lfo = LfoSinTbl[127 - ph];
			else if( ph < 192)	lfo = - LfoSinTbl[ph - 128] - 1;
			else				lfo = - LfoSinTbl[255 - ph] - 1;
			break;
		case 1:	// Triangle
			if(ph > 128)	lfo = 191 - ph;	
			else			lfo = ph - 64;
			break;
		case 2: // Sawtooth
			lfo = (ph >> 1) - 64;
			break;
		case 3: // Rectangle	
			if(ph > 128)	lfo = -64;	
			else			lfo = 63;
			break;
		default:
			lfo = 0;
	}

	// LFO LED
	if(ph > 128) 	LED_LFO_OFF;
	else			LED_LFO_ON;

	// Modulation Amount
	if( PINB & 0x20) 	mod = ((MWBuf * lfo) >> 2) + 0x200;
	else 				mod = ((0x1F  * lfo) >> 2) + 0x200;
	OCR1A = mod;

	// Gate
	GateIn |= (PINB & 0x10) ? 0x00 : 0x02;
	if( GateIn == 0x02) EGState = 1;
	GateIn = GateIn >> 1;

	if( MidiTrig) EGState = 1;
	MidiTrig = OFF;

	if( (MidiGate == OFF) && (GateIn == OFF)) {
		EGState = 0;
		LED_GATE_OFF;
	}
	else {
		LED_GATE_ON;
	}

	// Envelope Generator
	switch (EGState) {
		case 0: // RELEASE
			EGValue = ((unsigned long)EGValue * EGRelese) >> 15;
			break;
		case 1: // ATTACK
			etmp = ((unsigned long)(0x9FFF - EGValue) * EGAttack) >> 15;
			if( etmp <= 0x2000) {
				EGState = 2;
				etmp = 0x2000;
			}
			EGValue = 0x9FFF - etmp;
			break;
		case 2: // DECAY
			etmp = ((unsigned long)(EGValue - EGSustain) * EGDecay) >> 15;
			EGValue = etmp + EGSustain;
			if( etmp == 0) {
				EGState = 3;
			}
			break;
		case 3: // SUSTAIN
			EGValue = EGSustain;
			break;
	}	
    OCR1B = 0x3FF - (EGValue >> 5);
}

//-----------------------------------------------
// Handler for Interrupt ADC Complete

SIGNAL(ADC_vect)
{
	switch(ch_bit)
	{
		case 0: // LFO_RATE
			lfo_rate = LfoFreqTbl[(ADCW >> 2)];
			if(ADCW < 0x200) {
				dds_diff = (DDS_ACCU_MAX * lfo_rate) >> 17;
			}
			else {
				dds_diff = (DDS_ACCU_MAX * lfo_rate) >> 13;
			}
			break;
		case 1: // LFO_FORM
			lfo_form = ADCW >> 8;
			break;
		case 2: // ATTACK
			EGAttack = EGExpTbl[(ADCW >> 2)];
			break;
		case 3: // DECAY
			EGDecay = EGExpTbl[(ADCW >> 2)];
			break;
		case 4: // SUSTAIN
			if( EGState != 2) EGSustain = ADCW << 5;
			break;
		case 5: // RELEASE
			EGRelese = EGExpTbl[(ADCW >> 2)];
			break;
	}
}

//-----------------------------------------------
// Set CV

void SetCV(void)
{
	unsigned char note;

	note = PlayNoteBuf;
	if( note < 24) 	note = 24;
	note -= 24;

    OCR2A = 255 - (note << 1);
}

//-----------------------------------------------
// Note ON

void NoteON(void)
{
	unsigned char i;
	unsigned char max = 0;

	// disregard when registerd in buffer
	for(i = 0; i < NoteCnt; i++) {
		if(NoteBuf[i] == MByte[0]) {
			return;
		}
	}

	// buffer fullH
	if(NoteCnt == MAX_NOTE_CNT) {
		for(i = 0; i < (MAX_NOTE_CNT - 1); i++) {
			NoteBuf[i] = NoteBuf[i+1];
		}
		NoteBuf[MAX_NOTE_CNT - 1] = MByte[0];
	}
	else {
		NoteBuf[NoteCnt] = MByte[0];
		NoteCnt++;
	}

	// Highest Note
	for(i = 0; i < NoteCnt; i++) {
		if(max < NoteBuf[i]) {
			max = NoteBuf[i];
		}
	}
	PlayNoteBuf = max;

	SetCV();
	MidiGate = ON;
	MidiTrig = ON;
	GATE_ON;
}

//-----------------------------------------------
// Note OFF

void NoteOFF(void)
{
	unsigned char i;
	unsigned char max = 0;
	unsigned char flg = FALSE;

	// Delete Note
	for(i = 0; i < NoteCnt; i++) {
		if(flg) {
			NoteBuf[i-1] = NoteBuf[i];
		}
		if(NoteBuf[i] == MByte[0]) {
			flg = TRUE;
		}
	}
	if(flg) NoteCnt--;

	if(NoteCnt == 0) { // Note Buffer Empty
		// Gate OFF
		MidiGate = OFF;
		GATE_OFF;
	}
	else {
		//  Highest Note
		for(i = 0; i < NoteCnt; i++) {
			if(max < NoteBuf[i]) {
				max = NoteBuf[i];
			}
		}
		PlayNoteBuf = max;
		SetCV();
	}
}

//-----------------------------------------------
// Initialize

void Initialize(void)
{
	GATE_OFF;
	LED_GATE_OFF;
	LED_LFO_OFF;

	PBBuf = 128;
	MWBuf = 0;
	NoteCnt = 0;
	RPNMsbBuf = 127;
	RPNLSBBuf = 127;

	MidiGate = OFF;
	MidiTrig = OFF;

	EGState = 0;
	EGValue = 0x0000;
}

//-----------------------------------------------
// MIDI System Realtime Message

void MIDI_SystemRealtimeMessage(void)
{
	switch(RxByte) {
		case MIDI_TimingClock:
			break;
		case MIDI_Start:
			break;
		case MIDI_Continue:
			break;
		case MIDI_Stop:
			break;
		case MIDI_ActiveSensing:
			break;
		case MIDI_SystemReset:
			Initialize();
			break;
	}
}

//-----------------------------------------------
// MIDI System Message

unsigned char MIDI_SystemMessage(void)
{
	if(SysEx){
		if(RxByte == MIDI_EndSysEx){
			SysEx = FALSE;
		}
	}
	else{
		if(RxByte < 0xF8){
			if(RxByte > 0x7F){
				if(RxByte == MIDI_StartSysEx){
					SysEx = TRUE;
				}
				else{
					Rch = RxByte & 0x0F;
                }
				PC = 0;
 			}
			else{
				MByte[PC & 0x01] = RxByte;
			}
			return TRUE;
		}
		else {
			MIDI_SystemRealtimeMessage();
		}
	}
	return FALSE;
}

//-----------------------------------------------
// MIDI Channel Messag

void MIDI_ChannelMessage(void)
{
	switch(PC){
		case 0:
			switch(RxByte & 0xF0){
			case MIDI_NoteOff:
				PC = 2;
				break;
			case MIDI_NoteOn:
				PC = 4;
				break;
			case MIDI_PolykeyPressure:
				PC = 6;
				break;
			case MIDI_ProgramChange:
				PC = 8;
				break;
			case MIDI_ControlChange:
				PC = 10;
				break;
			case MIDI_ChannelPressure:
				PC = 12;
				break;
			case MIDI_PitchBend:
				PC = 14;
				break;
			} break;

        // Note OFF
		case 2:
			PC = 3;
			break;
        case 3:
			PC = 2;
			NoteOFF();
			break;

        // Note ON
        case 4:
			PC = 5;
			break;
        case 5:
			PC = 4;
			if( MByte[1] == 0){
				NoteOFF();
			}
			else{
				NoteON();
			}
			break;

		// Polyphonic key pressure 
        case 6:
			PC = 7;
			break;
        case 7:
			PC = 6;	// fix for running status
			break;

        // Program change
        case 8:
 			break;

        // Control Change
        case 10: PC = 11; break;
        case 11:
			switch(MByte[0]) {
				case MIDI_CC_Moduration:
					MWBuf = MByte[1] >> 2;
					break;
				case MIDI_CC_DataEntry:
					break;
				case MIDI_CC_RPN_LSB:
					RPNLSBBuf = MByte[1];
					break;
				case MIDI_CC_RPN_MSB:
					RPNMsbBuf = MByte[1];
					break;
				case MIDI_MM_AllSoundOff:
					NoteCnt = 0;
					MidiGate = OFF;
					GATE_OFF;
					LED_GATE_OFF;
					break;
				case MIDI_MM_ResetAllControl:
					PBBuf = 128;
					MWBuf = 0;
					RPNMsbBuf = 127;
					RPNLSBBuf = 127;
					break;
				case MIDI_MM_AllNoteOff:
					NoteCnt = 0;					
					MidiGate = OFF;
					GATE_OFF;
					LED_GATE_OFF;
					break;
			}
			break;

        // Channel Pressure
		case 12:
			break;

        // Pitch Bend
		case 14:
			PC = 15;
			break;
		case 15:
			PC = 14;	// fix for running status
			PBBuf = (MByte[1] << 1) | (MByte[0] >> 6);
			OCR2B = PBBuf;
			break;

		default:
			break;
    }
}

//-----------------------------------------------
// Main function

int main(void)
{
	DDRB = 0x0F;
	DDRC = 0x00;
	DDRD = 0x0E;

	Initialize();

	// ADC Initialize
	ADCSRA = 0x00;	// disable ADC
	ADCSRB = 0x00;	// Auto Triger None
	ADMUX = 0x00;	// select ADC input 0
	DIDR0 = 0x03;
	ch_bit = 0x01;

	// USART Initialize
	// CLOCK=20MHz BAUD=31.25kHz
	// UBRR = (CLOCK / (16BAUD)) - 1 = 39
	UCSR0B = 0x00;
	UCSR0A = 0x00;
	UCSR0C = 0x06;	// 8bit STOP1bit
	UBRR0L = 0x27;	// 39
	UBRR0H = 0x00;	
	UCSR0B |= (1 << RXCIE0) | (1 << RXEN0);
	Ptr_buf_in = Ptr_buf_out = 0;

	// Timmer0 Initialize 
	TCCR0A = 0x00;
	TCCR0B = 0x04; // 1/256Clk
	TIFR0 |= (1 << TOV0);
	TIMSK0 |= (1<< TOIE0);
	TCNT0 = INTRCOUNT;

	// Timer1(PWM) Initialize
	TCCR1B = 0x00;
	TCCR1A = 0xF3;
    OCR1A = 0x0200;	// LFO 
    OCR1B = 0x0200;	// EG
	TCCR1B = 0x09;	// START CLK/1

	// Timer2(PWM) Initialize
	TCCR2B = 0x00;	// STOP
	TCCR2A = 0xF3;
    OCR2A = 0xFF;	// CV
	OCR2B = 0x80;	// PITCH BEND
	TCCR2B = 0x01;	// START CLK/1

	sei();

	// Main loop
	while(TRUE) {
		// ring buffer emptyH
		if(Ptr_buf_in == Ptr_buf_out) 	continue;

		// get byte from ring buffer
		Ptr_buf_out++;
		Ptr_buf_out &= (BUFSIZE - 1);
		RxByte = RxBuf[Ptr_buf_out];

		if(MIDI_SystemMessage()) {
			if(Rch == 0) {
				MIDI_ChannelMessage();
			}
		}
	}
	return(0);
}
