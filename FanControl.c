/*
 * FanControl v3.0 - Fan Kitchen v2.0 Dimmer
 *
 * Created: 03.04.2023
 *  Author: Vadim Kulakov, vad7@yahoo.com
 *
 * ATtiny84A
 *
 * Radio nRF24L01+
 * IR TL1838V
 * Relay G3MB-202PL * 4
 * VCC 5V, 3.3V for nRF24
 */ 
#define F_CPU 8000000UL
// Fuses: BODLEVEL = 1.8V (BODLEVEL[2:0] = 110), RSTDISBL=0, EESAVE=0

//#define DEBUG_PROTEUS

#include <stdlib.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <avr/wdt.h>
#include <avr/pgmspace.h>
#include <avr/eeprom.h>
#include <util/delay.h>
#include <util/atomic.h>

const char ProgramID[] PROGMEM = "Fan Kitchen v2 DIM";	// CStr, [0]

#define KEY							(1<<PORTB2)
#define KEY_PRESSING				!(PINB & KEY)
#define KEY_SETUP					

#define COOKER_HOOD					// есть кухонная вытяжка
#ifdef COOKER_HOOD
#define FanCooker_PORT				PORTA
#define FanCooker_DDR				DDRA
#define FanCooker_PORT_ADDR			(0x20 + FanCooker_PORT)
#define FanCooker_PIN				PINA
#define FanCooker_Out1				(1<<PORTA7)	// F1.1
#define FanCooker_Out2				(1<<PORTA0)	// F1.2
#define FanCooker_OFF				FanCooker_PORT &= ~(FanCooker_Out1 | FanCooker_Out2)
#define FanCooker_Out2_ON			FanCooker_PORT |= FanCooker_Out2
#define FanCooker_Out2_OFF			FanCooker_PORT &= ~FanCooker_Out2
#endif

#define Fan_PORT					PORTB
#define Fan_DDR						DDRB
#define Fan_PORT_ADDR				(0x20 + Fan_PORT)
#define Fan_PIN						PINB
#define Fan_Out1					(1<<PORTB1)	// F2.1
#define Fan_Out2					(1<<PORTB0)	// F2.2
#define Fan_OFF						Fan_PORT &= ~(Fan_Out1 | Fan_Out2)

#define Fans_SETUP					FanCooker_DDR |= FanCooker_Out1 | FanCooker_Out2; Fan_DDR |= Fan_Out1 | Fan_Out2

#define LED1_PORT					PORTA	// Info LED(s)
#define LED1_DDR					DDRA
#define LED1_TWIN							// 2 LED - to VCC & to GND
#define LED1						(1<<PORTA1)
//register uint8_t led1 asm("10");	// control var: 0 - off, 1 - on (Attiny44A)
uint8_t led1;						// Attiny84A
#define LED1_ON						led1 = 1
#define LED1_OFF					led1 = 0
#ifndef LED1_TWIN					// ONE LED:
#define LED1_INIT					LED1_DDR |= LED1
#define _LED1_ON					LED1_PORT |= LED1
#define _LED1_OFF					LED1_PORT &= ~LED1
#define _LED1_STANDBY
#define LED1_ON_BEFORE_RESET		_LED1_ON
#else								// TWIN LED:		
#define LED1_INIT
#define _LED1_ON					LED1_PORT |= LED1; LED1_DDR |= LED1
#define _LED1_OFF					LED1_DDR &= ~LED1
#define _LED1_STANDBY				LED1_PORT &= ~LED1; LED1_DDR |= LED1
#define LED1_ON_BEFORE_RESET		LED1_DDR |= LED1
#endif

#define ZERO_CROSS_PORT				PORTB
#define ZERO_CROSS					(1<<PORTB3)
#define ZERO_CROSS_INIT				{	GIMSK |= (1<<PCIE1); /* Pin Change Interrupt Enable */\
										PCMSK1 = (1<<PCINT11); } /* Pin Change Mask Register - ONE PIN - Zero Cross */

//const uint8_t FanPORTS[] PROGMEM = { Fan_Speed1, Fan_Speed2,  } // FanSpeed_1 - FanCookerSpeed_3

#define Fan_DamperSwitchTime		25	// sec
#define FANS_SLEEP_STEP				30	// minutes
#define FAN_IDXS					3	// How many speeds indices does each fan have

//uint8_t FanSpeed					= 0; // Current fan speed
//uint8_t FanSleep					= 0; // Current fan sleep
uint8_t SleepTimer					= 0; // *FANS_SLEEP_STEP minutes
uint8_t OutSpeedMax;					// full power time
uint8_t FanPortADDR;				// global address of PORTA/PORTB for current fan 
uint8_t FanPortPIN;					// Turn on PORT value for current fan
uint8_t PayloadPortADDR;			// global address of PORTA/PORTB for Lamp/Damper if non zero, inited with not used IO!
uint8_t PayloadPortPIN;				// Turn on PORT value for Lamp/Damper

#define	IR_MAX_CONTROLS				10 // Max different remote controls
#define	IR_PULSES_MIN				8 // IR packet minimum pulses
enum {
	IR_WAITING = 0,
	IR_START,
	IR_READING, 
	IR_DONE
};
enum {
	IR_Key_Setup = 0,
	IR_Key_Off,
	IR_Key_FanUp,			// Fan (from OFF - speed 3)
	IR_Key_FanDown,			// Fan (from OFF - speed 1)
	IR_Key_CookerUp,		// or right - FanCooker + (from OFF - speed 3)
	IR_Key_CookerDown,		// or left  - FanCooker - (from OFF - speed 1)
	IR_Key_Cooker_Speed_1,
	IR_Key_Cooker_Speed_2,
	IR_Key_Cooker_Speed_3,
	IR_Key_CookerLight
};
#define IR_Keys_Total				(IR_Key_Cooker_Speed_3 + 1)		// number of keys
uint8_t IR_Status					= IR_WAITING;
uint8_t IR_LastDuration;
uint16_t IR_Hash					= 0;
uint16_t IR_Hash_Last				= 0;
uint8_t IR_Cnt						= 0;
uint8_t IR_CntLast					= 0;

uint8_t IRRepeatDelay				= 0;	// *0.1 sec
uint8_t IRRepeatCnt					= 0;
uint8_t Key1Pressed					= 0;	// *20ms
uint8_t Key1Pause					= 0;
volatile uint8_t Timer				= 0;	// sec
volatile uint8_t TimerMin			= 0;	// minutes
uint8_t TimerSecCnt					= 0;
uint8_t Flags;
uint8_t FanOn						= 0;	// 0 - off, 1..6 - FanSpeed_1 ... FanCookerSpeed_3
uint8_t FanSpeed					= 0;	// current speed
uint8_t FanOnNext					= 0;	// after timeouts next Fan speed
uint8_t FanOnLast					= 0;	// before FanOnNext
uint8_t FanOnNextCnt				= 0;	// sec
uint8_t SetupItem					= 0;
uint8_t Setup						= 0;	// Setup enum
uint8_t CookerLight_force_on		= 0;
uint8_t SetFanSpeed_by_CO2			= 0;	// 1 - change due to CO2 level

enum {
	fSetup_Off = 0,
	fSetup_IR,
	fSetup_Speed
};

#define REPEAT_TIMES_SETUP_SPEED	3
#define REPEAT_TIMES_SETUP_IR		5
#define IR_REPEAT_TIMEOUT			2		// *0.1 sec
#define IR_REPEAT_TIMEOUT_SETUP		50		// *0.1 sec
#define KEY_PressingTimeMin			4		// *20 msec

// EEPROM.Flags:
#define f_NRF24						(1<<0)	// Present nRF24L01
#define f_FanHiSpeed				(1<<1)	// Present Fan Hi speed - second motor coil - Port: FAN2.2 (Fan_Out2)
#define f_FanCookerHiSpeed			(1<<2)	// Present FanCooker Hi speed - second motor coil - Port: FAN1.2 (FanCooker_Out2)
#define f_CookerLamp_FanOut2		(1<<3)	// Present Cooker Lamp on Fan out 2 (FAN2.2)
#define f_FanDamper					(1<<4)	// Present Fan Damper - Port: FAN2.2 (Fan_Out2). Damper turn on before Fan start and is ON all time when Fan is ON
#define f_PowerSaving				(1<<5)	// Reduce SSR power consuming (Lamp & Damper pulse width = SSR_PulseWidth)
//#define f_FanCookerDamper			(1<<6)	// Present FanCooker - Port: FAN2.2 (Fan_Out2). Damper is ON all time when Fan is ON
//#define f_Damper					(1<<7)	// Central damper - one for all fans (FAN2.2)

uint8_t Fan_Out2_status				= 0;	// On/Off
#define Fan_Out2_ON					{ if(Flags & f_PowerSaving) { PayloadPortADDR = (uint16_t)&Fan_PORT; PayloadPortPIN = Fan_Out2; } else Fan_PORT |= Fan_Out2; Fan_Out2_status = 1; }
#define Fan_Out2_OFF				{ if(Flags & f_PowerSaving) { *(uint8_t*)(uint16_t)PayloadPortADDR &= ~PayloadPortPIN; PayloadPortADDR = (uint16_t)&OCR0B; } else Fan_PORT &= ~Fan_Out2; Fan_Out2_status = 0; } // OCR0B - not used IO

struct _EEPROM {
/* 0*/	uint8_t _OSCCAL;
/* 1*/	uint8_t Flags;				// Flags
/* 2*/	uint8_t RF_Address;			// nRF24 address LSB
/* 3*/	uint8_t RF_Channel;			// nRF24 channel
/* 4*/	uint16_t CO2_threshold1;	// CO2 threshold to start Fan speed 1 (if FanCooker is OFF)
/* 6*/	uint16_t CO2_threshold2;	// CO2 threshold to start Fan speed 2 (if FanCooker is OFF)
/* 8*/	uint16_t CO2_threshold3;	// CO2 threshold to start Fan speed 3 (if FanCooker is OFF)
/*10*/	uint8_t FanStartupMaxSpTime;// sec forced max speed or Damper on time if f_FanDamper = true
/*11*/	uint8_t FanShutdownTime;	// Fan turning off time (used for switch from Fan to FanCooker), sec
/*12*/	uint8_t FanCookerStartupMaxSpTime; // sec forced max speed 
/*13*/	uint8_t FanCookerShutdownTime;// FanCooker turning off time (used for switch from FanCooker to Fan), sec
/*14*/	uint8_t FanSleep;			// Auto sleep time, *FANS_SLEEP_STEP
/*15*/	uint8_t FanCookerSleep;		// Auto sleep time, *FANS_SLEEP_STEP
/*16*/	uint8_t SpeedInitIdx;		// Power up Fans speed index: 0 - all off, 1 - Fan speed1, 2 - Fan speed2, 3 - Fan speed3, 4 - FanCooker speed 1, 5 - FanCooker speed 2, 6 - FanCooker speed 3
/*17*/	uint8_t SpeedKeyIdx;		// When key pressed
/*18*/	uint8_t OutSpeedMax;		// must be less than 52
// Idx: From 0 (off) to OutSpeedMax * 2, if > OutSpeedMax then second coil used if available
/*19*/	uint8_t FanSpeed_1;
/*20*/	uint8_t FanSpeed_2;
/*21*/	uint8_t FanSpeed_3;
/*22*/	uint8_t FanCookerSpeed_1;
/*23*/	uint8_t FanCookerSpeed_2;
/*24*/	uint8_t FanCookerSpeed_3;
	//
/*25*/	uint8_t FanSpeeds;			// Number of speeds (1..FAN_IDXS)
/*26*/	uint8_t FanCookerSpeeds;	// Number of speeds (1..FAN_IDXS)
/*27*/	uint8_t PauseBetweenSetByCO2;// minutes
/*28*/  uint16_t SSR_PulseWidth;	// us
/*30*/  uint16_t SSR_PulseSafeTime; // off time after pulse end, us
/*32*/  uint16_t PayloadPulseWidth;	// us
/*34*/	uint8_t DamperOpenTime;		// sec
/*35*/	uint8_t FanCookerSpIdxDownKey;// Speed index when first DOWN key pressed
/*36*/	uint8_t FanCookerSpIdxUpKey;// Speed index when first UP key pressed
/*37*/	uint8_t IRRemotes;			// Total active remote controls
/*38*/	uint16_t IRCommandArray[IR_MAX_CONTROLS * IR_Keys_Total]; // type like IRHash
} __attribute__ ((packed));
struct _EEPROM EEMEM EEPROM;

#define FANS_IDX_EEPROM_OFFSET		&EEPROM.FanSpeed_1
#define GET_FANS_IDX_EEPROM(a)		(a - FANS_IDX_EEPROM_OFFSET + 1) // 1..FAN_IDXS*2

#define fCMD_Write					0x80 // EEPROM[Type] = Data, "fCMD_WriteStart" must be preceded, timeout - fCMD_Write_Timeout
#define fCMD_Read					0x40 // read MEM[Data] => Data, MEM: EEPROM, MAIN, PROGRAM
#define fCMD_Set					0xC0 // Set cmd, Type = cmd id, Data = cmd value
#define fCMD_WriteStart				0x2F
// fCMD +
#define fCMD_EEPROM					0x00	// EEPROM
#define fCMD_RAM					0x10	// RAM memory
#define fCMD_PROGMEM				0x20	// Program FLASH
#define fCMD_1b						0x01
#define fCMD_2b						0x02
#define fCMD_4b						0x03
#define fCMD_8b						0x04
#define fCMD_CStr					0x05	// #0 = ProgramID

#define Type_Set_Lamp				0	// Lamp ON/OFF, bit num
#define Type_Set_Fan				1	// Set Fan idx = Data
#define Type_Set_FanAdd				2	// Fan idx += Data
#define Type_Set_FanSpeedUp			3	// FanSpeed +1
#define Type_Set_FanSpeedDown		4	// FanSpeed -1
#define Type_Set_FanSpeedSave		5	// Activate Setup IR mode, bit num
#define Type_Set_SetupIR			6	// Save FanSpeed to working fan idx
#define Type_Set_RESET				14	// Restart program, software reset (Data = 0xEEEE)

#define fCMD_Write_Timeout			3	// *0.1 sec

struct SETUP_DATA { // the same size as SEND_DATA!
	uint16_t Data;	// Read/Write byte or word
	uint8_t Type;	// Read data type(SetupType.*) or Write EEPROM address
	uint8_t Flags;	// Setup command: fSetup_*
} __attribute__ ((packed));

struct SEND_DATA {
	uint16_t CO2level;
	uint8_t FanSpeed;
	uint8_t Flags;
} __attribute__ ((packed));
struct SEND_DATA data;
uint8_t WriteTimeout = 0;

#if(1)
void Delay10us(uint8_t ms) {
	while(ms-- > 0) _delay_us(10); 
	wdt_reset();
}
//void Delay1ms(uint8_t ms) {
//	while(ms-- > 0) {
//		_delay_ms(1); wdt_reset();
//	}
//}
void Delay100ms(unsigned int ms) {
	while(ms-- > 0) {
		_delay_ms(100); wdt_reset();
	}
}

void FlashLED(uint8_t num, uint8_t toff, uint8_t ton) {
	while (num-- > 0) {
		LED1_OFF;
		Delay100ms(toff);
		LED1_ON;
		Delay100ms(ton);
	}
	LED1_OFF;
}

#endif

#include "nRF24L01.h"

#define SETUP_WATCHDOG WDTCSR = (1<<WDCE) | (1<<WDE); WDTCSR = (1<<WDE) | (0<<WDIE) | (0<<WDP3) | (1<<WDP2) | (1<<WDP1) | (0<<WDP0);	//  Watchdog 1 s

//											// 0xF0 mask - Number of long flashes, 0x0F mask - Number of short flashes
#define WRN_SETUP					0x01
#define WRN_SETUP_INFO				0x02
#define WRN_FAN_SWITCHING			0x10
#define WRN_SETUP_ERR				0x10	// +ERR
#define WRN_RF_Receive				0x01
#define WRN_RF_Send					0x10	// Send failure, after short bursts = fan offset (mask 0x0F)
#define WRN_RF_SetAddr				0x20	// Set addresses failure,
#define WRN_RF_NotResp				0x30	// RF module not response,
#define WRN_CO2Sensor				0x40	// CO2 Sensor reading failure

//#define TIM0_INIT					TCCR0B = (1<<WGM02) | (1 << CS02) | (0 << CS01) | (1 << CS00); // Timer0 prescaller: 256
#define ZERO_CROSS_TIMER_TOP		19999	// 50Hz
#define ZERO_CROSS_TIMER_10ms		(ZERO_CROSS_TIMER_TOP / 2)
uint16_t TCNT1_zero;
uint16_t TCNT1_prev = 0;
uint16_t after_zero_cross_pulse_delay;// value for OCR0A
uint16_t SSR_pulse_width;			// value for OCR0B for SSR pulse
uint8_t SSR_pulse_status = 0;		// 0 - off, 1 - wait for pulse start, 2 - wait for pulse end
uint16_t SSR_full_period = ZERO_CROSS_TIMER_10ms;	// previous
uint16_t SSR_full_period_prev = ZERO_CROSS_TIMER_10ms;
uint16_t SSR_full_period_last = ZERO_CROSS_TIMER_10ms;
uint8_t PayloadPulseWidth;
uint8_t LED_Warning = 0, LED_WarningOnCnt = 0, LED_WarningOffCnt = 0, LED_Warning_WorkLong = 0, LED_Warning_WorkShort = 0;
uint8_t LED_WarningPause = 0;
uint8_t TimerCnt100ms = 0;
uint8_t TimerCnt = 0;
uint8_t Timer1sec = 0;
uint8_t TimerCntMin = 0;
uint16_t TimerCntFanSleepStep = 0;

ISR(ANA_COMP_vect) // continue Zero cross (not used INT)
{
	if(FanSpeed == OutSpeedMax) {
		*(uint8_t*)(uint16_t)FanPortADDR |= FanPortPIN;		// On
	} else if(FanSpeed) {
		//*FanPortADDR &= ~FanPortPIN;	// Off
		SSR_pulse_status = 0;
		uint16_t n = TCNT1_zero + after_zero_cross_pulse_delay;
		if(n > ZERO_CROSS_TIMER_TOP) n = n - ZERO_CROSS_TIMER_TOP + 1;
		OCR1A = n;	// pulse on
		n += SSR_pulse_width;
		if(n > ZERO_CROSS_TIMER_TOP) n = n - ZERO_CROSS_TIMER_TOP + 1;
		OCR1B = n;	// pulse off
	} else {
		OCR1B = OCR1A = 0xFFFF;
		*(uint8_t*)(uint16_t)FanPortADDR &= ~FanPortPIN;	// Off
	}
	sei(); // allow interrupts
	SSR_full_period_prev = SSR_full_period_last;
	SSR_full_period_last = TCNT1_zero >= TCNT1_prev ? TCNT1_zero - TCNT1_prev : ZERO_CROSS_TIMER_TOP - TCNT1_prev + TCNT1_zero;
	SSR_full_period = SSR_full_period_prev < SSR_full_period_last ? SSR_full_period_prev : SSR_full_period_last;
	TCNT1_prev = TCNT1_zero;
#ifdef DEBUG_PROTEUS
	// fix Proteus bug (no call) for TIM1_CAPT timer interrupt in CTC mode
	static uint8_t debug_TIM1_fix_cnt = 0;
	if(++debug_TIM1_fix_cnt == 2) {
		debug_TIM1_fix_cnt = 0;
		__asm__ ("LDI R30, %0" :: "I" (TIM1_CAPT_vect_num));
		__asm__ ("LDI R31, 0");
		__asm__ ("ICALL");
	}
#endif
	uint8_t dly;
	if(PayloadPortADDR && (dly = PayloadPulseWidth)) {
		while(dly-- > 0) _delay_us(10);
		*(uint8_t*)(uint16_t)PayloadPortADDR &= ~PayloadPortPIN; // Off
	}
}

ISR(PCINT1_vect)		// Zero cross
{
	*(uint8_t*)(uint16_t)PayloadPortADDR |= PayloadPortPIN;		// On
	TCNT1_zero = TCNT1;
	__asm__ ("LDI R30, %0" :: "I" (ANA_COMP_vect_num));			// call ANA_COMP_vect interrupt
	__asm__ ("LDI R31, 0");
	__asm__ ("ICALL");
}

ISR(TIM1_COMPA_vect)	// pulse on
{
	*(uint8_t*)(uint16_t)FanPortADDR |= FanPortPIN;		// On
	OCR1AH = 0xFF;
	OCR1AL = 0xFF;
}

ISR(TIM1_COMPB_vect)	// pulse off
{
	*(uint8_t*)(uint16_t)FanPortADDR &= ~FanPortPIN;	// Off
	OCR1BH = 0xFF;
	OCR1BL = 0xFF;
}

ISR(TIM1_CAPT_vect, ISR_NOBLOCK)	// 0.02 sec
{
// Bresenham's line algorithm for power control
//	static int8_t Out_accum = 0;
// 	if(FanOn) {
// 		Out_accum += FanSpeed;
// 		if(Out_accum >= OutPeriod){
// 			Out_accum -= OutPeriod;
// 			*FanPortADDR |= FanPortPIN;		// On
// 		} else *FanPortADDR &= ~FanPortPIN;	// Off
// 	}
	if(++TimerCnt100ms >= 5) { // 0.1 sec
		TimerCnt100ms = 0;
		if(++TimerCnt >= 10) { // 1 sec
			TimerCnt = 0;
			Timer1sec = 1;
			if(++TimerCntMin >= 60) { // 1 min
				TimerCntMin = 0;
				if(TimerMin) TimerMin--;
			}
		}
		if(LED_WarningPause) {
			if(--LED_WarningPause == 0) LED1_OFF;
		} else {
			// LED_Warning: 0xF0 mask - Number of long flashes, 0x0F mask - Number of short flashes, LED_Warning_NoRepeat = no repeat
			if(LED_WarningOnCnt) {
				LED1_ON;
				LED_WarningOnCnt--;
			} else if(LED_WarningOffCnt) {
				LED1_OFF;
				LED_WarningOffCnt--;
			} else if(LED_Warning_WorkLong) { // long flashes
				LED_Warning_WorkLong--;
				LED_WarningOnCnt = 14;			// 1.4s (*0.1s)
				if(LED_Warning_WorkLong == 0) {
					LED_WarningOffCnt = 6;		// 0.6s
					goto xSetPause;
				} else LED_WarningOffCnt = 4;	// 0.4s
			} else if(LED_Warning_WorkShort) { // short flashes
				LED_Warning_WorkShort--;
				LED_WarningOnCnt = 3;	// 0.3s
				LED_WarningOffCnt = 3;	// 0.3s
xSetPause:
				if(LED_Warning_WorkShort == 0) LED_WarningOffCnt = 25; // 2.5s
			} else if(LED_Warning) {
				LED_Warning_WorkLong = (LED_Warning & 0xF0) >> 4;
				LED_Warning_WorkShort = LED_Warning & 0x0F;
				LED_Warning = 0;
			}
		}
		if(IRRepeatDelay) IRRepeatDelay--;
		if(WriteTimeout) WriteTimeout--;
	}
	if(Key1Pause) Key1Pause--;
	if(led1) {
		_LED1_ON;
	} else {
		if(FanSpeed || Fan_Out2_status || (FanCooker_PORT & FanCooker_Out2) || (Fan_PORT & Fan_Out2)) { _LED1_OFF; } else { _LED1_STANDBY; }
	}
}


ISR(TIM0_OVF_vect) // IR timeout, 20 ms
{
	if(IR_Status == IR_READING) {
		if(IR_Cnt > IR_PULSES_MIN) IR_Status = IR_DONE; else IR_Status = IR_WAITING;
	} else if(IR_Status == IR_START) IR_Status = IR_WAITING;
	if(IR_Status != IR_DONE) {
		if(KEY_PRESSING) {
			if(!Key1Pause && Key1Pressed < 255) Key1Pressed++;
		} else {
			if(Key1Pressed <= KEY_PressingTimeMin) Key1Pressed = 0;
		}
	}
}

ISR(EXT_INT0_vect) // IR, PIN change
{
	if(IR_Status <= IR_READING) {
		uint8_t _TCNT = TCNT0;
		TCNT0 = 0;
		if(IR_Status == IR_WAITING) {
			IR_Status = IR_START;
		} else if(IR_Status == IR_START) {
			IR_Cnt = 0;
			IR_Hash = 5381; // hash init
			IR_LastDuration = _TCNT;
			IR_Status = IR_READING;
		} else {
			uint8_t n;
			if(_TCNT < IR_LastDuration * 3 / 4) n = 1;
			else if(IR_LastDuration < _TCNT * 3 / 4) n = 2;
			else n = 0;
			IR_Hash = ((IR_Hash << 5) + IR_Hash) ^ n;
			IR_LastDuration = _TCNT;
			if(++IR_Cnt == 0) IR_Status = IR_DONE;
		}
	}
}

void Set_LED_Warning(uint8_t d)
{
	if(LED_Warning == 0) LED_Warning = d;
}

void Set_LED_Warning_New(uint8_t d)
{
	LED_Warning_WorkLong = LED_Warning_WorkShort = LED_WarningOnCnt = LED_WarningOffCnt = 0;
	LED_Warning = d;
	LED1_OFF;
}

void ResetSettings(void)
{
	eeprom_update_byte(&EEPROM.Flags, f_NRF24 | f_FanCookerHiSpeed | f_FanDamper | f_PowerSaving);// Flags (0x35)
 	eeprom_update_byte(&EEPROM.RF_Address, 0xC1);		// nRF24 address LSB
 	eeprom_update_byte(&EEPROM.RF_Channel, 122);		// nRF24 channel
 	//eeprom_update_byte(&EEPROM.OutPeriod, 15);			// Fans period of regulation (number 1/100Hz halfwaves), max speed value
 	eeprom_update_byte(&EEPROM.SpeedInitIdx, 0);		// Power up Fans speed index: 0 - all off, 1 - Fan speed1, 2 - Fan speed2, 3 - Fan speed3, 4 - FanCooker speed 1, 5 - FanCooker speed 2, 6 - FanCooker speed 3
 	eeprom_update_byte(&EEPROM.IRRemotes, 0);

	eeprom_update_word(&EEPROM.CO2_threshold1, 820);// CO2 threshold to start Fan speed 1 (if FanCooker is OFF)
	eeprom_update_word(&EEPROM.CO2_threshold2, 910);	// CO2 threshold to start Fan speed 2 (if FanCooker is OFF)
	eeprom_update_word(&EEPROM.CO2_threshold3, 1000);	// CO2 threshold to start Fan speed 3 (if FanCooker is OFF)
	eeprom_update_byte(&EEPROM.FanStartupMaxSpTime, 1);// sec forced max speed
	eeprom_update_byte(&EEPROM.FanShutdownTime, 100);	// Fan turning off time (used for switch from Fan to FanCooker), sec
	eeprom_update_byte(&EEPROM.FanCookerShutdownTime, 0);// FanCooker turning off time (used for switch from FanCooker to Fan), sec
	eeprom_update_byte(&EEPROM.FanCookerStartupMaxSpTime, 3); // sec forced max speed
	eeprom_update_byte(&EEPROM.FanSleep, 20);			// *30 = 10 h, Auto sleep time, *FANS_SLEEP_STEP
	eeprom_update_byte(&EEPROM.FanCookerSleep, 6);		// *30 = 3  h, Auto sleep time, *FANS_SLEEP_STEP
	eeprom_update_byte(&EEPROM.SpeedKeyIdx, 6);			// FanCooker3
	eeprom_update_byte(&EEPROM.FanSpeeds, 3);
	eeprom_update_byte(&EEPROM.FanCookerSpeeds, 3);
	eeprom_update_byte(&EEPROM.OutSpeedMax, 20);
	eeprom_update_byte(&EEPROM.FanSpeed_1, 13);			// 1..OutPeriod, OutPeriod+1..OutPeriod*2 - second fan coil
	eeprom_update_byte(&EEPROM.FanSpeed_2, 15);
	eeprom_update_byte(&EEPROM.FanSpeed_3, 20);
	eeprom_update_byte(&EEPROM.FanCookerSpeed_1, 14);
	eeprom_update_byte(&EEPROM.FanCookerSpeed_2, 20);
	eeprom_update_byte(&EEPROM.FanCookerSpeed_3, 20 * 2);
	eeprom_update_word(&EEPROM.SSR_PulseWidth, 9990);	// us
	eeprom_update_word(&EEPROM.SSR_PulseSafeTime, 100);	// us
	eeprom_update_byte(&EEPROM.PauseBetweenSetByCO2, 5);	// min
	eeprom_update_word(&EEPROM.PayloadPulseWidth, 0);	// us (rounded *10)
	eeprom_update_byte(&EEPROM.DamperOpenTime, 60);		// sec
	eeprom_update_byte(&EEPROM.FanCookerSpIdxDownKey, 5);
	eeprom_update_byte(&EEPROM.FanCookerSpIdxUpKey, 6);
	for(uint8_t i = 0; i < IR_MAX_CONTROLS * IR_Keys_Total; i++) eeprom_update_word(&EEPROM.IRCommandArray[i], 0);
#ifdef DEBUG_PROTEUS
 	eeprom_update_byte(&EEPROM.IRRemotes, 0);
	eeprom_update_word(&EEPROM.IRCommandArray[0], 0xA5E7);
	eeprom_update_word(&EEPROM.IRCommandArray[1], 0x3C24); 
	eeprom_update_word(&EEPROM.IRCommandArray[2], 0x5927); 
	eeprom_update_word(&EEPROM.IRCommandArray[3], 0x58E4); 
	eeprom_update_word(&EEPROM.IRCommandArray[4], 0x2227); 
	eeprom_update_word(&EEPROM.IRCommandArray[5], 0x47A4); 
	eeprom_update_word(&EEPROM.IRCommandArray[6], 0xC0E7); 
#endif	
}

void GetSettings(void)
{
//	uint8_t b = eeprom_read_byte(&EEPROM._OSCCAL);
//	if(b != 0xFF) OSCCAL = b;
	Flags = eeprom_read_byte(&EEPROM.Flags);
	//zero_cross_delay = ZERO_CROSS_TIMER_10ms - (uint32_t)ZERO_CROSS_TIMER_10ms * eeprom_read_word(&EEPROM.ZeroCrossLag) / 10000;
	//SSR_pulse_width = (uint32_t)ZERO_CROSS_TIMER_10ms * eeprom_read_word(&EEPROM.SSR_PulseWidth) / 10000;
	OutSpeedMax = eeprom_read_byte(&EEPROM.OutSpeedMax);
	PayloadPulseWidth = eeprom_read_word(&EEPROM.PayloadPulseWidth) / 10;
}

void UpdateZeroCrossing(uint8_t __FanPortADDR, uint8_t __FanPortPIN)
{
	uint16_t __after_zero_cross_pulse_delay = (uint32_t)SSR_full_period * (OutSpeedMax - FanSpeed) / OutSpeedMax;
	uint16_t __SSR_pulse_width = (uint32_t)SSR_full_period * eeprom_read_word(&EEPROM.SSR_PulseWidth) / 10000;
	uint16_t safety = (uint32_t)SSR_full_period * eeprom_read_word(&EEPROM.SSR_PulseSafeTime) / 10000;
	uint16_t sub = SSR_full_period - safety - __after_zero_cross_pulse_delay;
	if(__SSR_pulse_width >= sub) __SSR_pulse_width = sub;
	ATOMIC_BLOCK(ATOMIC_FORCEON) {
		after_zero_cross_pulse_delay = __after_zero_cross_pulse_delay;
		SSR_pulse_width = __SSR_pulse_width;
		if(__FanPortADDR) {
			*(uint8_t*)(uint16_t)FanPortADDR &= ~FanPortPIN;	// Off
			FanPortADDR = __FanPortADDR;
			FanPortPIN = __FanPortPIN;
			OCR1A = OCR1B = 0xFFFF;
		}
	}
}

// Set speed of FanCooker/Fan, speed index: 0(off), 1..FAN_IDXS*2
void SetFanSpeed(uint8_t fidx)
{
	if(fidx > FAN_IDXS * 2) return;
	if(FanOnNext && (fidx == 0 || FanOnNext == fidx)) FanOnNext = 0;
	if(fidx != FanOn) {
		if(FanOn == 0) { // From OFF mode
xFanOn:			
			FanOnNext = fidx;
			if(fidx > FAN_IDXS) { // FanCooker
				if(FanOnNext > FAN_IDXS + eeprom_read_byte(&EEPROM.FanCookerSpeeds)) FanOnNext = FAN_IDXS + eeprom_read_byte(&EEPROM.FanCookerSpeeds);
				if(FanOnNextCnt) goto xEnd;
				if(FanOnLast && FanOnLast < FAN_IDXS) { // switch from Fan and shutdown is in process
					fidx = 0;
				} else {
					fidx = FAN_IDXS + eeprom_read_byte(&EEPROM.FanCookerSpeeds);
					FanOnNextCnt = eeprom_read_byte(&EEPROM.FanCookerStartupMaxSpTime);
				}
			} else { // Fan
				if(FanOnNext > eeprom_read_byte(&EEPROM.FanSpeeds)) FanOnNext = eeprom_read_byte(&EEPROM.FanSpeeds);
				if(FanOnNextCnt) goto xEnd;
				if(FanOnLast > FAN_IDXS) { // switch from FanCooker and shutdown is in process
					fidx = 0;
				} else {
					if((Flags & f_FanDamper) && !Fan_Out2_status) {
						Fan_Out2_ON;
						fidx = 0;
						FanOnNextCnt = eeprom_read_byte(&EEPROM.DamperOpenTime);
					} else { // set temporarily max speed if EEPROM.FanStartupMaxSpTime > 0
						fidx = eeprom_read_byte(&EEPROM.FanSpeeds);
						FanOnNextCnt = eeprom_read_byte(&EEPROM.FanStartupMaxSpTime);
					}
				}
			}
 			if(FanOnNextCnt == 0) {
 				fidx = FanOnNext;
 				FanOnNext = 0;
 			}
		} else if(fidx == 0) { // -> off
			FanOnNext = 0;
		} else if(fidx > FAN_IDXS && FanOn <= FAN_IDXS) { // switch from Fan to FanCooker
			if(Flags & f_FanDamper) Fan_Out2_OFF;
			FanOnNextCnt = eeprom_read_byte(&EEPROM.FanShutdownTime);
			if(FanOnNextCnt) {
				FanOnLast = FanOn;
				FanOnNext = fidx;
				fidx = 0;
			} else goto xFanOn;
		} else if(fidx <= FAN_IDXS && FanOn > FAN_IDXS) { // switch from FanCooker to Fan
			FanOnNextCnt = eeprom_read_byte(&EEPROM.FanCookerShutdownTime);
			if(FanOnNextCnt) {
				FanOnLast = FanOn;
				FanOnNext = fidx;
				fidx = 0;
			} else goto xFanOn;
		} else { // change speed
			if(FanOnNext) {
				FanOnNext = fidx;
				LED_Warning = fidx;
				goto xEnd;
			}
		}
		if(FanOnNext == fidx) {
			FanOnNextCnt = 0;
			FanOnNext = 0;
		}
		FanOn = 0; // disable function in ISR
		if(!CookerLight_force_on && !SetFanSpeed_by_CO2) {
			if(Flags & f_CookerLamp_FanOut2) Fan_Out2_OFF;
		}
		if(fidx) {
			uint8_t speed = eeprom_read_byte(FANS_IDX_EEPROM_OFFSET - 1 + fidx);
			uint8_t	 *__FanPortADDR;
			uint8_t  __FanPortPIN;
			if(fidx > FAN_IDXS) { // FanCooker
				__FanPortADDR = (uint8_t*)&FanCooker_PORT;
				__FanPortPIN = FanCooker_Out1;
				if(speed > OutSpeedMax) {
					if(Flags & f_FanCookerHiSpeed) __FanPortPIN = FanCooker_Out2;
					speed -= OutSpeedMax;
				}
				if(Flags & f_CookerLamp_FanOut2) Fan_Out2_ON;
//				if(Flags & f_FanCookerDamper) FanCooker_Out2_ON;
			} else { // Fan
				__FanPortADDR = (uint8_t*)&Fan_PORT;
				__FanPortPIN = Fan_Out1;
				if(speed > OutSpeedMax) {
					if(Flags & f_FanHiSpeed) __FanPortPIN = Fan_Out2;
					speed -= OutSpeedMax;
				}
				if(Flags & (f_FanDamper /*| f_Damper*/)) Fan_Out2_ON;
			}
			LED_Warning = fidx;
			FanSpeed = speed;
			FanOn = fidx;
			UpdateZeroCrossing((uint16_t)__FanPortADDR, __FanPortPIN);
		}
	}
xEnd:
	if(FanOn == 0) {
		if(FanOnNext == 0) {
			if(Flags & (f_FanDamper /*| f_Damper*/)) Fan_Out2_OFF;
//			if(Flags & f_FanCookerDamper) FanCooker_Out2_OFF;
			FanOnNextCnt = 0;
			SleepTimer = 0;
		}
		FanSpeed = 0;
	} else if(!SetFanSpeed_by_CO2) {
		if(fidx == 0) fidx = FanOnNext;
		SleepTimer = eeprom_read_byte(fidx > FAN_IDXS ? &EEPROM.FanCookerSleep : &EEPROM.FanSleep);
	}
}

void FanSpeedUp(uint8_t _FanOn)
{
	if(_FanOn < eeprom_read_byte(&EEPROM.FanSpeeds) || (_FanOn > FAN_IDXS && _FanOn - FAN_IDXS < eeprom_read_byte(&EEPROM.FanCookerSpeeds))) SetFanSpeed(_FanOn + 1);
}

void FanSpeedDown(uint8_t _FanOn)
{
	if(_FanOn == FAN_IDXS + 1) SetFanSpeed(0); else SetFanSpeed(_FanOn - 1);
}

int main(void)
{
	CLKPR = (1<<CLKPCE); CLKPR = (0<<CLKPS3) | (0<<CLKPS2) | (0<<CLKPS1) | (0<<CLKPS0); // Clock prescaler division factor: 1
	MCUCR = (1<<SE) | (0<<SM1) | (0<<SM0); // Idle sleep enable
	NRF24_DDR |= NRF24_CE | NRF24_CSN | NRF24_SCK | NRF24_MOSI; // Out
	LED1_INIT;
	Fans_SETUP;
	KEY_SETUP;
	// Timer 8 bit
	TCCR0A = (1<<WGM01) | (1<<WGM00);  // Timer0: PWM, Fast PWM
	TCCR0B = (1<<WGM02) | (1 << CS02) | (0 << CS01) | (1 << CS00); // Timer0 prescaller: 1024
	OCR0A = 155; // =Fclk/prescaller/Freq - 1
	//OCR0B = 0; // Half Duty cycle ((TOP+1)/2-1)
	TIMSK0 = (1<<TOIE0); // Timer/Counter Overflow Interrupt Enable
	// Timer 16 bit
	TCCR1A = (0<<WGM11) | (0<<WGM10);  // Timer1: CTC, top ICR1
	TCCR1B = (1<<WGM13) | (1<<WGM12) | (0<<CS12) | (1<<CS11) | (0<<CS10); // Timer1: /8 = 1000000
	ICR1 = ZERO_CROSS_TIMER_TOP; // 20Hz, OC0A(TOP)=Fclk/prescaller/Freq - 1; Freq=Fclk/(prescaller*(1+TOP))
	OCR1A = 0xFFFF;
	OCR1B = ZERO_CROSS_TIMER_TOP; //0xFFFF;
	TIMSK1 = (1<<OCIE1B) | (1<<OCIE1A) | (1<<ICIE1); // Timer/Counter Interrupt Enable
	// ADC
// 	ADMUX = (0<<REFS1) | (1<<MUX2)|(1<<MUX1)|(1<<MUX0); // ADC7 (PA7)
// 	ADCSRA = (1<<ADEN) | (0<<ADATE) | (1<<ADIE) | (1<<ADPS2) | (1<<ADPS1) | (1<<ADPS0); // ADC enable, Free Running mode, Interrupt, ADC 128 divider
// 	ADCSRB = (1<<ADLAR) | (0<<ADTS2) | (0<<ADTS1) | (0<<ADTS0); // ADC Left Adjust Result
	// Pin change
	ZERO_CROSS_INIT;
	PayloadPortADDR = (uint16_t)&OCR0B;	// inited with not used IO!
	// Prepare for IR receiving
	MCUCR |= (0<<ISC01) | (1<<ISC00); // Any logical change on INT0
	GIMSK |= (1<<INT0); // External Interrupt Request 0 Enable
	GIFR |= (1<<INTF0); // Clear INT flag
	SETUP_WATCHDOG;
	uint8_t si = eeprom_read_byte(&EEPROM.SpeedInitIdx);
	if(si > FAN_IDXS * 2) {
		ResetSettings();
		si = 0;
	}
	GetSettings();
	sei();
	FlashLED(1, 0, 10);
	SetFanSpeed(si);
	if(Flags & f_NRF24) {
		NRF24_init(eeprom_read_byte(&EEPROM.RF_Channel)); // After init transmission must be delayed
		while(!NRF24_SetAddresses(eeprom_read_byte(&EEPROM.RF_Address))) {
			FlashLED(5,1,1);
#ifdef DEBUG_PROTEUS
			break;
#endif
		}
		NRF24_SetMode(NRF24_ReceiveMode);
	}
	while(1)
	{
		__asm__ volatile ("" ::: "memory"); // Need memory barrier
		sleep_cpu();
		wdt_reset();
		if(Timer1sec) {
			Timer1sec = 0;
			if(++TimerCntFanSleepStep >= FANS_SLEEP_STEP * 60) {
				TimerCntFanSleepStep = 0;
				if(SleepTimer) if(--SleepTimer == 0 && FanOn) SetFanSpeed(0);
			}
			if(FanOnNextCnt) if(--FanOnNextCnt == 0) {
				if(FanOnNext) SetFanSpeed(FanOnNext); else FanOnLast = 0;
			}
			if(FanOnNext) Set_LED_Warning(WRN_FAN_SWITCHING);
			if(Timer) {
				if(--Timer == 0) {
 					if(Setup) {
xSetup_finish:						 
						 Setup = fSetup_Off;
						 SetupItem = 0;
						 FlashLED(5, 3, 3);
					}
				}
				if(Setup) Set_LED_Warning(SetupItem + 1);
			}
			if(IRRepeatDelay == 0) { // Setup pressed n times
				uint8_t n = IRRepeatCnt;
				IRRepeatCnt = 0;
				if(n) {
					if(Setup == fSetup_Speed) {
						eeprom_update_byte(FANS_IDX_EEPROM_OFFSET - 1 + FanOn, FanSpeed);
						goto xSetup_finish;
					} else if(n >= REPEAT_TIMES_SETUP_IR) {
						goto xStartSetupIR;
					} else if(n >= REPEAT_TIMES_SETUP_SPEED) {
						if(FanOn) {
							FlashLED(1, 5, 15);
							Setup = fSetup_Speed;
							LED_Warning = FanSpeed;
							Timer = 255;
						}
					}
				}
			}
		}
		if(IR_Status == IR_DONE) {
			IR_Status = IR_WAITING;
			IR_Hash_Last = IR_Hash;
			IR_CntLast = IR_Cnt;
			uint8_t remotes_max = eeprom_read_byte(&EEPROM.IRRemotes) * IR_Keys_Total;
			if(Setup == fSetup_IR) {
				if(IRRepeatDelay) continue;
				remotes_max += SetupItem;
			}
			LED_WarningPause = 2;
			LED1_ON;
			uint8_t i = 0;
			for(; i < remotes_max; i++) {
				uint16_t _hash = eeprom_read_word(&EEPROM.IRCommandArray[i]);
				if(_hash == 0) continue;
				if(_hash == IR_Hash) {
					uint8_t key = i % IR_Keys_Total;
					if(Setup == fSetup_IR) { // already exist!
						if(key == IR_Key_Setup) { // skip if pressed "Setup" on other or the same remote
							IR_Hash = 0;
							goto xSetupIR_New;
						}
						FlashLED(5, 1, 1);
						break;
					}
					if(key == IR_Key_Setup) {
						IRRepeatCnt++;
						IRRepeatDelay = IR_REPEAT_TIMEOUT_SETUP;
					} else if(IRRepeatDelay == 0 || (IRRepeatCnt && IRRepeatDelay < IR_REPEAT_TIMEOUT_SETUP - IR_REPEAT_TIMEOUT)) {
						IRRepeatDelay = IR_REPEAT_TIMEOUT;
						if(key == IR_Key_Off) {
							if(Setup == fSetup_Speed) {
								goto xSetup_finish;
							} else {
								CookerLight_force_on = 0;
								SetFanSpeed(0); // Off
							}
						} else {
							uint8_t _FanOn = FanOn ? FanOn : FanOnNext;
							if(key == IR_Key_CookerUp) { // FanCooker
								SetFanSpeed_by_CO2 = 0;
								if(Setup == fSetup_Speed) goto xFanSpeedInc;
								else if(_FanOn > FAN_IDXS) FanSpeedUp(_FanOn); else SetFanSpeed(eeprom_read_byte(&EEPROM.FanCookerSpIdxUpKey));
							} else if(key == IR_Key_CookerDown) { // FanCooker
								SetFanSpeed_by_CO2 = 0;
								if(Setup == fSetup_Speed) goto xFanSpeedDec;
 								else if(_FanOn > FAN_IDXS) FanSpeedDown(_FanOn); else SetFanSpeed(eeprom_read_byte(&EEPROM.FanCookerSpIdxDownKey));
							} else if(key == IR_Key_Cooker_Speed_1) { // FanCooker
								SetFanSpeed_by_CO2 = 0;
								SetFanSpeed(GET_FANS_IDX_EEPROM(&EEPROM.FanCookerSpeed_1));
							} else if(key == IR_Key_Cooker_Speed_2) { // FanCooker
								SetFanSpeed_by_CO2 = 0;
								SetFanSpeed(GET_FANS_IDX_EEPROM(&EEPROM.FanCookerSpeed_2));
							} else if(key == IR_Key_Cooker_Speed_3) { // FanCooker
								SetFanSpeed_by_CO2 = 0;
								SetFanSpeed(GET_FANS_IDX_EEPROM(&EEPROM.FanCookerSpeed_3));
							} else if(key == IR_Key_FanUp) { // Fan
								if(Setup == fSetup_Speed) goto xFanSpeedInc;
								else if(_FanOn && _FanOn <= FAN_IDXS) FanSpeedUp(_FanOn); else SetFanSpeed(GET_FANS_IDX_EEPROM(&EEPROM.FanSpeed_3));
							} else if(key == IR_Key_FanDown) { // Fan
								if(Setup == fSetup_Speed) goto xFanSpeedDec;
								else if(_FanOn && _FanOn <= FAN_IDXS) FanSpeedDown(_FanOn); else SetFanSpeed(GET_FANS_IDX_EEPROM(&EEPROM.FanSpeed_1));
							} else if(key == IR_Key_CookerLight) {
								CookerLight_force_on ^= 1;
								goto xLampForce;
							}
						}
					}
					break;
				}
			}
			if(Setup == fSetup_IR && i == remotes_max) { // New remote command
				IRRepeatDelay = 10;
xSetupIR_New:				
				i = eeprom_read_byte(&EEPROM.IRRemotes);
				eeprom_update_word(&EEPROM.IRCommandArray[i * IR_Keys_Total + SetupItem], IR_Hash);
				Set_LED_Warning_New(0);
				if(SetupItem < IR_Keys_Total - 1) {
					SetupItem++;
					Timer = 60;
				} else { // Finish, next remote
					eeprom_update_byte(&EEPROM.IRRemotes, ++i);
					if(i >= IR_MAX_CONTROLS) { // not enough space
						goto xSetup_finish;
					} else goto xStartSetupIR;
				}
			}
		}
		if(Key1Pressed > KEY_PressingTimeMin) {
			if(Key1Pressed >= 200) { // > 4 sec
				LED1_ON;
				Key1Pressed = 1;
				while(KEY_PRESSING) {
					__asm__ volatile ("" ::: "memory"); // Need memory barrier
					wdt_reset();
				}
				if(Key1Pressed == 255) { // reset settings - pressed > 9 sec
					ResetSettings();
					goto xReset;
				}
xStartSetupIR:
				// Setup IR Commands
				FlashLED(3, 1, 1);
				Delay100ms(15);
				Key1Pressed = 0;
				uint8_t i = eeprom_read_byte(&EEPROM.IRRemotes);
				if(i >= IR_MAX_CONTROLS) {
					FlashLED(50, 1, 1);
					if(Key1Pressed) eeprom_update_byte(&EEPROM.IRRemotes, 0); else continue;
				}
				LED_Warning = (i + 1) << 4;
				IR_Status = IR_WAITING;
				Setup = fSetup_IR;
				SetupItem = 0;
				Timer = 60;
			} else if(!KEY_PRESSING) { // Manual speed - pressed < 2 sec
				uint8_t i = eeprom_read_byte(&EEPROM.SpeedKeyIdx);
				SetFanSpeed(/* FanOn == i ? 0 : */ i);
				Key1Pause = 10;	// 1 sec
				Key1Pressed = 0;
			}
		}
		if(Flags & f_NRF24) {
#ifndef DEBUG_PROTEUS
			if(NRF24_Receive((uint8_t*)&data)) {
				LED_WarningPause = 2;
				LED1_ON;
#else
			if((PINA & (1<<PORTA6)) && !LED_WarningPause) {
				data.CO2level = 0; //0x1122;
				data.FanSpeed = 4; //0x8F; //4;
				data.Flags = 0xC0; //0x92 << (PINA & (1<<PORTA4)); //0xC0;
				//WriteTimeout = 1;
				//SSR_pulse_width = (uint32_t)ZERO_CROSS_TIMER_10ms * 5000 / 10000;;
				LED_WarningPause = 5;
				LED1_ON;
#endif
				struct SETUP_DATA *p = (struct SETUP_DATA*)&data;
				register uint8_t cmd = p->Flags;
				if(cmd == fCMD_Set) { // SET command
					int8_t d = p->Data;
					register uint8_t type = p->Type;
					if(type == Type_Set_RESET) {
						if(p->Data != 0xEEEE) continue;
xReset:
						LED1_ON_BEFORE_RESET;
						cli(); while(1) ; // restart
					} else if(type == Type_Set_Fan) { // Set Fan Index
						if(d <= FAN_IDXS * 2) {
							SetFanSpeed_by_CO2 = 0;
							SetFanSpeed(d);
							if(d == 0) FlashLED(3, 2, 2);
						}
					} else if(type == Type_Set_FanAdd) { // Fan Index + n
						uint8_t _FanOn = FanOn ? FanOn : FanOnNext;
						if(d == 1) FanSpeedUp(_FanOn); else FanSpeedDown(_FanOn);
					} else if(type == Type_Set_FanSpeedUp) { // FanSpeed +1
xFanSpeedInc:
						if(FanSpeed < OutSpeedMax) {
							FanSpeed++;
							UpdateZeroCrossing(0, 0);
							LED_Warning = FanSpeed;
						}
						Timer = 255;
					} else if(type == Type_Set_FanSpeedDown) { // FanSpeed -1
xFanSpeedDec:
						if(FanSpeed > 1) {
							FanSpeed--;
							UpdateZeroCrossing(0, 0);
							LED_Warning = FanSpeed;
						}
						Timer = 255;
 					} else if(type == Type_Set_FanSpeedSave) { // Save to current fan idx
 						if(FanOn) {
 							eeprom_update_byte(FANS_IDX_EEPROM_OFFSET - 1 + FanOn, FanSpeed);
 						}
 						Timer = 255;
					} else if(type == Type_Set_Lamp) { // Switch Lamp
						CookerLight_force_on = d;
xLampForce:
						if(CookerLight_force_on) {
							if(Flags & f_CookerLamp_FanOut2) Fan_Out2_ON;
						} else {
							if(Flags & f_CookerLamp_FanOut2) Fan_Out2_OFF;
						}
					} else if(type == Type_Set_SetupIR) { // Enter Setup IR mode
						goto xStartSetupIR;
					}
				} else if(cmd == fCMD_WriteStart) {
					WriteTimeout = fCMD_Write_Timeout;
				} else if(cmd & fCMD_Read) { // READ command
					register uint8_t cmd2 = cmd & 0xF0;
					cmd &= 0x0F;
					if(cmd2 == fCMD_Read + fCMD_EEPROM) {
						if(p->Data < sizeof(struct _EEPROM)) {
							if(cmd == fCMD_2b) p->Data = eeprom_read_word((uint16_t*)((uint8_t*)&EEPROM + p->Data));
							else p->Data = eeprom_read_byte((uint8_t*)&EEPROM + p->Data);
							//p->Type = p->Flags = 0;
						} else continue;
					} else if(cmd2 == fCMD_Read + fCMD_RAM) {
						ATOMIC_BLOCK(ATOMIC_FORCEON) {
							if(cmd == fCMD_2b) p->Data = *((uint16_t *)p->Data);
							else p->Data = *((uint8_t *)p->Data);
						}
						//p->Type = p->Flags = 0;
					} else if(cmd2 == fCMD_Read + fCMD_PROGMEM) {
						if(cmd == fCMD_2b) p->Data = pgm_read_word(p->Data);
						else if(cmd == fCMD_1b) p->Data = pgm_read_byte(p->Data);
						//p->Type = p->Flags = 0;
					} else continue;
					Delay100ms(1);
					NRF24_SetMode(NRF24_TransmitMode);
					uint8_t err = 0;
					if(cmd == fCMD_CStr) {
						for(uint8_t i = 0; i < sizeof(ProgramID); i++) {
							p->Data = pgm_read_byte(&ProgramID[i]);
							err = NRF24_Transmit((uint8_t *)&data);
							if(err) break;
							Delay10us(255);
						}
					} else err = NRF24_Transmit((uint8_t *)&data);
					NRF24_SetMode(NRF24_ReceiveMode);
					if(err) Set_LED_Warning_New(WRN_SETUP_ERR + err);
					//Timer = 120; // sec
				} else if(cmd & fCMD_Write) { // WRITE command
					if(WriteTimeout) {
						register uint8_t cmd2 = cmd & 0x30;
						cmd &= 0x0F;
						if(cmd2 == fCMD_RAM) {
							ATOMIC_BLOCK(ATOMIC_FORCEON) {
								if(cmd == fCMD_2b) *((uint16_t *)(uint16_t)p->Type) = p->Data;
								else *((uint8_t *)(uint16_t)p->Type) = p->Data;
							}
						} else if(cmd2 == fCMD_EEPROM) {
							if(cmd == fCMD_2b) {
								eeprom_update_word((uint16_t*)((uint8_t*)&EEPROM + p->Type), p->Data);
							} else {
								eeprom_update_byte((uint8_t*)&EEPROM + p->Type, p->Data);
							}
							GetSettings();
							Set_LED_Warning_New(WRN_SETUP_INFO);
						}
						WriteTimeout = fCMD_Write_Timeout;
					}
					//Timer = 120; // sec
				} else { // CO2 received
					if(SleepTimer == 0 && Setup == fSetup_Off) {
						uint8_t fs;
						if(data.CO2level > eeprom_read_word(&EEPROM.CO2_threshold3)) fs = 3;
						else if(data.CO2level > eeprom_read_word(&EEPROM.CO2_threshold2)) fs = 2;
						else if(data.CO2level > eeprom_read_word(&EEPROM.CO2_threshold1)) fs = 1;
						else fs = 0;
						if(fs != FanOn && (SetFanSpeed_by_CO2 || (TimerMin == 0 && FanOn == 0 && FanOnNext == 0))) {
							SetFanSpeed_by_CO2 = 1;
							SetFanSpeed(fs);
							if(!fs) SetFanSpeed_by_CO2 = 0;
							TimerMin = eeprom_read_byte(&EEPROM.PauseBetweenSetByCO2);
						}
					}
				}
			}
		}
	}
}

// Check connected SSR relay
/*	FanCooker_PORT |= FanCooker_Speed1;
	Delay10us(100);
	FanCookerPresent = (FanCooker_PIN & FanCooker_Speed1) != 0;
	FanCooker_PORT &= ~FanCooker_Speed1;
	FanCooker_PORT |= FanCooker_Speed2;
	Delay10us(100);
	FanCookerPresent |= ((FanCooker_PIN & FanCooker_Speed2) != 0) << 1;
	FanCooker_PORT &= ~FanCooker_Speed2;
	Fan_PORT |= Fan_Speed1;
	Delay10us(100);
	FanPresent = (Fan_PIN & Fan_Speed1) != 0;
	Fan_PORT &= ~Fan_Speed1;
	Fan_PORT |= Fan_Speed2;
	Delay10us(100);
	FanPresent |= ((Fan_PIN & Fan_Speed2) != 0) << 1;
	Fan_PORT &= ~Fan_Speed2;
	FlashLED((FanPresent == 3 ? 2 : FanPresent ? 1 : 0) + (FanCookerPresent == 3 ? 2 : FanCookerPresent ? 1 : 0), 3, 3);
*/
