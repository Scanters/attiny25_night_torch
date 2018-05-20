#define F_CPU	1000000UL  // 1MHz (8/8 by FUSES)

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

/************************************************************************/
/* Function prototypes and global variables                             */
/************************************************************************/
// FUSES:
// PINS:	RESET disabled, SPI disabled, SELFPROG disabled
// CLOCK:	8MHz/8 from internal RC oscillator, CKOUT off, WDT off
// EEPROM:	EESAVE off
// LOCKBT:	Lock1 and Lock2 off
#define PIN_ERROR_LED		(1<<PB0)
#define PIN_OUT_RELAY		(1<<PB1)

#define PIN_IN_LIGHT		(1<<PB2)	// pull down by resistor or #inverse?	INT0?
#define PIN_IN_MOV1			(1<<PB3)	// pull down by resistor				PCINT
#define PIN_IN_MOV2			(1<<PB4)	// pull down by resistor				PCINT
#define PIN_IN_MOV3			(1<<PB5)	// pull down by resistor				PCINT

// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
#define LIGHTS_TIMEOUT		(10)		// Turn off lights after X inactive seconds
// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

#define TIMER_1MHZ_COMP		(122)		// TODO: Measure the value for 1 sec delay.
#define TIMER_1HZ_PRESC		(8)			// (1_000_000 / 1024 / 122 / _8_) == 1Hz == 0.1 sec
#define	TIMER_10SEC_COMP	(TIMER_1HZ_PRESC * LIGHTS_TIMEOUT)	// (1 / _10_) == 0.1Hz == 10 sec // 8 * 8 = 64

struct state
{
	union
	{
		uint8_t any;
		struct
		{
			uint8_t move_sensor_1	: 1;
			uint8_t move_sensor_2	: 1;
			uint8_t move_sensor_3	: 1;
			uint8_t light_sensor	: 1;
			uint8_t	/*padding*/		: 4;
		};
	} input;

	union
	{
		uint8_t all;
		struct
		{
			uint8_t relay			: 1;
			uint8_t	/*padding*/		: 7;
		};
	} output;

	union
	{
		uint8_t all;
		struct
		{
			uint8_t state			: 1;
			uint8_t	timeout			: 7;
		};
	} timer;
};

void initialize(volatile state& state_ptr);

inline void relay_on();
inline void relay_off();

inline void timer_stop();
inline void timer_start();

inline uint8_t is_relay_on();
inline uint8_t is_daylight_detected();
inline uint8_t is_movement_detected();

static volatile state g_state { 0 };

/************************************************************************/
/* Main logic                                                           */
/************************************************************************/
int main(void)
{
	initialize(g_state);
	
	timer_start();
	
    while (1) 
    {
		if (is_daylight_detected())
		{
			relay_off();
			timer_stop();
		}
	}
}

void initialize(volatile state& state_ref)
{
	cli();
	
	// Setup ports mode
	DDRB |= (PIN_OUT_RELAY); // | PIN_ERROR_LED);

	state_ref.output.all = 0;
	
	// Setup timers
	OSCCAL = 0x8A;
	
	OCR0A = TIMER_1MHZ_COMP;  // TODO: Measure the value for 1 sec delay.
	TIMSK |= (1<<OCIE0A);  // Enable interrupts on COMPA
	TCCR0A |= (1<<WGM01);  // Set timer0 CTC mode
	
	// Setup interrupts
	MCUCR |= (1<<ISC00);  // Any logical change for EXTI
	GIMSK |= ((1<<INT0) | (1<<PCIE));  // Enable EXTI and PCINT
	
	PCMSK |= ((1<<PCINT3) | (1<<PCINT4) | (1<<PCINT5)); // PCINT for MOV pins only
	
	TCCR0B |= ((1<<CS00) | (1<<CS02));
	
	sei();
}

inline void timer_start()
{
	if (is_daylight_detected())
	{
		return timer_stop();
	}

	// Reset global timer0 stop flag, set timer state to True
	g_state.timer.all = 1;
	return;
}

inline void timer_stop()
{
	// Reset global timer0 stop flag
	g_state.timer.all = 0;
	return;
}

inline void relay_on()
{
	if (is_daylight_detected())
	{
		return relay_off();
	}
	
	PORTB |= PIN_OUT_RELAY;
	
	return timer_start();
}

inline void relay_off()
{
	cli();
	
	timer_stop();
	PORTB &= ~(PIN_OUT_RELAY);
	
	sei();
}

inline uint8_t is_relay_on()
{
	return (PINB & PIN_OUT_RELAY);
}

inline uint8_t is_movement_detected()
{
	return PINB & (PIN_IN_MOV1 | PIN_IN_MOV2 | PIN_IN_MOV3);
}

inline uint8_t is_daylight_detected()
{
	// #inverse signal
	return !(PINB & PIN_IN_LIGHT);
}

/************************************************************************/
/* Interrupts                                                           */
/************************************************************************/
ISR(TIMER0_COMPA_vect)
{
	if (g_state.timer.state == 0)
	{
		return relay_off();
	}

	// Hit OVF ISR every 1/8 seconds (80Hz) at 1024 prescaler
	if (g_state.timer.timeout++ > TIMER_10SEC_COMP)
	{
		return relay_off();
	}
}

ISR(PCINT0_vect)
{
	if (is_movement_detected())
	{
		return relay_on();
	}
}

// EXTI from light density sensor.
// Sets relay control policy for the main loop when MVT sensors activate
ISR(INT0_vect)
{
	if (is_movement_detected())
	{
		relay_on();
	}
	else
	{
		relay_off();
	}
}

// Tabs of 4
