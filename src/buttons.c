#include "buttons.h"

#include <stdint.h>

#include <avr/interrupt.h>


//////////////////////////////////////////////////////////////////////////
// KEY STATUS GETTERS
//////////////////////////////////////////////////////////////////////////

// check if a key has been pressed. Each pressed key is reported only once.
//

uint8_t get_key_press(uint8_t key_mask)
{
	cli();			// read and clear atomic !
	key_mask &= key_press;	// read key(s)
	key_press ^= key_mask;	// clear key(s)
	sei();
	return key_mask;
}

///////////////////////////////////////////////////////////////////
//
// check if a key has been pressed long enough such that the
// key repeat functionality kicks in. After a small setup delay
// the key is reported being pressed in subsequent calls
// to this function. This simulates the user repeatedly
// pressing and releasing the key.
//
uint8_t get_key_rpt(uint8_t key_mask)
{
	cli();					// do not disturb, keep read and clear together
	key_mask &= key_rpt;	// read key(s)
	key_rpt ^= key_mask;	// clear key(s)
	sei();
	return key_mask;
}

///////////////////////////////////////////////////////////////////
//
// check if a key is pressed right now
//
uint8_t get_key_state(uint8_t key_mask)
{
	key_mask &= key_state;
	return key_mask;
}

///////////////////////////////////////////////////////////////////
//
uint8_t get_key_short(uint8_t key_mask)
{
	cli();			// read key state and key press atomic !
	return get_key_press(~key_state & key_mask);
}

///////////////////////////////////////////////////////////////////
//
uint8_t get_key_long(uint8_t key_mask)
{
	return get_key_press(get_key_rpt(key_mask));
}

uint8_t get_key_long_r(uint8_t key_mask)
{				// if repeat function needed
	return get_key_press(get_key_rpt(key_press & key_mask));
}

uint8_t get_key_rpt_l(uint8_t key_mask)
{				// if long function needed
	return get_key_rpt(~key_press & key_mask);
}

uint8_t get_key_common(uint8_t key_mask)
{
	return get_key_press((key_press & key_mask) == key_mask ? key_mask : 0);
}


uint8_t get_key_common_l(uint8_t key_mask)
{
	return get_key_state((key_press & key_mask) == key_mask ? key_mask : 0);
}
