#include "display.h"

#include <stdbool.h>
#include <stdint.h>

#include <avr/interrupt.h>
#include <util/delay.h>

// FRAMEBUFFERS
//-----------------------------------------------
// Contains the values to be displayed next

// fb is the "raw" data which is directly displayed by the timer-ISR
uint8_t fb[3] = { 0xFF, 0xFF, 0xFF };	// dig0, dig1, dig2

// framebuffer is a bit more structured, but needs and fb_update() to be stored in fb
// right digit, middle digit, left digit, right dot, middle dot, left dot, change indicator
framebuffer_t framebuffer = { {0x00, 0x00, 0x00}, {0, 0, 0}, 0 };


void display_update(){

    static uint8_t digit = 0;

	digit++;

	if (digit == 24) {
		digit = 0;
	}

	uint8_t bm;
	// explicit switch is faster than variable shifting
	// TODO: magic constant!
	switch (digit & 0x07) {
	case 0:
		bm = ~(1 << 0);
		break;
	case 1:
		bm = ~(1 << 1);
		break;
	case 2:
		bm = ~(1 << 2);
		break;
	case 3:
		bm = ~(1 << 3);
		break;
	case 4:
		bm = ~(1 << 4);
		break;
	case 5:
		bm = ~(1 << 5);
		break;
	case 6:
		bm = ~(1 << 6);
		break;
	case 7:
		bm = (uint8_t) ~ (1 << 7);
		break;
	}

	// all segments OFF (set HIGH, as current sinks)
	SEGS_OFF;

	switch (digit / 8) {
	case 0:
		DIG0_ON;	// turn on digit #0 (from right)
		PORTD = fb[0] | bm;
		DIG1_OFF;
		DIG2_OFF;
		break;
	case 1:
		DIG1_ON;	// #1
		PORTD = fb[1] | bm;
		DIG0_OFF;
		DIG2_OFF;
		break;
	case 2:
		DIG2_ON;	// #2
		PORTD = fb[2] | bm;
		DIG0_OFF;
		DIG1_OFF;
		break;
	default:
		DIG0_OFF;
		DIG1_OFF;
		DIG2_OFF;
		break;
	}

	if (OCR1B == 640) {
		OCR1B = 8;
	} else {
		OCR1B += 8;
	}

}

//////////////////////////////////////////////////////////////////////////
// DISPLAY FUNCTIONS
//////////////////////////////////////////////////////////////////////////

 /**
  * \brief display a number.
  *
  * This function displays a number on the 7-segment display.
  * Related functions:
  * display_char()
  * display_string()
  *
  * \param[in] number     number to display (negative numbers => underflow on the display)
  * \return    void
  */
void display_number(int16_t number)
{
	if (number < 0) {
		framebuffer.dot[0] = 1;
		framebuffer.dot[1] = 1;
		framebuffer.dot[2] = 1;
		number = -number;
	} else {
		// don't clear framebuffer[3], as this is the heater-indicator
		framebuffer.dot[1] = 0;
		framebuffer.dot[2] = 0;
	}

	framebuffer.digit[0] = (uint8_t) (number % 10);
	number /= 10;
	framebuffer.digit[1] = (uint8_t) (number % 10);
	number /= 10;
	framebuffer.digit[2] = (uint8_t) (number % 10);
	framebuffer.changed = 1;
	fb_update();
}


 /**
  * \brief display a character.
  *
  * This function displays a character at a given position on the 7-segment display.
  *
  * \param[in] digit     position (0-2)
  * \param[in] character character to display (e.g. 'A' or '1')
  * \param[in] dot       if > 0: activate the dot behind the given position
  * \return    void
  */
void display_char(uint8_t digit, uint8_t character, uint8_t dot)
{
	uint8_t portout = 0xFF;

	// 8 Bit
	// -> '0'-Bit: active
	// -> '1'-Bit: not active

	//  0b0000_0000
	//	  |||| |||'--- top
	//    |||| ||'---- bottom-left
	//    |||| |'----- bottom
	//    |||| '------ top-left
	//    |||'-------- dot (on the right side of the segment)
	//    ||'--------- bottom-right
	//    |'---------- middle
	//    '----------- top-right


	switch (character) {
	case 0:
		// ~0b?1010_1111? = 0b0101_0000
		portout = (uint8_t) (~0xAF);	// activate segments for displaying a '0'
		break;
	case 1:
		// ~0b?1010_0000? = 0b0101_1111
		portout = (uint8_t) (~0xA0);	// '1'
		break;
	case 2:
		// ~0b?1100_0111? = 0b0011_1000
		portout = (uint8_t) (~0xC7);	// '2'
		break;
	case 3:
		// ~0b?1110_0101? = 0b0001_1010
		portout = (uint8_t) (~0xE5);	// '3'
		break;
	case 4:
		portout = (uint8_t) (~0xE8);	// '4'
		break;
	case 5:
		portout = (uint8_t) (~0x6D);	// '5'
		break;
	case 6:
		portout = (uint8_t) (~0x6F);	// '6'
		break;
	case 7:
		// ~0b?10100001? = 0b01011110
		portout = (uint8_t) (~0xA1);	// '7'
		break;
	case 8:
		portout = (uint8_t) (~0xEF);	// '8'
		break;
	case 9:
		portout = (uint8_t) (~0xE9);	// '9'
		break;
	case '-':
		portout = (uint8_t) (~0x40);	// '-'
		break;
	case '.':
		// ~0b0001_0000 = 0b1110_1111
		portout = (uint8_t) (~0x10);	// '.'
		break;
	case 'A':
		portout = (uint8_t) (~0xEB);	// 'A'
		break;
	case 'B':
		portout = (uint8_t) (~0x6E);	// 'b'
		break;
	case 'C':
		portout = (uint8_t) (~0x0F);	// 'C'
		break;
	case 'D':
		portout = (uint8_t) (~0xE6);	// 'd'
		break;
	case 'E':
		portout = (uint8_t) (~0x4F);	// 'E'
		break;
	case 'F':
		portout = (uint8_t) (~0x4B);	// 'F'
		break;
	case 'G':
		portout = (uint8_t) (~0x6F);	// 'G'
		break;
	case 'H':
		portout = (uint8_t) (~0x6A);	// 'h'
		break;
	case 'I':
		portout = (uint8_t) (~0x20);	// 'i'
		break;
	case 'L':
		portout = (uint8_t) (~0x0E);	// 'L'
		break;
	case 'N':
		portout = (uint8_t) (~0xAB);	// 'N'
		break;
	case 'O':
		portout = (uint8_t) (~0x66);	// 'o'
		break;
	case 'P':
		portout = (uint8_t) (~0xCB);	// 'P'
		break;
	case 'R':
		portout = (uint8_t) (~0x42);	// 'r'
		break;
	case 'S':
		portout = (uint8_t) (~0x6D);	// 'S'
		break;
	case 'T':
		portout = (uint8_t) (~0x4E);	// 't'
		break;
	case 'U':
		portout = (uint8_t) (~0x26);	// 'u'
		break;
	case 'V':
		portout = (uint8_t) (~0x26);	// 'v'
		break;
	case '*':
		portout = (uint8_t) (~0xC9);	// '�'
		break;
	case ' ':
	case 255:
		portout = (uint8_t) (0xFF);		// segments OFF
		break;
	default:
		portout = (uint8_t) (~0x10);	// '.'
		break;
	}

	if (dot)
		portout &= (~0x10);	// '.'

	fb[digit] = portout;
}


 /**
  * \brief display a string.
  *
  * This function displays a string on the 7-segment display.
  * It reads only the first three characters.
  *
  * \param[in] *string     pointer to the string to display
  * \return    void
  */
void display_string(const char *string)
{
	framebuffer.digit[0] = 255;
	framebuffer.digit[1] = 255;
	framebuffer.digit[2] = 255;
	framebuffer.dot[0] = 0;
	framebuffer.dot[1] = 0;
	framebuffer.dot[2] = 0;

	uint8_t ctr;

	for (ctr = 0; ctr <= 2; ctr++) {
		// read the first 3 characters of the string
		if (string[ctr] == '\0') {
			break;
			} else {
			framebuffer.digit[2 - ctr] = string[ctr];
		}
	}
	framebuffer.changed = 1;
	fb_update();
}


 /**
  * \brief display a string.
  *
  * This function displays a running string on the 7-segment display.
  * It reads out and displays every character until a '\0' is reached.
  * CAUTION! Always be sure that your string is null-terminated
  * CAUTION! This functions is blocking...
  *
  * The standard delay is 500ms
  *
  * \param[in] *string     pointer to the c-string to display
  * \return    void
  */
void display_string_running(const char* string)
{
	// TODO: The text does not run at the moment, why?

	// empty string, return
	if(string[0] == '\0')
		return;

	// string isn't long enough for "running", hence display normally
	if((string[1] == '\0') | (string[2] == '\0') | (string[3] == '\0'))
		display_string(string);

	// if you get here, the string seems to be ok
	for(uint8_t i = 0; string[i+2] != '\0'; i++){
		display_string(&string[i]);
		_delay_ms(500);
	}

}

void clear_display(void)
{
	framebuffer.digit[0] = 255;
	framebuffer.digit[1] = 255;
	framebuffer.digit[2] = 255;
	framebuffer.dot[0] = 0;
	framebuffer.dot[1] = 0;
	framebuffer.dot[2] = 0;
	framebuffer.changed = 1;
	fb_update();
}

void set_dot(void)
{
	framebuffer.dot[0] = 1;
	framebuffer.changed = 1;
	fb_update();
}


void clear_dot(void)
{
	framebuffer.dot[0] = 0;
	framebuffer.changed = 1;
	fb_update();
}


void set_eeprom_saved_dot(void)
{
	framebuffer.dot[1] = 1;
	framebuffer.changed = 1;
	fb_update();
}


void clear_eeprom_saved_dot(void)
{
	framebuffer.dot[1] = 0;
	framebuffer.changed = 1;
	fb_update();
}

//////////////////////////////////////////////////////////////////////////
// FRAME BUFFER UPDATE
//////////////////////////////////////////////////////////////////////////
// framebuffer -> fb

void fb_update()
{
	if (!framebuffer.changed)
		return;

	uint8_t _sreg = SREG;	/* save SREG */
	cli();			/* disable all interrupts to avoid half-updated screens */

	for (uint8_t digit = 0; digit < 3; digit++) {
		display_char(digit, framebuffer.digit[digit], framebuffer.dot[digit]);
	}
	framebuffer.changed = 0;

	SREG = _sreg;
}
