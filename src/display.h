#ifndef INC_DISPLAY_H
#define INC_DISPLAY_H

#include "settings.h"

#define DIG0_OFF ( PORTB &= ~_BV(PB0) )
#define DIG1_OFF ( PORTB &= ~_BV(PB7) )
#define DIG2_OFF ( PORTB &= ~_BV(PB6) )

#define DIG0_ON ( PORTB |= _BV(PB0) )
#define DIG1_ON ( PORTB |= _BV(PB7) )
#define DIG2_ON ( PORTB |= _BV(PB6) )

#define SEGS_OFF ( PORTD = 0xFF )

typedef struct {
	char digit[3];
	bool dot[3];
	bool changed:1;
} framebuffer_t;

void display_update();
void display_number(int16_t number);
void display_char(uint8_t digit, uint8_t character, uint8_t dot);
void display_string(const char *string);
void display_string_running(const char* string);
void clear_display(void);
void clear_dot(void);
void clear_eeprom_saved_dot(void);
void fb_update(void);

#endif // INC_DISPLAY_H
