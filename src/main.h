#ifndef MAIN_H
#define MAIN_H

#define FW_MAJOR_V 2
#define FW_MINOR_V_A 0
#define FW_MINOR_V_B 1

#define low(x)   ((x) & 0xFF)
#define high(x)   (((x)>>8) & 0xFF)

#include "settings.h"

#include <stdbool.h>
#include <stdint.h>




/*
 * See the Docs folder for how to add a 1 Ohm current sense
 * resistor to meaure the fan-current.
 *
 * Some time in the future, this may be used to check for
 * true fan-speed via the commutation signal.
 *
 * This only requires an opamp (used as comparator) +
 * a few resistors / caps.
 *
 * http://dangerousprototypes.com/2014/03/01/app-note-fan-health-monitoring-and-the-mic502/
 * http://www.micrel.com/_PDF/App-Notes/an-34.pdf
 *
 */

//#define DEBUG
//#define CURRENT_SENSE_MOD

#define USE_WATCHDOG
//#define DISPLAY_MCUSR
//#define WATCHDOG_TEST

typedef struct CPARAM {
	char* text;
	int16_t value_min;
	int16_t value_max;
	int16_t value_default;
	int16_t value;
	uint8_t *p_eep_addr_high;
	uint8_t *p_eep_addr_low;
} CPARAM;

void show_parameter_config(CPARAM * param, const char *string);
void char_test(void);

void eep_load(CPARAM * param);
void eep_save(CPARAM * param);

void fan_test(void);

void restore_default_conf(void);
void segm_test(void);
void set_dot(void);
void set_eeprom_saved_dot(void);
void setup_858D(void);
void setup_timer1_ctc(void);
void show_firmware_version(void);
void show_config();


void adcInit (void);
uint16_t adcRead (uint8_t);

unsigned long millis();
void init();

#define FAN_OFF ( PORTC |= _BV(PC3) )
#define FAN_ON  ( PORTC &= ~_BV(PC3) )
#define FAN_IS_ON ( !(PINC & _BV(PC3)) )
#define FAN_IS_OFF ( PINC & _BV(PC3) )

// THIS IS WHERE IT GETS DANGEROUS
// YOU CAN START A FIRE AND DO A LOT OF HARM WITH
// THE HEATER / TRIAC COMMANDS
#define TRIAC_ON ( PORTB &= ~_BV(PB1) )
#define HEATER_ON TRIAC_ON
#define TRIAC_OFF ( PORTB |= _BV(PB1) )
#define HEATER_OFF TRIAC_OFF

#define SW0_PRESSED ( !(PINB & _BV(PB5)) )
#define SW1_PRESSED ( !(PINB & _BV(PB2)) )

#define REEDSW_CLOSED ( !(PINB & _BV(PB4)) )
#define REEDSW_OPEN ( PINB & _BV(PB4) )

#define SHOW_SETPOINT_TIMEOUT 2000L

#define HEATER_DUTY_CYCLE_MAX 512L
#define PWM_CYCLES 512L

#define P_GAIN_DEFAULT 650
#define I_GAIN_DEFAULT 15
#define D_GAIN_DEFAULT 500
#define I_THRESH_DEFAULT 45
#define P_GAIN_SCALING 100.0
#define I_GAIN_SCALING 10000.0
#define D_GAIN_SCALING 25.0

#define TEMP_OFFSET_CORR_DEFAULT 33
#define TEMP_GAIN_DEFAULT 100

#define TEMP_SETPOINT_DEFAULT 75

#define TEMP_AVERAGES_DEFAULT 250L
#define TEMP_REACHED_MARGIN 3

#define MAX_TEMP_ERR 550L
#define SAFE_TO_TOUCH_TEMP 40

#define FAN_ON_TEMP 60
#define FAN_OFF_TEMP 45
#define FAN_OFF_TEMP_FANONLY (SAFE_TO_TOUCH_TEMP - 2)

#define FAN_OFF_TEMP_DELAY_SEC 15
#define FAN_OFF_TEMP_DELAY_MILLI (FAN_OFF_TEMP_DELAY_SEC * 1000)


//
// Comment out the following 2 #defines, if you want to use the FAN-speed mod (HW changes required)
// Continue reading below...
//

#define FAN_SPEED_MIN_DEFAULT 130UL
#define FAN_SPEED_MAX_DEFAULT 400UL

//
// Good starting values with BLDC FAN-speed mod
//
// #define FAN_SPEED_MIN_DEFAULT 450UL
// #define FAN_SPEED_MAX_DEFAULT 800UL
//
// --> Don't forget to extend the ranges in the .ino <--
//
// CPARAM fan_speed_min = { 0, 999, FAN_SPEED_MIN_DEFAULT, FAN_SPEED_MIN_DEFAULT, 18, 19 };
// CPARAM fan_speed_max = { 0, 999, FAN_SPEED_MAX_DEFAULT, FAN_SPEED_MAX_DEFAULT, 20, 21 };
//

#define FAN_CURRENT_MIN_DEFAULT 30UL
#define FAN_CURRENT_MAX_DEFAULT 71UL

#define SLP_TIMEOUT_DEFAULT 10

#define KEY_DDR         DDRB
#define KEY_PORT        PORTB
#define KEY_PIN         PINB
#define KEY_UP          5
#define KEY_DOWN        2
#define ALL_KEYS        (1<<KEY_DOWN | 1<<KEY_UP)

#define REPEAT_MASK     (1<<KEY_DOWN | 1<<KEY_UP)
#define REPEAT_START    20	// after 20*20.48ms = 409.6ms
#define REPEAT_NEXT     8	// every 6*20.48ms = 122.88ms

#endif /* MAIN_H */
