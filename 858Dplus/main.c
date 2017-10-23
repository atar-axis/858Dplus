#include "main.h"

#include "debugprint.h"
#include "watchdog.h"
#include "debugprint.h"

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <util/delay.h>

/*
 * TODO:
 * - Remove _all_ the arduino specific code, it's not an arduino! 
 * - use plain C for consistency and speed
 * - merge rol_string´and display_string_running() into one single, nonblocking function
 * - avoid jumping around while cooling down (smoothing necessary)
 * - show the dot for temp_gain_correction (tgc) (2.40 instead of 240)
 * - Leave menu by long pressing of both buttons
 * - delay big steps a bit more
 */

/*
 * This is a custom firmware for my 'Youyue 858D+' hot-air soldering station.
 * It may or may not be useful to you, always double check if you use it.
 *
 * V1.47
 *
 * 2017    - Florian Dollinger
 * 2015/16 - Robert Spitzenpfeil
 * 2015    - Moritz Augsburger
 *
 * License: GNU GPL v2
 *
 *
 * Developed for / tested on by Robert Spitzenpfeil:
 * -------------------------------------------------
 *
 * Date:	2015-02-01
 * PCB version: 858D V6.0
 * Date code:   20140415
 *
 * Developed for / tested on by Moritz Augsburger:
 * -----------------------------------------------
 *
 * Date:	2015-02-01
 * PCB version: 858D V6.0
 * Date code:   20140415
 * 
 * Reported to work with (I did not test these myself):
 * ----------------------------------------------------
 *
 * PCB version: 858D V4.3
 * Date code:   20130529
 * HW mods:     not tested!
 *
 * ---
 *
 * PCB version: 858D V4.10
 * Date code:	20140112
 * HW mods:	not tested!
 *
 */

#define FW_MAJOR_V 2
#define FW_MINOR_V_A 0
#define FW_MINOR_V_B 0

/* PIN MAPPING:
 * ---
 * PC5: FAN-speed (A5 in Arduino lingo) (OK)
 * PC3: TIP122.base --> FAN (OK)
 * PC2: fan-current-sense mod (OPTIONAL) - see Docs folder
 * PC0: ADC <-- amplif. thermo couple voltage (A0 in Arduino lingo) (OK)
 * #21: AREF <--- about 2.5V as analog reference for ADC
 * PB1: opto-triac driver !! THIS IS DANGEROUS TO USE !! (OK)
 *
 * PB0: 7-seg digit 0 [common Anode] (OK) 
 * PB7: 7-seg digit 1 [common Anode] (OK)
 * PB6: 7-seg digit 2 [common Anode] (OK)
 *
 * PD0: 7-seg top (OK)
 * PD1: 7-seg bottom left (OK)
 * PD2: 7-seg bottom (OK)
 * PD3: 7-seg top left (OK)
 * PD4: 7-seg dot (OK)
 * PD5: 7-seg bottom right (OK)
 * PD6: 7-seg middle (OK)
 * PD7: 7-seg top right (OK)
 *
 * PB5: SW1 (button1) (OK)
 * PB2: SW2 (button2) (OK)
 * PB4: reed switch (wand cradle sensor) (OK)
 */




//////////////////////////////////////////////////////////////////////////
// GLOBAL VARIABLES
//////////////////////////////////////////////////////////////////////////

// FRAMEBUFFERS
//-----------------------------------------------
// Contains the values to be displayed next

// fb is the "raw" data which is directly displayed by the timer-ISR
uint8_t fb[3] = { 0xFF, 0xFF, 0xFF };	// dig0, dig1, dig2
	
// framebuffer is a bit more structured, but needs and fb_update() to be stored in fb
// right digit, middle digit, left digit, right dot, middle dot, left dot, change indicator
framebuffer_t framebuffer = { {0x00, 0x00, 0x00}, {0, 0, 0}, 0 };


// PARAMETERS
//-----------------------------------------------
// min, max, default, value, eep_addr_high, eep_addr_low
CPARAM p_gain = {"P", 0, 999, P_GAIN_DEFAULT, P_GAIN_DEFAULT, (uint8_t*) 2, (uint8_t*) 3 };
CPARAM i_gain = {"I", 0, 999, I_GAIN_DEFAULT, I_GAIN_DEFAULT, (uint8_t*) 4, (uint8_t*) 5 };
CPARAM d_gain = {"D", 0, 999, D_GAIN_DEFAULT, D_GAIN_DEFAULT, (uint8_t*) 6, (uint8_t*) 7 };
CPARAM i_thresh = {"ITH", 0, 100, I_THRESH_DEFAULT, I_THRESH_DEFAULT, (uint8_t*) 8, (uint8_t*) 9 };
CPARAM temp_offset_corr = {"TOF", -100, 100, TEMP_OFFSET_CORR_DEFAULT, TEMP_OFFSET_CORR_DEFAULT, (uint8_t*) 10, (uint8_t*) 11 };
CPARAM temp_gain_corr = {"TGN", 100, 999, TEMP_GAIN_DEFAULT, TEMP_GAIN_DEFAULT, (uint8_t*) 30, (uint8_t*) 31 };
CPARAM temp_setpoint = {"", 50, 500, TEMP_SETPOINT_DEFAULT, TEMP_SETPOINT_DEFAULT, (uint8_t*) 12, (uint8_t*) 13 };
CPARAM temp_averages = {"AVG", 100, 999, TEMP_AVERAGES_DEFAULT, TEMP_AVERAGES_DEFAULT, (uint8_t*) 14, (uint8_t*) 15 };
CPARAM slp_timeout = {"SLP", 0, 30, SLP_TIMEOUT_DEFAULT, SLP_TIMEOUT_DEFAULT, (uint8_t*) 16, (uint8_t*) 17 };
CPARAM fan_only = {"FAN", 0, 1, 0, 0, (uint8_t*) 26, (uint8_t*) 27 };
CPARAM display_adc_raw = {"ADC", 0, 1, 0, 0, (uint8_t*) 28, (uint8_t*) 29 };
#ifdef CURRENT_SENSE_MOD
CPARAM fan_current_min = {"FCL", 0, 999, FAN_CURRENT_MIN_DEFAULT, FAN_CURRENT_MIN_DEFAULT, (uint8_t*) 22, (uint8_t*) 23 };
CPARAM fan_current_max = {"FCH", 0, 999, FAN_CURRENT_MAX_DEFAULT, FAN_CURRENT_MAX_DEFAULT, (uint8_t*) 24, (uint8_t*) 25 };
#else
//
// See youyue858d.h if you want to use the 'FAN-speed mod' (HW changes required)
// The following 2 CPARAM lines need changes in that case
//
CPARAM fan_speed_min = {"FSL", 120, 180, FAN_SPEED_MIN_DEFAULT, FAN_SPEED_MIN_DEFAULT, (uint8_t*) 18, (uint8_t*) 19 };
CPARAM fan_speed_max = {"FSH", 300, 400, FAN_SPEED_MAX_DEFAULT, FAN_SPEED_MAX_DEFAULT, (uint8_t*) 20, (uint8_t*) 21 };
#endif

// KEY STATE
//-----------------------------------------------
volatile uint8_t key_state;	// debounced and inverted key state: bit = 1: key pressed
volatile uint8_t key_press;	// key press detect
volatile uint8_t key_rpt;	// key long press and repeat


volatile uint8_t display_blink;


///////////////////////////////////
// MAIN FUNCTION
///////////////////////////////////

int main(void)
{
	init();			// make sure the Arduino-specific stuff is up and running (timers... see 'wiring.c')
	setup_858D();

#ifdef DISPLAY_MCUSR
	HEATER_OFF;
	FAN_ON;
	display_number(_mcusr);
	MCUSR = 0;
	// ATmega168 MCUSR (MSB to LSB): x-x-x-x-WDRF-BORF-EXTRF-PORF
	_delay_ms(1000);
#endif

#ifdef USE_WATCHDOG

	if(watchdogCheck()){
		HEATER_OFF;
		FAN_ON;
		while (1) {
			display_string("RST");
			_delay_ms(1000);
			clear_display();
			_delay_ms(1000);
		}
	}


#endif

	show_firmware_version();
	
#ifdef USE_WATCHDOG
	watchdogTestCpuFreq();
#endif

	fan_test();
	
#ifdef USE_WATCHDOG
	watchdogOn();
#endif

#ifdef DEBUG
	Serial.begin(2400);
	Serial.println("\nRESET");
#endif


	//////////////////////////////////////////////////////////////////////////
	// MAIN LOOP
	//////////////////////////////////////////////////////////////////////////

	while (1) {
		
		//////////////////////////////////////////////////////////////////////////
		// STATIC VARIABLES
		//////////////////////////////////////////////////////////////////////////
		// Remember: Static variables are initialized only once!
		//
		
#ifdef DEBUG
		int32_t start_time = micros();
#endif
		static int16_t temp_inst = 0;
		static int32_t temp_accu = 0;
		static int16_t temp_average = 0;
		static int16_t temp_average_previous = 0;

		static int32_t button_input_time = 0;

		static int16_t heater_ctr = 0;
		static int16_t heater_duty_cycle = 0;
		static int16_t error = 0;
		static int32_t error_accu = 0;
		static int16_t velocity = 0;
		static float PID_drive = 0;

		static uint8_t temp_setpoint_saved = 1;
		static int32_t temp_setpoint_saved_time = 0;

		static uint32_t heater_start_time = 0;
		
		//static uint32_t temp_low_fanonly_firstTime = 0;
		static uint32_t temp_low_firstTime = 0;
				
		//static bool temp_low_fanonly_firstEvent = 1;
		static bool temp_low_firstEvent = 1;
		
		static bool fan_off_allowed = 0;
		static bool was_hot_before = 0;

		
		//////////////////////////////////////////////////////////////////////////
		// PID LOOP / HEATER CONTROL
		//////////////////////////////////////////////////////////////////////////
		
		uint16_t adc_raw = adcRead(PC0);	// need raw value later, store it here and avoid 2nd ADC read		
		temp_inst = (adc_raw / (temp_gain_corr.value / 100)) + temp_offset_corr.value;	// approx. temp in °C

		if (temp_inst < 0) {
			temp_inst = 0;
		}
		
		// pid loop / heater handling
		
		// FAN-ONLY MODE
		if (fan_only.value == 1 || REEDSW_CLOSED) {
			
			HEATER_OFF;
			heater_start_time = millis();
			clear_dot();
		
		// REGULAR/NORMAL MODE
		} else if (REEDSW_OPEN
					&& (temp_setpoint.value >= temp_setpoint.value_min)
					&& (temp_average < MAX_TEMP_ERR)
					&& ((millis() - heater_start_time) < ((uint32_t) (slp_timeout.value) * 60 * 1000))) {

			FAN_ON;

			error = temp_setpoint.value - temp_average;
			velocity = temp_average_previous - temp_average;

			if (abs(error) < i_thresh.value) {
				// if close enough to target temperature use PID control
				error_accu += error;
			} else {
				// otherwise only use PD control (avoids issues with error_accu growing too large)
				error_accu = 0;
			}

			PID_drive =
			    error * (p_gain.value / P_GAIN_SCALING) + error_accu * (i_gain.value / I_GAIN_SCALING) +
			    velocity * (d_gain.value / D_GAIN_SCALING);

			heater_duty_cycle = (int16_t) (PID_drive);

			if (heater_duty_cycle > HEATER_DUTY_CYCLE_MAX) {
				heater_duty_cycle = HEATER_DUTY_CYCLE_MAX;
			}

			if (heater_duty_cycle < 0) {
				heater_duty_cycle = 0;
			}

			if (heater_ctr < heater_duty_cycle) {
				set_dot();
				HEATER_ON;
			} else {
				HEATER_OFF;
				clear_dot();
			}

			heater_ctr++;
			
			if (heater_ctr == PWM_CYCLES) {
				heater_ctr = 0;
			}
			
		} else {
			
			HEATER_OFF;
			clear_dot();
			
		}

		//////////////////////////////////////////////////////////////////////////
		// SMOOTHING TEMPERATURE
		//////////////////////////////////////////////////////////////////////////
		// Accumulates as many values as defined in the menu entry "AVG"
		// and averages them to calculate the smoothed temperature.
		//

		static uint16_t temp_avg_ctr = 0;

		temp_accu += temp_inst;
		temp_avg_ctr++;

		if (temp_avg_ctr == (uint16_t) (temp_averages.value)) {
			temp_average_previous = temp_average;
			temp_average = temp_accu / temp_averages.value;
			temp_accu = 0;
			temp_avg_ctr = 0;
		}
		
		
		//////////////////////////////////////////////////////////////////////////
		// FAN SHUTDOWN DELAY
		//////////////////////////////////////////////////////////////////////////
		
		// If the temperature is LOW		

		if(temp_average <= FAN_OFF_TEMP){

			// First iteration
			// Remember the time when the temperature became low the first time (only if it was hot before)
			
			if(was_hot_before && temp_low_firstEvent){
				temp_low_firstTime = millis();
				temp_low_firstEvent = 0;
			}
			
			// All the other iterations:
			// Calculate the time since the temperature is low
			// If it is >= FAN_OFF_TEMP_DELAY_MILLI, allow to shut down the FAN
			// otherwise, cool a bit more down
			
			if(!temp_low_firstEvent){
				if((millis() - temp_low_firstTime) >= FAN_OFF_TEMP_DELAY_MILLI)
					fan_off_allowed = 1;
			}
			
		// If the temperature is HIGH
		
		} else {
			
			// reset variables
			was_hot_before = 1;
			temp_low_firstEvent = 1;
			
			fan_off_allowed = 0;
		}
		
		
		//////////////////////////////////////////////////////////////////////////
		// FAN CRADLE HANDLING
		//////////////////////////////////////////////////////////////////////////
		// TODO: fan_off_allowed for fan_only_mode

		if (temp_average >= FAN_ON_TEMP) {
			
			FAN_ON;
			
		} else if (REEDSW_CLOSED && fan_only.value == 1 && (temp_average <= FAN_OFF_TEMP_FANONLY)) {
			
			FAN_OFF;
						
		} else if (REEDSW_CLOSED && fan_only.value == 0 && (temp_average <= FAN_OFF_TEMP)) {
			
			if(fan_off_allowed)	FAN_OFF;
			
		} else if (REEDSW_OPEN) {
			
			FAN_ON;
			
		}
		
		
		/////////////////////////////////
		// MENU KEY HANDLING
		/////////////////////////////////
		// CHANGING THE TEMPERATURE
		//
		// TODO: Do not in-/decrease by 10, but increase the change rate - it should depend on how long you already pressed the button
		
		// - INCREASE BY 1
		if (get_key_short(1 << KEY_UP)) {
			button_input_time = millis();
			if (temp_setpoint.value < temp_setpoint.value_max) {
				temp_setpoint.value++;
			}
			temp_setpoint_saved = 0;
			
		// - DECREASE BY 1
		} else if (get_key_short(1 << KEY_DOWN)) {
			button_input_time = millis();
			if (temp_setpoint.value > temp_setpoint.value_min) {
				temp_setpoint.value--;
			}
			temp_setpoint_saved = 0;
			
		// - INCREASE BY 10
		} else if (get_key_long_r(1 << KEY_UP) || get_key_rpt_l(1 << KEY_UP)) {
			button_input_time = millis();
			if (temp_setpoint.value < (temp_setpoint.value_max - 10)) {
				temp_setpoint.value += 10;
			} else {
				temp_setpoint.value = temp_setpoint.value_max;
			}
			temp_setpoint_saved = 0;
			
		// - DECREASE BY 10
		} else if (get_key_long_r(1 << KEY_DOWN) || get_key_rpt_l(1 << KEY_DOWN)) {
			button_input_time = millis();

			if (temp_setpoint.value > (temp_setpoint.value_min + 10)) {
				temp_setpoint.value -= 10;
			} else {
				temp_setpoint.value = temp_setpoint.value_min;
			}

			temp_setpoint_saved = 0;
			
		// ENTER THE MENU
		} else if (get_key_common_l(1 << KEY_UP | 1 << KEY_DOWN)) {
			HEATER_OFF;	// security reasons, delay below!
			
			watchdogOff(false);
			_delay_ms((uint16_t)(20.48 * (REPEAT_START - 3) + 1));

			
			// ENTER CONFIG MENU	
			if (get_key_long_r(1 << KEY_UP | 1 << KEY_DOWN)) {
				
				show_config();

			// FAN ONLY MODE
			// This mode is entered only at setup_858D() below
			} else {
				get_key_press(1 << KEY_UP | 1 << KEY_DOWN);	// clear inp state
				fan_only.value ^= 0x01;
				temp_setpoint_saved = 0;
				if (fan_only.value == 0) {
					button_input_time = millis();	// show set temp after disabling fan only mode
				}
				display_blink = 0;	// make sure we start displaying "FAN" or set temp
			}
#ifdef USE_WATCHDOG
			watchdogOn();
#endif
		}
		
		
		//////////////////////////////////////////////////////////////////////////
		// TEMPERATURE SECURITY
		//////////////////////////////////////////////////////////////////////////
		
		if (temp_average >= MAX_TEMP_ERR) {
			
			// something might have gone terribly wrong
			HEATER_OFF;
			FAN_ON;
			watchdogOff(false);

			while (1) {
				// stay here until the power is cycled
				// make sure the user notices the error by blinking "FAN"
				// and don't resume operation if the error goes away on its own
				//
				// possible reasons to be here:
				//
				// * wand is not connected (false temperature reading)
				// * thermo couple has failed
				// * true over-temperature condition
				//
				display_string("*C");
				_delay_ms(1000);
				display_string("ERR");
				_delay_ms(2000);
				clear_display();
				_delay_ms(1000);
			}
		}
		
		
		//////////////////////////////////////////////////////////////////////////
		// DISPLAY OUTPUT
		//////////////////////////////////////////////////////////////////////////
		
		if ((millis() - button_input_time) < SHOW_SETPOINT_TIMEOUT) {
			if (display_blink < 5) {
				clear_display();
			} else {
				display_number(temp_setpoint.value);	// show temperature setpoint
			}
		} else {
			if (temp_setpoint_saved == 0) {
				set_eeprom_saved_dot();
				eep_save(&temp_setpoint);
				eep_save(&fan_only);
				temp_setpoint_saved_time = millis();
				temp_setpoint_saved = 1;
			} else if (temp_average <= SAFE_TO_TOUCH_TEMP) {
				if (fan_only.value == 1) {
					display_string(fan_only.text);
				} else {
					display_string("---");
				}
			} else if (fan_only.value == 1) {
				if (display_blink < 20) {
					display_string(fan_only.text);
				} else {
					display_number(temp_average);
				}
			} else if (display_adc_raw.value == 1) {
				display_number(adc_raw);
			} else if (abs((int16_t) (temp_average) - (int16_t) (temp_setpoint.value)) < TEMP_REACHED_MARGIN) {
				display_number(temp_setpoint.value);	// avoid showing insignificant fluctuations on the display (annoying)
			} else {
				display_number(temp_average);
			}
		}

		if ((millis() - temp_setpoint_saved_time) > 500) {
			clear_eeprom_saved_dot();
		}

		fb_update();

#if defined(WATCHDOG_TEST) && defined(USE_WATCHDOG)
		// watchdog test
		if (temp_average > 100) {
			_delay_ms(150);
		}
#endif

#ifdef USE_WATCHDOG
		watchdogReset();
#endif

#ifdef DEBUG
		int32_t stop_time = micros();
		Serial.println(stop_time - start_time);
#endif
	}

}


//////////////////////////////////////////////////////////////////////////
// INITIAL SETUP ROUTINE
//////////////////////////////////////////////////////////////////////////

void setup_858D(void)
{
	HEATER_OFF;
	DDRB |= _BV(PB1);	// set as output for TRIAC control

	DDRB &= ~(_BV(PB5) | _BV(PB2));	// set as inputs (switches)
	PORTB |= (_BV(PB5) | _BV(PB2));	// pull-up on

	DDRB &= ~_BV(PB4);	// set as input (reed sensor)
	PORTB |= _BV(PB4);	// pull-up on

	FAN_OFF;
	DDRC |= _BV(PC3);	// set as output (FAN control)

	DDRD |= 0xFF;		// all as outputs (7-seg segments)
	DDRB |= (_BV(PB0) | _BV(PB6) | _BV(PB7));	// 7-seg digits 1,2,3

#ifdef CURRENT_SENSE_MOD
	DDRC &= ~_BV(PC2);	// set as input
#endif

	setup_timer1_ctc();	// needed for background display refresh
	
	adcInit();

	if (eeprom_read_byte(0) != 0x22) {
		// check if the firmware was just flashed and the EEPROM is therefore empty
		// assumption: full chip erase with ISP programmer (preserve eeprom fuse NOT set!)
		// if so, restore default parameter values & write a 'hint' to address 0
		restore_default_conf();
		eeprom_write_byte(0, 0x22);
	}

	if (SW0_PRESSED && SW1_PRESSED) {
		restore_default_conf();
	} else if (SW0_PRESSED) {
		display_string("FAN");
		_delay_ms(1000);
		display_string("TST");
		_delay_ms(1000);
		FAN_ON;
		while (1) {
			uint16_t fan;
			_delay_ms(500);
#ifdef CURRENT_SENSE_MOD
			fan = adcRead(PC2);
#else				//CURRENT_SENSE_MOD
			fan = adcRead(PC5);
#endif				//CURRENT_SENSE_MOD
			display_number(fan);
		}
	}

	eep_load(&p_gain);
	eep_load(&i_gain);
	eep_load(&d_gain);
	eep_load(&i_thresh);
	eep_load(&temp_offset_corr);
	eep_load(&temp_gain_corr);
	eep_load(&temp_setpoint);
	eep_load(&temp_averages);
	eep_load(&slp_timeout);
	eep_load(&fan_only);
	eep_load(&display_adc_raw);
#ifdef CURRENT_SENSE_MOD
	eep_load(&fan_current_min);
	eep_load(&fan_current_max);
#else
	eep_load(&fan_speed_min);
	eep_load(&fan_speed_max);
#endif
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


//////////////////////////////////////////////////////////////////////////
// EEPROM SAVE & LOAD
//////////////////////////////////////////////////////////////////////////

void eep_save(CPARAM * param)
{
	// make sure NOT to save invalid parameter values
	if ((param->value >= param->value_min) && (param->value <= param->value_max)) {
		// nothing to do
	} else {
		// reset to sensible minimum
		param->value = param->value_default;
	}
	eeprom_update_byte(param->p_eep_addr_high, high(param->value));
	eeprom_update_byte(param->p_eep_addr_low, low(param->value));
}


void eep_load(CPARAM * param)
{
	int16_t tmp = (eeprom_read_byte(param->p_eep_addr_high) << 8) | eeprom_read_byte(param->p_eep_addr_low);

	// make sure NOT to restore invalid parameter values
	if ((tmp >= param->value_min) && (tmp <= param->value_max)) {
		// the value was good, so we use it
		param->value = tmp;
	} else {
		// reset to sensible value
		param->value = param->value_default;
	}
}


//////////////////////////////////////////////////////////////////////////
// RESTORE THE DEFAULT CONFIGURATION
//////////////////////////////////////////////////////////////////////////

void restore_default_conf(void)
{
	p_gain.value = p_gain.value_default;
	i_gain.value = i_gain.value_default;
	d_gain.value = d_gain.value_default;
	i_thresh.value = i_thresh.value_default;
	temp_offset_corr.value = temp_offset_corr.value_default;
	temp_gain_corr.value = temp_gain_corr.value_default;
	temp_setpoint.value = temp_setpoint.value_default;
	temp_averages.value = temp_averages.value_default;
	slp_timeout.value = slp_timeout.value_default;
	fan_only.value = 0;
	display_adc_raw.value = 0;
#ifdef CURRENT_SENSE_MOD
	fan_current_min.value = fan_current_min.value_default;
	fan_current_max.value = fan_current_max.value_default;
#else
	fan_speed_min.value = fan_speed_min.value_default;
	fan_speed_max.value = fan_speed_max.value_default;
#endif

	eep_save(&p_gain);
	eep_save(&i_gain);
	eep_save(&d_gain);
	eep_save(&i_thresh);
	eep_save(&temp_offset_corr);
	eep_save(&temp_gain_corr);
	eep_save(&temp_setpoint);
	eep_save(&temp_averages);
	eep_save(&slp_timeout);
	eep_save(&fan_only);
	eep_save(&display_adc_raw);
#ifdef CURRENT_SENSE_MOD
	eep_save(&fan_current_min);
	eep_save(&fan_current_max);
#else
	eep_save(&fan_speed_min);
	eep_save(&fan_speed_max);
#endif
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
		portout = (uint8_t) (~0xC9);	// '°'
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


//////////////////////////////////////////////////////////////////////////
// STRING MANIPULATION
//////////////////////////////////////////////////////////////////////////

/*
 * ROTATES THE CSTRING
 * to the left by 1
 */
void rol_string(char* text){
	
	int len = strlen(text);
	char tmp = text[0];
	
	for(int i = 0; i <= len-2; i++){
		text[i] = text[i+1];
	}
	
	text[len-1] = tmp;
}


//////////////////////////////////////////////////////////////////////////
// INITIAL FAN TEST
//////////////////////////////////////////////////////////////////////////
//
// check if the fan speed / fan current is within the given limits
// otherwise: stop the system and show an error

void fan_test(void)
{
	char cradle_err[] = "CRADLE ";
	char fanSpd_err[] = "FANSPEED ";
	//char fanCur_err[] = "FANCUR ";
	
	
	HEATER_OFF;

	// if the wand is not in the cradle when powered up, go into a safe mode
	// and display an error

	while (!REEDSW_CLOSED) {
 		display_string(cradle_err);
		rol_string(cradle_err);
		_delay_ms(500);
	}

	FAN_ON;
	_delay_ms(3000);
	
#ifdef CURRENT_SENSE_MOD
	uint16_t fan_current = adcRead(PC2);

	if ((fan_current < (uint16_t) (fan_current_min.value)) || (fan_current > (uint16_t) (fan_current_max.value))) {
#else				//CURRENT_SENSE_MOD
	uint16_t fan_speed = adcRead(PC5);

	if ((fan_speed < (uint16_t) (fan_speed_min.value)) || (fan_speed > (uint16_t) (fan_speed_max.value))) {
#endif				//CURRENT_SENSE_MOD


		// the fan is not working as it should
		FAN_OFF;
		while (1) {
#ifdef CURRENT_SENSE_MOD
			display_string_running(fanCur_err);
#else				//CURRENT_SENSE_MOD
			display_string_running(fanSpd_err);
#endif				//CURRENT_SENSE_MOD
			_delay_ms(2000);
			clear_display();
			_delay_ms(1000);
		}
	}

	FAN_OFF;

}

//////////////////////////////////////////////////////////////////////////
// SHOW FIRMWARE VERSION
//////////////////////////////////////////////////////////////////////////
// shows the actual firmware version on the display

void show_firmware_version(void)
{
	framebuffer.digit[0] = FW_MINOR_V_B;	// dig0
	framebuffer.digit[1] = FW_MINOR_V_A;	// dig1
	framebuffer.digit[2] = FW_MAJOR_V;	// dig2
	framebuffer.dot[0] = 0;	// dig0.dot
	framebuffer.dot[1] = 0;	// dig1.dot
	framebuffer.dot[2] = 1;	// dig2.dot
	framebuffer.changed = 1;
	fb_update();
	_delay_ms(2000);
}


//////////////////////////////////////////////////////////////////////////
// TIMER 1 SETUP
//////////////////////////////////////////////////////////////////////////

void setup_timer1_ctc(void)
{
	// ATmega168 running at 8MHz internal RC oscillator
	// Timer1 (16bit) Settings:
	// prescaler (frequency divider) values:   CS12    CS11   CS10
	//                                           0       0      0    stopped
	//                                           0       0      1      /1  
	//                                           0       1      0      /8  
	//                                           0       1      1      /64
	//                                           1       0      0      /256 
	//                                           1       0      1      /1024
	//                                           1       1      0      external clock on T1 pin, falling edge
	//                                           1       1      1      external clock on T1 pin, rising edge
	//
	uint8_t _sreg = SREG;	/* save SREG */
	cli();			/* disable all interrupts while messing with the register setup */

	/* set prescaler to 256 */
	TCCR1B &= ~(_BV(CS11) | _BV(CS10));
	TCCR1B |= _BV(CS12);

	/* set WGM mode 4: CTC using OCR1A */
	TCCR1A &= ~(_BV(WGM10) | _BV(WGM11));
	TCCR1B |= _BV(WGM12);
	TCCR1B &= ~_BV(WGM13);

	/* normal operation - disconnect PWM pins */
	TCCR1A &= ~(_BV(COM1A1) | _BV(COM1A0) | _BV(COM1B1) | _BV(COM1B0));

	/* set top value for TCNT1 */
	OCR1A = 640;		// key debouncing every 20.48ms
	OCR1B = 8;		// new segment every 256µs, complete display update every 6ms <=> 160Hz

	/* enable COMPA and COMPB isr */
	TIMSK1 |= _BV(OCIE1A) | _BV(OCIE1B);

	/* restore SREG with global interrupt flag */
	SREG = _sreg;
}


//////////////////////////////////////////////////////////////////////////
// INTERRUPT SERVICE ROUTINE
// -> TIMER 1, COMPA
//////////////////////////////////////////////////////////////////////////
// refresh the display (background)
//

ISR(TIMER1_COMPB_vect)
{
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
// INTERRUPT SERVICE ROUTINE
// -> TIMER 1, COMPB
//////////////////////////////////////////////////////////////////////////
// reads out the buttons
//

ISR(TIMER1_COMPA_vect)
{
	// explained in https://www.mikrocontroller.net/articles/Entprellung#Komfortroutine_.28C_f.C3.BCr_AVR.29
	static uint8_t ct0, ct1, rpt;
	uint8_t i;

	i = key_state ^ ~KEY_PIN;	// key changed ?
	
	ct0 = ~(ct0 & i);	// reset or count ct0
	ct1 = ct0 ^ (ct1 & i);	// reset or count ct1
	
	i &= ct0 & ct1;		// count until roll over ?
	
	key_state ^= i;		// then toggle debounced state
	
	key_press |= key_state & i;	// 0->1: key press detect


	// REPEAT_MASK: vector, which inputs are checked for repeated input?
	// REPEAT_START: how many time to delay at first?
	// REPEAT_NEXT:  how many time should pass until we recognice a "long press"
	
	if ((key_state & REPEAT_MASK) == 0)	// check repeat function
		rpt = REPEAT_START;	// start delay
		
	if (--rpt == 0) {
		rpt = REPEAT_NEXT;	// repeat delay
		key_rpt |= key_state & REPEAT_MASK;
	}

	if (++display_blink > 50)
		display_blink = 0;
}


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


//////////////////////////////////////////////////////////////////////////
// DISPLAYS THE CONFIG MENU
//////////////////////////////////////////////////////////////////////////

void show_config(){
				
	// We enter the first parameter config menu.
	// That function is blocking, it repeatedly loops to react to user inputs like "value up" or "value down".
	// We leave that menu-entry by pressing both buttons again (and automatically enter the next menu-entry)
	
	show_parameter_config(&p_gain, p_gain.text);
	show_parameter_config(&i_gain, i_gain.text);
	show_parameter_config(&d_gain, d_gain.text);
	show_parameter_config(&i_thresh, i_thresh.text);
	show_parameter_config(&temp_offset_corr, temp_offset_corr.text);
	show_parameter_config(&temp_gain_corr, temp_gain_corr.text);
	show_parameter_config(&temp_averages, temp_averages.text);
	show_parameter_config(&slp_timeout, slp_timeout.text);
	show_parameter_config(&display_adc_raw, display_adc_raw.text);
#ifdef CURRENT_SENSE_MOD
	show_parameter_config(&fan_current_min, fan_current_min.text);
	show_parameter_config(&fan_current_max, fan_current_max.text);
#else
	show_parameter_config(&fan_speed_min, fan_speed_min.text);
	show_parameter_config(&fan_speed_max, fan_speed_max.text);
#endif

	// TODO:
	//
	// 			if(get_key_long_r(1 << KEY_UP | 1 << KEY_DOWN)){
	//
	// 				do {
	//
	// 					show_parameter_config(&(param[i]), param[i].text);
	//
	// 				} while(!get_key_long_r(1 << KEY_UP | 1 << KEY_DOWN))
	//
	// 			}
	
}

//////////////////////////////////////////////////////////////////////////
// DISPLAYS THE CURRENT MENU ENTRY
//////////////////////////////////////////////////////////////////////////
// Blocking!
// Leave the current menu entry by long pressing both, up and down


void show_parameter_config(CPARAM * param, const char *string)
{
	display_string(string);
	_delay_ms(750);		// let the user read what is shown

	uint8_t loop = 1;

	while (loop == 1) {
		
		// INCREASE BY 1
		if (get_key_short(1 << KEY_UP)) {
			if (param->value < param->value_max) {
				param->value++;
			}
			
			// DECREASE BY 1
			} else if (get_key_short(1 << KEY_DOWN)) {
			if (param->value > param->value_min) {
				param->value--;
			}
			
			// INCREASE BY 10
			} else if (get_key_long_r(1 << KEY_UP) || get_key_rpt_l(1 << KEY_UP)) {
			if (param->value < param->value_max - 10) {
				param->value += 10;
			}
			
			// DECREASE BY 10
			} else if (get_key_long_r(1 << KEY_DOWN) || get_key_rpt_l(1 << KEY_DOWN)) {
			if (param->value > param->value_min + 10) {
				param->value -= 10;
			}
			
			// LEAVE THE MENU-ENTRY (next one is entered in the main loop)
			} else if (get_key_common(1 << KEY_UP | 1 << KEY_DOWN)) {
			loop = 0;
		}

		display_number(param->value);
	}
	set_eeprom_saved_dot();
	eep_save(param);
	_delay_ms(1000);
	clear_eeprom_saved_dot();
}


void adcInit(void)
{
        // External reference-voltage (2.5V)
        ADMUX = (0<<REFS0) | (0<<REFS1);
  
        // Prescalar, divides the CPU clock frequency
        // A value between 50 and 200kHz is necessary
        // 20 MHz / 200 kHz = 100
        // 20 MHz /  50 kHz = 400
        // Lowest (in this case the only) prescalar within the range is 128, which is set by ADPSx '111'
        ADCSRA = (1<<ADPS2) | (1<<ADPS1) | (1<<ADPS0);
  
        // activate ADC
        ADCSRA |= (1<<ADEN);

        // Dummy readout
        ADCSRA |= (1<<ADSC);            // Start conversion
        while (ADCSRA & (1<<ADSC)) {}   // Wait for it
        (void) ADCW;                    // Read out
}


uint16_t adcRead( uint8_t channel )
{
        // Kanal waehlen, ohne andere Bits zu beeinflußen
        ADMUX = (ADMUX & ~(0x1F)) | (channel & 0x1F);

        ADCSRA |= (1<<ADSC);            // Start a single conversion
        while (ADCSRA & (1<<ADSC) ) {}  // Wait for it
        return ADCW;                    // Read out and return
}








// TEMPORARY STUFF BORROWED BY ARDUINO
// ---
// TODO: Replace by own/library code which is not(!) arduino specific

#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))

void init(){

        // this needs to be called before setup() or some functions won't
        // work there
        sei();

        // on the ATmega168, timer 0 is also used for fast hardware pwm
        // (using phase-correct PWM would mean that timer 0 overflowed half as often
        // resulting in different millis() behavior on the ATmega8 and ATmega168)
        #if defined(TCCR0A) && defined(WGM01)
        sbi(TCCR0A, WGM01);
        sbi(TCCR0A, WGM00);
        #endif

        // set timer 0 prescale factor to 64
        #if defined(__AVR_ATmega128__)
        // CPU specific: different values for the ATmega128
        sbi(TCCR0, CS02);
        #elif defined(TCCR0) && defined(CS01) && defined(CS00)
        // this combination is for the standard atmega8
        sbi(TCCR0, CS01);
        sbi(TCCR0, CS00);
        #elif defined(TCCR0B) && defined(CS01) && defined(CS00)
        // this combination is for the standard 168/328/1280/2560
        sbi(TCCR0B, CS01);
        sbi(TCCR0B, CS00);
        #elif defined(TCCR0A) && defined(CS01) && defined(CS00)
        // this combination is for the __AVR_ATmega645__ series
        sbi(TCCR0A, CS01);
        sbi(TCCR0A, CS00);
        #else
        #error Timer 0 prescale factor 64 not set correctly
        #endif

        // enable timer 0 overflow interrupt
        #if defined(TIMSK) && defined(TOIE0)
        sbi(TIMSK, TOIE0);
        #elif defined(TIMSK0) && defined(TOIE0)
        sbi(TIMSK0, TOIE0);
        #else
        #error	Timer 0 overflow interrupt not set correctly
        #endif

        // timers 1 and 2 are used for phase-correct hardware pwm
        // this is better for motors as it ensures an even waveform
        // note, however, that fast pwm mode can achieve a frequency of up
        // 8 MHz (with a 16 MHz clock) at 50% duty cycle

        #if defined(TCCR1B) && defined(CS11) && defined(CS10)
        TCCR1B = 0;

        // set timer 1 prescale factor to 64
        sbi(TCCR1B, CS11);
        #if F_CPU >= 8000000L
        sbi(TCCR1B, CS10);
        #endif
        #elif defined(TCCR1) && defined(CS11) && defined(CS10)
        sbi(TCCR1, CS11);
        #if F_CPU >= 8000000L
        sbi(TCCR1, CS10);
        #endif
        #endif
        // put timer 1 in 8-bit phase correct pwm mode
        #if defined(TCCR1A) && defined(WGM10)
        sbi(TCCR1A, WGM10);
        #endif

        // set timer 2 prescale factor to 64
        #if defined(TCCR2) && defined(CS22)
        sbi(TCCR2, CS22);
        #elif defined(TCCR2B) && defined(CS22)
        sbi(TCCR2B, CS22);
        //#else
        // Timer 2 not finished (may not be present on this CPU)
        #endif

        // configure timer 2 for phase correct pwm (8-bit)
        #if defined(TCCR2) && defined(WGM20)
        sbi(TCCR2, WGM20);
        #elif defined(TCCR2A) && defined(WGM20)
        sbi(TCCR2A, WGM20);
        //#else
        // Timer 2 not finished (may not be present on this CPU)
        #endif

        #if defined(TCCR3B) && defined(CS31) && defined(WGM30)
        sbi(TCCR3B, CS31);		// set timer 3 prescale factor to 64
        sbi(TCCR3B, CS30);
        sbi(TCCR3A, WGM30);		// put timer 3 in 8-bit phase correct pwm mode
        #endif

        #if defined(TCCR4A) && defined(TCCR4B) && defined(TCCR4D) /* beginning of timer4 block for 32U4 and similar */
        sbi(TCCR4B, CS42);		// set timer4 prescale factor to 64
        sbi(TCCR4B, CS41);
        sbi(TCCR4B, CS40);
        sbi(TCCR4D, WGM40);		// put timer 4 in phase- and frequency-correct PWM mode
        sbi(TCCR4A, PWM4A);		// enable PWM mode for comparator OCR4A
        sbi(TCCR4C, PWM4D);		// enable PWM mode for comparator OCR4D
        #else /* beginning of timer4 block for ATMEGA1280 and ATMEGA2560 */
        #if defined(TCCR4B) && defined(CS41) && defined(WGM40)
        sbi(TCCR4B, CS41);		// set timer 4 prescale factor to 64
        sbi(TCCR4B, CS40);
        sbi(TCCR4A, WGM40);		// put timer 4 in 8-bit phase correct pwm mode
        #endif
        #endif /* end timer4 block for ATMEGA1280/2560 and similar */

        #if defined(TCCR5B) && defined(CS51) && defined(WGM50)
        sbi(TCCR5B, CS51);		// set timer 5 prescale factor to 64
        sbi(TCCR5B, CS50);
        sbi(TCCR5A, WGM50);		// put timer 5 in 8-bit phase correct pwm mode
        #endif

        #if defined(ADCSRA)
        // set a2d prescaler so we are inside the desired 50-200 KHz range.
        #if F_CPU >= 16000000 // 16 MHz / 128 = 125 KHz
        sbi(ADCSRA, ADPS2);
        sbi(ADCSRA, ADPS1);
        sbi(ADCSRA, ADPS0);
        #elif F_CPU >= 8000000 // 8 MHz / 64 = 125 KHz
        sbi(ADCSRA, ADPS2);
        sbi(ADCSRA, ADPS1);
        cbi(ADCSRA, ADPS0);
        #elif F_CPU >= 4000000 // 4 MHz / 32 = 125 KHz
        sbi(ADCSRA, ADPS2);
        cbi(ADCSRA, ADPS1);
        sbi(ADCSRA, ADPS0);
        #elif F_CPU >= 2000000 // 2 MHz / 16 = 125 KHz
        sbi(ADCSRA, ADPS2);
        cbi(ADCSRA, ADPS1);
        cbi(ADCSRA, ADPS0);
        #elif F_CPU >= 1000000 // 1 MHz / 8 = 125 KHz
        cbi(ADCSRA, ADPS2);
        sbi(ADCSRA, ADPS1);
        sbi(ADCSRA, ADPS0);
        #else // 128 kHz / 2 = 64 KHz -> This is the closest you can get, the prescaler is 2
        cbi(ADCSRA, ADPS2);
        cbi(ADCSRA, ADPS1);
        sbi(ADCSRA, ADPS0);
        #endif
        // enable a2d conversions
        sbi(ADCSRA, ADEN);
        #endif

        // the bootloader connects pins 0 and 1 to the USART; disconnect them
        // here so they can be used as normal digital i/o; they will be
        // reconnected in Serial.begin()
        #if defined(UCSRB)
        UCSRB = 0;
        #elif defined(UCSR0B)
        UCSR0B = 0;
        #endif
}

#define clockCyclesPerMicrosecond() (F_CPU / 1000000L)
#define clockCyclesToMicroseconds(a) ((a) / clockCyclesPerMicrosecond ())

// the prescaler is set so that timer0 ticks every 64 clock cycles, and the
// the overflow handler is called every 256 ticks.
#define MICROSECONDS_PER_TIMER0_OVERFLOW (clockCyclesToMicroseconds(64 * 256))

// the whole number of milliseconds per timer0 overflow
#define MILLIS_INC (MICROSECONDS_PER_TIMER0_OVERFLOW / 1000)

// the fractional number of milliseconds per timer0 overflow. we shift right
// by three to fit these numbers into a byte. (for the clock speeds we care
// about - 8 and 16 MHz - this doesn't lose precision.)
#define FRACT_INC ((MICROSECONDS_PER_TIMER0_OVERFLOW % 1000) >> 3)
#define FRACT_MAX (1000 >> 3)

volatile unsigned long timer0_overflow_count = 0;
volatile unsigned long timer0_millis = 0;
static unsigned char timer0_fract = 0;


#if defined(__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__)
ISR(TIM0_OVF_vect)
#else
ISR(TIMER0_OVF_vect)
#endif
{
        // copy these to local variables so they can be stored in registers
        // (volatile variables must be read from memory on every access)
        unsigned long m = timer0_millis;
        unsigned char f = timer0_fract;

        m += MILLIS_INC;
        f += FRACT_INC;
        if (f >= FRACT_MAX) {
                f -= FRACT_MAX;
                m += 1;
        }

        timer0_fract = f;
        timer0_millis = m;
        timer0_overflow_count++;
}

unsigned long millis()
{
        unsigned long m;
        uint8_t oldSREG = SREG;

        // disable interrupts while we read timer0_millis or we might get an
        // inconsistent value (e.g. in the middle of a write to timer0_millis)
        cli();
        m = timer0_millis;
        SREG = oldSREG;

        return m;
}