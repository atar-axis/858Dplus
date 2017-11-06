#ifndef INC_BUTTONS_H
#define INC_BUTTONS_H



// KEY STATE
//-----------------------------------------------
volatile uint8_t key_state;	// debounced and inverted key state: bit = 1: key pressed
volatile uint8_t key_press;	// key press detect
volatile uint8_t key_rpt;	// key long press and repeat

extern uint8_t get_key_press(uint8_t key_mask);
extern uint8_t get_key_rpt(uint8_t key_mask);
extern uint8_t get_key_state(uint8_t key_mask);
extern uint8_t get_key_short(uint8_t key_mask);
extern uint8_t get_key_long(uint8_t key_mask);
extern uint8_t get_key_long_r(uint8_t key_mask);
extern uint8_t get_key_rpt_l(uint8_t key_mask);
extern uint8_t get_key_common(uint8_t key_mask);
extern uint8_t get_key_common_l(uint8_t key_mask);


#endif // INC_BUTTONS_H
