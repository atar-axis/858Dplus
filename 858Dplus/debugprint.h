#ifndef INC_DEBUGPRINT_H
#define INC_DEBUGPRINT_H

#define DEBUGPRINT_LVL_NONE	0
#define DEBUGPRINT_LVL_FEW	1
#define DEBUGPRINT_LVL_MANY	2
#define DEBUGPRINT_LVL_ALL	3

extern int global_debug_print_level;

#ifdef DEBUG
#define USE_DEBUG_PRINTING 1
#else
#define USE_DEBUG_PRINTING 0
#endif

#define debug_print(lvl, msg) \
if(USE_DEBUG_PRINTING) if(lvl <= global_debug_print_level) Serial.println(msg)

#endif /* INC_DEBUGPRINT_H */