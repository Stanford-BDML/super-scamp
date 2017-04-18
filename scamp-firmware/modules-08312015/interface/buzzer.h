#ifndef __BUZZER_H__
#define __BUZZER_H__

#include <stdint.h>
#include <stdbool.h>

void buzzerInit(void);

#define STAYING_NEUTRAL (0)
#define MOVING_HORIZONTALLY_IN (1)
#define SEARCHING_FOR_TOP_ASPERITY (2)
#define ENGAGING_TOP_FOOT (3)
#define PULLING_UP (4)
#define MOVING_HORIZONTALLY_OUT (5)
#define SEARCHING_FOR_BOTTOM_ASPERITY (6)
#define ENGAGING_BOTTOM_FOOT (7)
#define RECYCLING_TOP_FOOT (8)
// gap for the regular cycle to reset on
#define RESET_TOP_SEARCH (10)
// gap for top search to reset to moving in
#define RESET_BOTTOM_SEARCH (12)

#endif //__BUZZER_H__
