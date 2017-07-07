#ifndef __BUZZER_H__
#define __BUZZER_H__

#include <stdint.h>
#include <stdbool.h>

void buzzerInit(void);
bool buzzerTest();


struct buzzerControl
{
  void (*off)();
  void (*on)(uint32_t freq);
};



void buzzerOff();
void buzzerOn(uint32_t freq);
void buzzerSetControl(struct buzzerControl * bc);


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

#define SEARCHING_FOR_RIGHT_ASPERITY (13)
#define SEARCHING_FOR_LEFT_ASPERITY (14)
#define ENGAGE_RIGHT (15)
#define ENGAGE_LEFT (16)
#define PULL_RIGHT (17)
#define PULL_LEFT (18)
#define RESET_RIGHT_SEARCH (19)
#define RESET_LEFT_SEARCH (20)


#endif //__BUZZER_H__
