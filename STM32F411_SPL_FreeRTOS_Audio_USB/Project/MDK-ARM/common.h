#ifndef _COMMON_
#define _COMMON_

#include "stm32f4xx.h"

#define INITIAL_2000MS 5
#define TIME_BUTTON_3000MS 3000

#define ON_BIT(X, POS) (X |= (1u << POS))
#define OFF_BIT(X, POS) (X &= ~(1u << POS))
#define TOGGLE_BIT(X, POS) (X ^= (1u << POS))
#define CLEAR_2BIT(X) (X &= ~3u);

#define BIT_0 0
#define BIT_1 1
#define BIT_2 2

#define LOW 0
#define HIGH 1

typedef enum{
	STATE_DISABLE,
	STATE_ENABLE
}EnableDisable_t;
extern EnableDisable_t State_Active;
#define STATE_ACTIVE(x) (StateActive = x)


typedef enum{
	ZERO = 0,
	START_STOP,
	SKIP,
	PAUSE,
	MAX
}State_Button_t;
extern State_Button_t State_Button;
#define STATE_BUTTON(x) (State_Button = x)


#endif /* _COMMON_ */
