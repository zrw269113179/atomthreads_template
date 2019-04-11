#ifndef __DRV_KEY_H__
#define __DRV_KEY_H__

#include "stm8l15x.h"
#define KEY_PORT    GPIOA
#define KEY_PIN     GPIO_Pin_4

#define KEY_NULL            0
#define KEY_PRESS           1
#define KEY_PRESS_CONFIG    2
#define KEY_RELEASE         3

#define KEY_PRESS_TICK      20
#define KEY_ENABLE_LEVEL    RESET

typedef struct drv_key
{
    BitStatus level;
    BitStatus last_level;
    unsigned char key_state:3;
    unsigned char key_tick;
} Key;


void key_init();
void key_loop();
void key_event();
#endif