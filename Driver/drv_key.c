#include "HeaderInclude.h"
static Key      key1;

void key_init()
{
    GPIO_Init( KEY_PORT, KEY_PIN, GPIO_Mode_In_FL_No_IT);
}
void key_loop()
{
    key1.level = GPIO_ReadInputDataBit(KEY_PORT,KEY_PIN);
    if(key1.level == KEY_ENABLE_LEVEL)
    {
        if(key1.last_level != KEY_ENABLE_LEVEL)
        {
            key1.key_state = KEY_PRESS;
            key1.key_tick = 0;
        }
        else
        {
            key1.key_tick++;
            if(key1.key_tick > KEY_PRESS_TICK && key1.key_state == KEY_PRESS)
            {
                key1.key_state = KEY_PRESS_CONFIG;
                key_event();
            }
        }
    }
    else
    {
        if(key1.last_level == KEY_ENABLE_LEVEL)
        {
            key1.key_state = KEY_RELEASE;
        }
    }
    key1.last_level = key1.level;
    
}
void key_event()
{

}