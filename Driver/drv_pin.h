#ifndef	__DRV_PIN_H__
#define __DRV_PIN_H__

typedef enum
{
  PIN_MODE_INPUT      = 0x00,
  PIN_MODE_INPUT_PULLUP,
  PIN_MODE_INPUT_IRQ,
  PIN_MODE_INPUT_PULLUP_IRQ,
  PIN_MODE_OUT_OD,
  PIN_MODE_OUT,
}PIN_MODE;

void pin_mode_set(unsigned char id,PIN_MODE type);
void pin_write(unsigned char id,unsigned char level);
unsigned char pin_read(unsigned char id);


#endif