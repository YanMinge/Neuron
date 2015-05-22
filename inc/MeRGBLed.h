#ifndef __MERGBLED_H__
#define __MERGBLED_H__

#include "M051Series.h"
#include "mygpio.h"

#define TRUE  1
#define FALSE 0
#define MAX_RGB_LED_NUM   30

/*---------------------------------------------------------------------------------------------------------*/
/* GPIO config: should change the value in rgb_init()                                                      */
/*---------------------------------------------------------------------------------------------------------*/

#if !defined(__bool_true_false_are_defined)
typedef unsigned char           bool; //!< Boolean.
#endif

extern uint8_t rgb_led_num;
extern uint8_t pixels[MAX_RGB_LED_NUM*3];
extern volatile uint32_t* di_gpio_addr;

struct cRGB
{ 
  uint8_t g; 
  uint8_t r; 
  uint8_t b; 
};

void rgb_init(uint8_t num_leds,uint8_t din_pin);
void rgb_clear(void);
void h_dat0(void);
void h_dat1(void);
void setNumber(uint8_t num_leds);
void send_byte(uint8_t var);
void led_reset(void);
void rgb_show(void);
bool setColorIndex(uint8_t index, uint8_t red,uint8_t green,uint8_t blue);
bool setColorAll(uint8_t red,uint8_t green,uint8_t blue);
#endif
