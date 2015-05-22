#include<stdlib.h>
#include<string.h>
#include "MeRGBLed.h"
#include "systime.h"

#ifdef PLL_CLOCK 
#define F_CPU                     PLL_CLOCK
#else
#define F_CPU                     50000000
#endif

uint8_t rgb_led_num = 0;
uint8_t pixels[MAX_RGB_LED_NUM*3];
volatile uint32_t* di_gpio_addr = NULL;

/*---------------------------------------------------------------------------------------------------------*/
/* GPIO config: should change the value in rgb_init()                                                      */
/*---------------------------------------------------------------------------------------------------------*/
// Timing in ns
#define t0h   350
#define t0l   800

#define t1h   700
#define t1l   600

// Fixed cycles used by the inner loop
#define w_fixedlow    16      //(MOVS+2LDR+STR+3BL)
#define w_fixedhigh   7       //(MOVS+2LDR+STR)

// Insert NOPs to match the timing, if possible
#define w_t0h           (((F_CPU/1000)*t0h          )/1000000)
#define w_t0l           (((F_CPU/1000)*t0l          )/1000000)

#define w_t1h           (((F_CPU/1000)*t1h          )/1000000)
#define w_t1l           (((F_CPU/1000)*t1l          )/1000000)

// w1 - nops between rising edge and falling edge - 0
#define w1 (w_t0h-w_fixedhigh)
// w2   nops between fe low and fe high
#define w2 (w_t0l-w_fixedlow)

// w3 - nops between rising edge and falling edge - 1
#define w3 (w_t1h-w_fixedhigh)
// w4   nops between fe low and fe high
#define w4 (w_t1l-w_fixedlow)


#if w1>0
  #define w0h_nops w1
#else
  #define w0h_nops  0
#endif

#if w2>0
#define w0l_nops w2
#else
#define w0l_nops  0
#endif

#if w3>0
  #define w1h_nops w3
#else
  #define w1h_nops  0
#endif

#if w4>0
#define w1l_nops w4
#else
#define w1l_nops  0
#endif

// The only critical timing parameter is the minimum pulse length of the "0"
// Warn or throw error if this timing can not be met with current F_CPU settings.
#define w_lowtime ((w0h_nops+w_fixedlow)*1000000)/(F_CPU/1000)

#if w_lowtime >550
   #error "Light_ws2812: Sorry, the clock speed is too low. Did you set F_CPU correctly?"
#elif w_lowtime>450
   #warning "Light_ws2812: The timing is critical and may only work on WS2812B, not on WS2812(S)."
   #warning "Please consider a higher clockspeed, if possible"
#endif   

#define w_nop1  __NOP();
#define w_nop2  __NOP();__NOP();
#define w_nop4  w_nop2 w_nop2
#define w_nop8  w_nop4 w_nop4
#define w_nop16 w_nop8 w_nop8
#define w_nop32 w_nop16 w_nop16

void rgb_init(uint8_t num_leds,uint8_t din_pin)
{
    pinMode(din_pin,GPIO_PMD_OUTPUT);
    setNumber(num_leds);
		di_gpio_addr = Pin2Addr(din_pin);
    rgb_clear();
}

void setNumber(uint8_t num_leds)
{
    rgb_led_num = num_leds;
}

bool setColorIndex(uint8_t index, uint8_t red,uint8_t green,uint8_t blue)
{
    if(index <= rgb_led_num)
    {
         uint8_t tmp = (index-1) * 3;
         pixels[tmp] = green;
         pixels[tmp+1] = red;
         pixels[tmp+2] = blue;
         return TRUE;
    } 
    return FALSE;
}

bool setColorAll(uint8_t red,uint8_t green,uint8_t blue)
{
    uint8_t i;
    for(i = 1;i <= rgb_led_num;i++)
    {
        setColorIndex(i,red,green,blue);
    }
    return TRUE;
}

bool setColor(uint8_t index,uint8_t red,uint8_t green,uint8_t blue)
{
    if(index == 0)
    {
        return setColorAll(red,green,blue);
		}
    else
    {
        return setColorIndex(index,red,green,blue);
    }
}

void h_dat0(void)
{ 
  *di_gpio_addr = 1;
#if (w0h_nops&1)
  w_nop1
#endif
#if (w0h_nops&2)
  w_nop2
#endif
#if (w0h_nops&4)
  w_nop4
#endif
#if (w0h_nops&8)
  w_nop8
#endif
#if (w0h_nops&16)
  w_nop16
#endif
#if (w0h_nops&32)
	w_nop32
#endif
  *di_gpio_addr = 0;
#if (w0l_nops&1)
  w_nop1
#endif
#if (w0l_nops&2)
  w_nop2
#endif
#if (w0l_nops&4)
  w_nop4
#endif
#if (w0l_nops&8)
  w_nop8
#endif
#if (w0l_nops&16)
  w_nop16 
#endif
#if (w0l_nops&32)
	w_nop32
#endif
}

void h_dat1(void)				  
{   
  *di_gpio_addr = 1;
#if (w1h_nops&1)
	w_nop1
#endif
#if (w1h_nops&2)
	w_nop2
#endif
#if (w1h_nops&4)
	w_nop4
#endif
#if (w1h_nops&8)
	w_nop8
#endif
#if (w1h_nops&16)
	w_nop16 
#endif
#if (w1h_nops&32)
	  w_nop32
#endif
  *di_gpio_addr = 0;
#if (w1l_nops&1)
  w_nop1
#endif
#if (w1l_nops&2)
  w_nop2
#endif
#if (w1l_nops&4)
  w_nop4
#endif
#if (w1l_nops&8)
  w_nop8
#endif
#if (w1l_nops&16)
  w_nop16
#endif
#if (w1l_nops&32)
	w_nop32
#endif
}

void led_reset(void)		 
{	
  *di_gpio_addr = 0;
  delayMicroseconds(100);
}

void send_byte(uint8_t var)
{
    uint8_t k;
    for(k=0;k<8;k++)
    {
        if(var&0x80)
        {
            h_dat1();
        }
        else
        {
            h_dat0();
        }
        var<<=1;
    }
}

void rgb_clear(void)
{
    uint8_t curbyte;
    for(curbyte = 0;curbyte < (3*rgb_led_num);curbyte++)
    {
        pixels[curbyte] = 0;
		}
    rgb_show();
}

void rgb_show(void)
{
    uint8_t curbyte = 0,b_index = 0;
    uint16_t datlen = 3*rgb_led_num;
	  __disable_irq();
    while (datlen--)
    {
        curbyte=pixels[b_index];
        send_byte(curbyte);
        b_index++;			
    }
		__enable_irq();
		led_reset();
}
