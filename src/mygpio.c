#include "mygpio.h"

void pinMode(uint8_t pin,uint8_t mode)
{
    uint8_t port,pin_num;
    port = (pin&0xf0) >> 4;
    pin_num =  1 << (pin&0x0f);
    switch(port)
    {
        case 0:
            PIN_Mode(0,pin_num,mode);
            break;
        case 1:
            PIN_Mode(1,pin_num,mode);
            break;
        case 2:
            PIN_Mode(2,pin_num,mode);
            break;
        case 3:
            PIN_Mode(3,pin_num,mode);
            break;
        case 4:
            PIN_Mode(3,pin_num,mode);
            break;
        default:
            break;
    }				
}

uint32_t* Pin2Addr(uint8_t pin)
{
    uint32_t *addr;
    uint8_t port, pin_num;
    port = (pin&0xf0) >> 4;
    pin_num =  pin&0x0f;
    addr = (uint32_t *)((GPIO_PIN_DATA_BASE+(0x20*(port))) + ((pin_num)<<2));
    return addr;
}

void digitalWrite(uint8_t pin,int val)
{
    *(Pin2Addr(pin)) = val;
}

int digitalRead(uint8_t pin)
{
    uint8_t port, pin_num;
    port = (pin&0xf0) >> 4;
    pin_num =  pin&0x0f;
    return GPIO_PIN_ADDR(port,pin_num);
}
//int analogRead(uint8_t pin)
//{
//	
//}
//void analogWrite(uint8_t pin, int val)
//{

//}
