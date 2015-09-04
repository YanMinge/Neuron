#ifndef __SMARTSERVO_H__
#define __SMARTSERVO_H__

#include "M051Series.h"
#include "mygpio.h"
#include "MePwm.h"

typedef enum
{
  SMART_SERVO_DIR_CW = 0,    ///< Clockwise
  SMART_SERVO_DIR_CCW  = 1   ///< Counter-Clockwise
} smart_servo_dir;

//GPIO define
#define SMART_SERVO_LED_R     P2_4
#define SMART_SERVO_LED_G     P2_6
#define SMART_SERVO_LED_B     P2_5

#define SMART_SERVO_SLEEP     P0_6
#define SMART_SERVO_PW1       P2_2
#define SMART_SERVO_PW2       P2_3
#define SMART_SERVO_DIR_AD    P1_4


#define SMART_SERVO_MIN_POSITION  (0)
#define SMART_SERVO_MAX_POSITION  (4096)

#define SMART_SERVO_MIN_LIM_POS  (300)
#define SMART_SERVO_MAX_LIM_POS  (3700)

#define SMART_SERVO_PER_SPPED_MAX  (22)

// The minimum and maximum output.
#define SMART_SERVO_MAX_OUTPUT              (255)
#define SMART_SERVO_MIN_OUTPUT              (-SMART_SERVO_MAX_OUTPUT)
#define SMART_SERVO_DEADBAND                (10)


extern int16_t smart_servo_pre_pos;
volatile extern int16_t smart_servo_cur_pos;
extern int16_t smart_servo_target_pos;
extern int16_t smart_servo_pre_target_pos;
extern int16_t smart_servo_speed;

extern void samrt_servo_init(void);
extern void smart_servo_stop(void);
extern int16_t adc_get_position_value(void);
extern void smart_servo_move_to(int16_t absolute_angle,int16_t speed);
extern void smart_servo_move(int16_t angle,int16_t speed);
extern int16_t pid_position_to_pwm(void);
extern void smart_servo_speed_update(int16_t speed);
extern void servo_move_test(void);

#endif/* __SMARTSERVO_H__ */
