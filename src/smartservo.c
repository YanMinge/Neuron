#include "smartservo.h"
#include "systime.h"
#include "uart_printf.h"

#define FILTER_SHIFT 3

int16_t smart_servo_pre_pos = 0;
volatile int16_t smart_servo_cur_pos = 0;
int16_t smart_servo_target_pos = 0;
int16_t smart_servo_pre_target_pos = 0;
int16_t smart_servo_speed = 0;
int16_t smart_servo_max_speed = 0;

static void smart_servo_ccw(uint8_t speed);

void samrt_servo_init(void)
{
  pinMode(SMART_SERVO_LED_R,GPIO_PMD_OUTPUT);
  pinMode(SMART_SERVO_LED_G,GPIO_PMD_OUTPUT);
  pinMode(SMART_SERVO_LED_B,GPIO_PMD_OUTPUT);   
  pinMode(SMART_SERVO_SLEEP,GPIO_PMD_OUTPUT);

  digitalWrite(SMART_SERVO_LED_R,0);
  digitalWrite(SMART_SERVO_LED_G,1);
  digitalWrite(SMART_SERVO_LED_B,0);
  digitalWrite(SMART_SERVO_SLEEP,1);
	
//  pinMode(SMART_SERVO_PW1,GPIO_PMD_OUTPUT);
//  pinMode(SMART_SERVO_PW2,GPIO_PMD_OUTPUT);
//  digitalWrite(SMART_SERVO_PW1,0);
//  digitalWrite(SMART_SERVO_PW2,0);
  pwm_init(SMART_SERVO_PW1,500);
  pwm_init(SMART_SERVO_PW2,500);

//  pinMode(SMART_SERVO_PW1,GPIO_PMD_OUTPUT);
//  pinMode(SMART_SERVO_PW2,GPIO_PMD_OUTPUT);
//  digitalWrite(SMART_SERVO_PW1,1);
//  digitalWrite(SMART_SERVO_PW2,1);
	
  smart_servo_cur_pos = adc_get_position_value();
  smart_servo_pre_pos = smart_servo_cur_pos;
	smart_servo_target_pos = smart_servo_cur_pos;
  smart_servo_speed = SMART_SERVO_PER_SPPED_MAX;
}

void smart_servo_stop(void)
{
  pwm_write(SMART_SERVO_PW1,255,0,255);
  pwm_write(SMART_SERVO_PW2,255,0,255);
}

static void smart_servo_ccw(uint8_t speed)
{
  pwm_write(SMART_SERVO_PW1,speed,0,255);
  pwm_write(SMART_SERVO_PW2,0,0,255);
}

static void smart_servo_cw(uint8_t speed)
{
  pwm_write(SMART_SERVO_PW1,0,0,255);
  pwm_write(SMART_SERVO_PW2,speed,0,255);
}

void smart_servo_speed_update(int16_t speed)
{
  int16_t speed_temp = speed;
  smart_servo_cur_pos = adc_get_position_value();

  if((smart_servo_cur_pos > (SMART_SERVO_MAX_LIM_POS + 200)) ||
     (smart_servo_cur_pos < (SMART_SERVO_MIN_LIM_POS - 200)))
  {
    speed_temp = (int16_t)((smart_servo_speed*255.0)/SMART_SERVO_PER_SPPED_MAX);
  }

  if((speed_temp == 0) || (abs(speed_temp) < 30))
  {
    smart_servo_stop();
  }
	else if(speed_temp > 0)
  {
    smart_servo_cw((uint8_t)speed_temp);
  }
  else
  {
    smart_servo_ccw((uint8_t)abs(speed_temp));
	}
}

void smart_servo_move_to(int16_t absolute_angle,int16_t speed)
{
  float speed_temp;
  smart_servo_target_pos = absolute_angle;
  speed_temp = (speed * SMART_SERVO_PER_SPPED_MAX)/255;
  smart_servo_speed = (int16_t)abs(speed_temp);
}

void smart_servo_move(int16_t angle,int16_t speed)
{
  smart_servo_move_to((smart_servo_cur_pos + angle),speed);
}

static int16_t normalize_position_difference(int16_t posdiff)
{
  if (posdiff > ((SMART_SERVO_MAX_POSITION - SMART_SERVO_MIN_POSITION) / 2))
  {
    posdiff -= (SMART_SERVO_MAX_POSITION - SMART_SERVO_MIN_POSITION);
  }

  if (posdiff < -((SMART_SERVO_MAX_POSITION - SMART_SERVO_MIN_POSITION) / 2))
  {
    posdiff += (SMART_SERVO_MAX_POSITION - SMART_SERVO_MIN_POSITION);
  }
  return posdiff;
}

static int32_t check_pwm_output_staturation(int32_t pwm_output)
{
  // Check for output saturation.
  if (pwm_output > (SMART_SERVO_MAX_OUTPUT * 256))
  {
    // Can't go higher than the maximum output value.
    pwm_output = (SMART_SERVO_MAX_OUTPUT * 256);
  }
  else if (pwm_output < (SMART_SERVO_MIN_OUTPUT * 256))
  {
    // Can't go lower than the minimum output value.
    pwm_output = (SMART_SERVO_MIN_OUTPUT * 256);
  }
	return pwm_output;
}

int16_t pid_position_to_pwm(void)
{
  float d_component_temp;
  int16_t smart_servo_cur_pos;
  static int16_t deadband;
  static int16_t p_component;
  static int16_t d_component;
  static int16_t minimum_position;
  static int16_t maximum_position;
  static int16_t current_velocity;
  static int16_t seek_velocity;
  static int32_t pwm_output;
  static float d_gain;
  static float p_gain;
  smart_servo_cur_pos = adc_get_position_value();
  current_velocity = normalize_position_difference(smart_servo_cur_pos - smart_servo_pre_pos);
  smart_servo_pre_pos = smart_servo_cur_pos;

  seek_velocity = smart_servo_speed;
  minimum_position = SMART_SERVO_MIN_LIM_POS;
  maximum_position = SMART_SERVO_MAX_LIM_POS;

  // Get the deadband.
  deadband = SMART_SERVO_DEADBAND;

  smart_servo_pre_target_pos = smart_servo_target_pos;

  // Keep the seek position bound within the minimum and maximum position.
  if (smart_servo_target_pos < minimum_position) 
  {
    smart_servo_target_pos = minimum_position;
  }
  if (smart_servo_target_pos > maximum_position)
  {
    smart_servo_target_pos = maximum_position;
  }
	
  // The proportional component to the PID is the position error.
  p_component = smart_servo_target_pos - smart_servo_cur_pos;

  if(p_component > 0)
  {
    d_component_temp = (seek_velocity - current_velocity)*(255.0/SMART_SERVO_PER_SPPED_MAX);
  }
  else
  {
    d_component_temp = ((-seek_velocity) - current_velocity)*(255.0/SMART_SERVO_PER_SPPED_MAX);
  }

  // The derivative component to the PID is the velocity.
  d_component = (int16_t)d_component_temp;

  if(abs(p_component) <= 4*deadband)
  {
    d_component = 0;
  }

  // Get the proportional, derivative and integral gains.
  p_gain = 300;
  d_gain = 256;

  // Start with zero PWM output.
  pwm_output = 0;
	
  // Apply proportional component to the PWM output if outside the deadband.
  if ((p_component > deadband) || (p_component < -deadband))
  {
    // Apply the proportional component of the PWM output.
    pwm_output += (int32_t) (p_component * p_gain);
    pwm_output = check_pwm_output_staturation(pwm_output);
		pwm_output = (pwm_output * seek_velocity)/SMART_SERVO_PER_SPPED_MAX;
    // Apply the derivative component of the PWM output.
    pwm_output += (int32_t) d_component * (int32_t) d_gain;
  }
  // Shift by 8 to account for the multiply by the 8:8 fixed point gain values.

  pwm_output = check_pwm_output_staturation(pwm_output);

  pwm_output = pwm_output/256;

  // Return the PID output.
  return (int16_t) pwm_output;
}

int16_t adc_get_position_value(void)
{
  int value = -1;
  value = analogRead(SMART_SERVO_DIR_AD);
  return value;
}

void servo_move_test(void)
{
  smart_servo_ccw(255);
}
