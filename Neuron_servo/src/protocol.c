#include "protocol.h"
#include "uart_printf.h"
#include <stdio.h>
#include <stdarg.h>
#include "smartservo.h"
#include "MePwm.h"

uint8_t Uart0SendData[UART0_SEND_BUF_SIZE]={0};
uint8_t Uart1RecData[UART1_REV_BUF_SIZE]={0};
uint8_t Uart1SendData[DEFAULT_UART_BUF_SIZE]={0};

volatile boolean parsingSysex = false;
volatile boolean parsingSysexComplete = true;
volatile boolean isLocalOutput = false;
volatile boolean isForwardOutput = false;
volatile int sysexBytesRead = 0 ;
volatile int ForwardBytesRead = 0;
volatile int Uart0SendBytes = 0;
volatile int Uart1Revhead  = 0;
volatile int Uart1RevRtail  = 0;
volatile int Uart0Sendhead  = 0;
volatile int Uart0SendRtail  = 0;

uint8_t device_id = 0;    //hardware ID, Uniqueness in a link.
uint8_t device_type = CUSTOM_TYPE;  //device type, Indicates the type of module, 0x00 indicates no external module

union sysex_message sysex = {0};

static volatile uint16_t monochrome_led_dis = 0x00;
static volatile uint8_t led_loop_counter = 0;

//Sensor value
volatile int report_mode = 0;

volatile int temperature_val = 270;
volatile int pre_temperature_val = 270;

volatile int light_sensor_val = 0;
volatile int pre_light_sensor_val = 0;

//^^^^^^^^^^^^^^^^^^^^^^^ edited by Rafael
volatile int variable_resistor_val = 0;
volatile int pre_variable_resistor_val = 0;
//vvvvvvvvvvvvvvvvvvvvvvv edited by Rafael

volatile int mic_sensor_val = 0;
volatile int pre_mic_sensor_val = 0;

volatile int joystick_val[2] = {0};
volatile int pre_joystick_val[2] = {0};

volatile boolean limit_switch_val = 0;
volatile boolean pre_limit_switch_val = 0;

volatile boolean button_key_val = 0;
volatile boolean pre_button_key_val = 0;

volatile unsigned char ir_receive_data = 0;
volatile unsigned char pre_ir_receive_data = 0;
volatile unsigned char ir_receive_devID = 0;
volatile unsigned char pre_ir_receive_devID = 0;

volatile int smart_servo_pos_val = 0;
volatile int pre_smart_servo_pos_val = 0;

union{
  uint8_t byteVal[8];
  double doubleVal;
}val8byte;

union{
    uint8_t byteVal[4];
    float floatVal;
    long longVal;
}val4byte;

union{
  uint8_t byteVal[2];
  short shortVal;
}val2byte;

union{
  uint8_t byteVal[1];
  uint8_t charVal;
}val1byte;

uint8_t readbyte(uint8_t *argv,int idx)
{
    uint8_t temp;
    val1byte.byteVal[0] = argv[idx] & 0x7f;
    temp = argv[idx+1] << 7;
    val1byte.byteVal[0] |= temp;
    return val1byte.charVal;
}

short readShort(uint8_t *argv,int idx,boolean ignore_high)
{
    uint8_t temp;
    val2byte.byteVal[0] = argv[idx] & 0x7f;
    temp = argv[idx+1] << 7;
    val2byte.byteVal[0] |= temp;

    val2byte.byteVal[1] = (argv[idx+1] >> 1) & 0x7f;

    //Send analog can ignored high
    if(ignore_high == false)
    {
        temp = (argv[idx+2] << 6);
        val2byte.byteVal[1] |= temp;
    }

    return val2byte.shortVal;
}

float readFloat(uint8_t *argv,int idx){
    uint8_t temp;
    val4byte.byteVal[0] = argv[idx] & 0x7f;
    temp = argv[idx+1] << 7;
    val4byte.byteVal[0] |= temp;

    val4byte.byteVal[1] =  (argv[idx+1] >> 1) & 0x7f;
    temp = (argv[idx+2] << 6);
    val4byte.byteVal[1] += temp;

    val4byte.byteVal[2] =  (argv[idx+2] >> 2) & 0x7f;
    temp = (argv[idx+3] << 5);
    val4byte.byteVal[2] += temp;

    val4byte.byteVal[3] =  (argv[idx+3] >> 3) & 0x7f;
    temp = (argv[idx+4] << 4);
    val4byte.byteVal[3] += temp;

    return val4byte.floatVal;
}

void sendbyte(uint8_t val)
{
    uint8_t val_7bit[2]={0};
    val1byte.charVal = val;
    val_7bit[0] = val1byte.byteVal[0] & 0x7f;
    write_byte_uart0(val_7bit[0]);
    val_7bit[1] = (val1byte.byteVal[0] >> 7) & 0x7f;
    write_byte_uart0(val_7bit[1]);
}

void sendShort(int val,boolean ignore_high)
{
    uint8_t val_7bit[3]={0};
    val2byte.shortVal = val;
    val_7bit[0] = val2byte.byteVal[0] & 0x7f;
    write_byte_uart0(val_7bit[0]);
    val_7bit[1] = ((val2byte.byteVal[1] << 1) | (val2byte.byteVal[0] >> 7)) & 0x7f;
    write_byte_uart0(val_7bit[1]);

    //Send analog can ignored high
    if(ignore_high == false)
    {
        val_7bit[2] = (val2byte.byteVal[1] >> 6) & 0x7f;
        write_byte_uart0(val_7bit[2]);
    }
}

void sendFloat(float val)
{
    uint8_t val_7bit[5]={0};
    val4byte.floatVal = val;
    val_7bit[0] = val4byte.byteVal[0] & 0x7f;
    write_byte_uart0(val_7bit[0]);
    val_7bit[1] = ((val4byte.byteVal[1] << 1) | (val4byte.byteVal[0] >> 7)) & 0x7f;
    write_byte_uart0(val_7bit[1]);
    val_7bit[2] = ((val4byte.byteVal[2] << 2) | (val4byte.byteVal[1] >> 6)) & 0x7f;
    write_byte_uart0(val_7bit[2]);
    val_7bit[3] = ((val4byte.byteVal[3] << 3) | (val4byte.byteVal[2] >> 5)) & 0x7f;
    write_byte_uart0(val_7bit[3]);
    val_7bit[4] = (val4byte.byteVal[3] >> 4) & 0x7f;
    write_byte_uart0(val_7bit[4]);
}

static void assign_dev_id_response(void *arg)
{
    //The arg from value[0]
    device_id = sysex.val.value[0] +1;
    //response mesaage to UART0
    write_byte_uart0(START_SYSEX);
    write_byte_uart0(device_id);
    write_byte_uart0(CTL_ASSIGN_DEV_ID);
    write_byte_uart0(device_type);
    write_byte_uart0(END_SYSEX);

    //forward mesaage to UART1
    write_byte_uart1(START_SYSEX);
    write_byte_uart1(ALL_DEVICE);
    write_byte_uart1(CTL_ASSIGN_DEV_ID);
    write_byte_uart1(device_id);
    write_byte_uart1(END_SYSEX);
}

static void system_reset_response(void *arg)
{

}

static void write_digital_response(void *arg)
{

}

static void write_analog_response(void *arg)
{

}

static void report_digital_response(void *arg)
{

}

static void report_analog_response(void *arg)
{

}

static void set_pin_mode_response(void *arg)
{

}

static void read_digital_response(void *arg)
{
    uint8_t pin,value;
    pin = sysex.val.value[0];
    value = digitalRead(pin);

    //response mesaage to UART0
    write_byte_uart0(START_SYSEX);
    write_byte_uart0(device_id);
    write_byte_uart0(CTL_READ_DIGITAL);
    write_byte_uart0(value);
    write_byte_uart0(END_SYSEX);
}

static void read_analog_response(void *arg)
{
    uint8_t pin;
    int value;
    pin = sysex.val.value[0];
    value = analogRead(pin);

    //response mesaage to UART0
    write_byte_uart0(START_SYSEX);
    write_byte_uart0(device_id);
    write_byte_uart0(CTL_READ_ANALOG);
    sendShort(value,true);
    write_byte_uart0(END_SYSEX);
}

static void pwm_rgb_led_response(void *arg)
{
    uint8_t r_val,g_val,b_val;
    r_val = readbyte(sysex.val.value,0);
    g_val = readbyte(sysex.val.value,2);
    b_val = readbyte(sysex.val.value,4);
    pwm_write(P2_5,r_val,0,255);
    pwm_write(P2_4,g_val,0,255);
    pwm_write(P2_3,b_val,0,255);
}

static void ws2812_display_response(void *arg)
{

}

static void monochrome_led_response(void *arg)
{

}

static void segment_dis_response(void *arg)
{

}

static void infrared_send_response(void *arg)
{

}

static void motor_dc_response(void *arg)
{

}

static void motor_step_response(void *arg)
{

}

static void motor_encoder_response(void *arg)
{

}
static void motor_servo_response(void *arg)
{

}

static void voice_buzzer_tone_response(void *arg)
{

}

static void laser_device_response(void *arg)
{

}

static void send_temperature_18b20_value(int value)
{

}

static void temperature_18b20_response(void *arg)
{

}

static void mpu6050_response(void *arg)
{

}

static void send_digital_signal_value(uint8_t srv_id_val,boolean value)
{
    //response mesaage to UART0
    write_byte_uart0(START_SYSEX);
    write_byte_uart0(device_id);
    write_byte_uart0(srv_id_val);
    write_byte_uart0(value);
    write_byte_uart0(END_SYSEX);
}

static void limit_switch_response(void *arg)
{

}

static void send_joystick_value(int x_value, int y_value)
{

}

static void joystick_response(void *arg)
{

}

static void send_infrared_receiver_value(unsigned char devID,unsigned char value)
{
    //response mesaage to UART0
    write_byte_uart0(START_SYSEX);
    write_byte_uart0(device_id);
    write_byte_uart0(INFRARED_RECEIVER);
    sendbyte(devID);
    sendbyte(value);
    write_byte_uart0(END_SYSEX);
}

static void infrared_receiver_response(void *arg)
{
    report_mode = sysex.val.value[0];
    send_infrared_receiver_value(ir_receive_devID,ir_receive_data);
}


static void send_analog_signal_value(uint8_t srv_id_val,int value)
{
    //response mesaage to UART0
    write_byte_uart0(START_SYSEX);
    write_byte_uart0(device_id);
    write_byte_uart0(srv_id_val);
    sendShort(value,true);
    write_byte_uart0(END_SYSEX);
}

static void mic_sensor_response(void *arg)
{
    report_mode = sysex.val.value[0];
    send_analog_signal_value(MIC_SENSOR,mic_sensor_val);
}

static void light_sensor_response(void *arg)
{
    report_mode = sysex.val.value[0];
    send_analog_signal_value(LIGHT_SENSOR,light_sensor_val);
}

static void rtc_clock_response(void *arg)
{

}

static void button_key_1_response(void *arg)
{
    report_mode = sysex.val.value[0];
    send_digital_signal_value(BUTTON_KEY_1,button_key_val);
}

static void send_smart_pos(short pos_value)
{
  //response mesaage to UART0
	write_byte_uart0(START_SYSEX);
	write_byte_uart0(device_id);
	write_byte_uart0(SMART_SERVO);
	write_byte_uart0(GET_SERVO_CUR_POS);
	sendShort(pos_value,false);
	write_byte_uart0(END_SYSEX);
}

static void smart_servo_cmd_process(void *arg)
{
    short pos_value;
    short speed;
    uint8_t cmd;
    cmd = sysex.val.value[0];
    switch(cmd)
    {
        case SET_SERVO__PID:
            break;
        case SET_SERVO_ABSOLUTE_POS:
        {
            pos_value = readShort(sysex.val.value,1,false);
            if(sysexBytesRead > 5)
            {
                speed = readShort(sysex.val.value,4,false);
            }
            else
            {
                speed = SMART_SERVO_MAX_OUTPUT;
            }
            smart_servo_move_to(pos_value,speed);
            SendErrorUart0(PROCESS_SUC);
            break;
        }
        case SET_SERVO_RELATIVE_POS:
        {
            pos_value = readShort(sysex.val.value,1,false);
            if(sysexBytesRead > 5)
            {
                speed = readShort(sysex.val.value,4,false);
            }
            else
            {
                speed = SMART_SERVO_MAX_OUTPUT;
            }
            smart_servo_move(pos_value,speed);
            SendErrorUart0(PROCESS_SUC);
            break;
        }
        case GET_SERVO_STATUS:
            break;
        case GET_SERVO_PID:
            break;
        case GET_SERVO_CUR_POS:
        {
            report_mode = sysex.val.value[1];
            pos_value = adc_get_position_value();
            send_smart_pos(pos_value);
            break;
        }
        case GET_SERVO_SPEED:
        case GET_SERVO_TEMPERATURE:
        case GET_SERVO_ELECTRIC_CURRENT:
        case GET_SERVO_VOLTAGE:
            break;				
        default:
            SendErrorUart0(PROCESS_ERROR);
            break;				
    }
}

const Cmd_list_tab_type cmd_list_tab[]={
{CTL_ASSIGN_DEV_ID,NULL,assign_dev_id_response},
{CTL_SYSTEM_RESET,NULL,system_reset_response},
{CTL_DIGITAL_MESSAGE,NULL,write_digital_response},
{CTL_ANALOG_MESSAGE,NULL,write_analog_response},
{CTL_REPORT_DIGITAL,NULL,report_digital_response},
{CTL_REPORT_ANALOG,NULL,report_analog_response},
{CTL_SET_PIN_MODE,NULL,set_pin_mode_response},
{CTL_READ_DIGITAL,NULL,read_digital_response},
{CTL_READ_ANALOG,NULL,read_analog_response},
{DIS_RGB_LED_PWM,NULL,pwm_rgb_led_response},
{DIS_RGB_LED_WS2812,NULL,ws2812_display_response},
{DIS_MONOCHROME_LED,NULL,monochrome_led_response},
{DIS_SEGMENT_DIS,NULL,segment_dis_response},
{INFRARED_SEND,NULL,infrared_send_response},
{MOTOR_DC,NULL,motor_dc_response},
{MOTOR_STEP,NULL,motor_step_response},
{MOTOR_ENCODER,NULL,motor_encoder_response},
{MOTOR_SERVO,NULL,motor_servo_response},
{VOICE_BUZZER_TONE,NULL,voice_buzzer_tone_response},
{LASER_DEVICE,NULL,laser_device_response},
{TEMPERATURE_18B20,NULL,temperature_18b20_response},
{MPU6050,NULL,mpu6050_response},
{LIMITSWITCH,NULL,limit_switch_response},
{JOYSTICK,NULL,joystick_response},
{INFRARED_RECEIVER,NULL,infrared_receiver_response},
{MIC_SENSOR,NULL,mic_sensor_response},
{LIGHT_SENSOR,NULL,light_sensor_response},
{RTC_CLOCK,NULL,rtc_clock_response},
{BUTTON_KEY_1,NULL,button_key_1_response},
{SMART_SERVO,NULL,smart_servo_cmd_process},
};

#define CMD_LIST_TAB_ARRAY_SIZE sizeof(cmd_list_tab)/ sizeof(cmd_list_tab[0])
/*******************************************************************************
* Function Name  : SendErroruart0
* Description    : .....
* Input          :
* Output         : None
* Return         : None
* Attention      : None
*******************************************************************************/
void SendErrorUart0(uint8_t errorcode)
{
    write_byte_uart0(START_SYSEX);
    write_byte_uart0(device_id);
    write_byte_uart0(CTL_ERROR_CODE);
    write_byte_uart0(errorcode);
    write_byte_uart0(END_SYSEX);
}

/*******************************************************************************
* Function Name  : processSysexMessage
* Description    : .....
* Input          : None
* Output         : None
* Return         : None
* Attention      : None
*******************************************************************************/
void processSysexMessage(void)
{
    uint8_t i;
    parsingSysexComplete = false;
    if(sysex.val.dev_id == ALL_DEVICE || sysex.val.dev_id == device_id)
    {
        //CTL_ASSIGN_DEV_ID should processed one by one
        if((sysex.val.dev_id == ALL_DEVICE) &&
           (sysex.val.srv_id != CTL_ASSIGN_DEV_ID))
        {
            write_byte_uart1(START_SYSEX);
            flush_uart0_forward_buffer();
            write_byte_uart1(END_SYSEX);
        }
        for(i=0;i<CMD_LIST_TAB_ARRAY_SIZE;i++)
        {
             if(cmd_list_tab[i].service_id == sysex.val.srv_id)
             {
                 if((device_type != CUSTOM_TYPE) &&
                    (sysex.val.srv_id != device_type) &&
                    ((sysex.val.srv_id >> 4) != 1))
                 {
                     SendErrorUart0(WRONG_TYPE_OF_SERVICE);
                     return;
                 }
                 else if(cmd_list_tab[i].response_fun != NULL)
                 {
                     cmd_list_tab[i].response_fun((void*)NULL);
                 }
                 break;
             }
        }
    }
    else
    {
        write_byte_uart1(START_SYSEX);
        flush_uart0_forward_buffer();
        write_byte_uart1(END_SYSEX);
    }
    parsingSysexComplete = true;
}

/*******************************************************************************
* Function Name  : flush_uart0_forward_buffer
* Description    : .....
* Input          : None
* Output         : None
* Return         : None
* Attention      : None
*******************************************************************************/
void flush_uart0_forward_buffer(void)
{
    uint8_t i;
    for(i =0;i<sysexBytesRead;i++)
    {
        UART_WRITE(UART1, sysex.storedInputData[i]);
        UART_WAIT_TX_EMPTY(UART0);
    }
    sysexBytesRead = 0;
}

/*******************************************************************************
* Function Name  : flush_uart1_forward_buffer
* Description    : flush the uart1 forward data on UART0
* Input          : ....
* Output         : None
* Return         : None
* Attention      : None
*******************************************************************************/
void flush_uart1_forward_buffer(void)
{
    uint8_t inputData = 0xFF;
    uint16_t tmp;
    tmp = Uart1RevRtail;
    while(Uart1Revhead != tmp)
    {
        tmp = Uart1RevRtail;
        inputData = Uart1RecData[Uart1Revhead];
        if(inputData == START_SYSEX)
        {
            isForwardOutput = true;
        }
        UART_WRITE(UART0, inputData);
        UART_WAIT_TX_EMPTY(UART0);
        Uart1Revhead = (Uart1Revhead == (UART1_REV_BUF_SIZE - 1)) ? 0 : (Uart1Revhead + 1);
        ForwardBytesRead--;
        if(inputData == END_SYSEX)
        {
            isForwardOutput = false;
            break;
        }
    }
}

/*******************************************************************************
* Function Name  : flush_uart0_local_buffer
* Description    : flush the uart0 send buffer
* Input          : ....
* Output         : None
* Return         : None
* Attention      : None
*******************************************************************************/
void flush_uart0_local_buffer(void)
{
    uint8_t inputData = 0xFF;
    uint16_t tmp;
    tmp = Uart0SendRtail;
    while(Uart0Sendhead != tmp)
    {
        tmp = Uart0SendRtail;
        inputData = Uart0SendData[Uart0Sendhead];
        if(inputData == START_SYSEX)
        {
            isLocalOutput = true;
        }
        UART_WRITE(UART0, inputData);
        UART_WAIT_TX_EMPTY(UART0);
        Uart0Sendhead = (Uart0Sendhead == (UART0_SEND_BUF_SIZE - 1)) ? 0 : (Uart0Sendhead + 1);
        Uart0SendBytes--;
        if(inputData == END_SYSEX)
        {
            isLocalOutput = false;
            break;
        }
    }
}

/*******************************************************************************
* Function Name  : write_byte_uart0
* Description    : write a byte to the uart0
* Input          : ....
* Output         : None
* Return         : None
* Attention      : None
*******************************************************************************/
void write_byte_uart0(uint8_t inputData)
{
    if(Uart0SendBytes < UART0_SEND_BUF_SIZE)
    {
        /* Enqueue the character */
        Uart0SendData[Uart0SendRtail] = inputData;
        Uart0SendRtail = (Uart0SendRtail == (UART0_SEND_BUF_SIZE - 1)) ? 0 : (Uart0SendRtail + 1);
        Uart0SendBytes++;
    }
}

/*******************************************************************************
* Function Name  : write_byte_uart1
* Description    : write a byte to the uart1
* Input          : ....
* Output         : None
* Return         : None
* Attention      : None
*******************************************************************************/
void write_byte_uart1(uint8_t inputData)
{
    UART_WRITE(UART1, inputData);
    UART_WAIT_TX_EMPTY(UART1);
}

void check_digital_report(void)
{

}

void check_analog_report(void)
{

}

void send_sensor_report(void)
{
    if(report_mode != 0x00)
    {
        switch(device_type)
        {
            case TEMPERATURE_18B20:
            {
                if(((report_mode == 0x01) &&
                    (pre_temperature_val != temperature_val)) ||
                   (report_mode == 0x02))
                {
                    pre_temperature_val = temperature_val;
                    send_temperature_18b20_value(temperature_val);
                }
                break;
            }
            case MPU6050:
            case LIMITSWITCH:
            case JOYSTICK:
            {
                if(((report_mode == 0x01) &&
                    ((pre_joystick_val[0] != joystick_val[0]) ||
                     (pre_joystick_val[1] != joystick_val[1]))) ||
                   (report_mode == 0x02))
                {
                    pre_joystick_val[0] = joystick_val[0];
                    pre_joystick_val[1] = joystick_val[1];
                    send_joystick_value(joystick_val[0],joystick_val[1]);
                }
                break;
            }
            case INFRARED_RECEIVER:
            case MIC_SENSOR:
            case LIGHT_SENSOR:
            case RTC_CLOCK:
            case BUTTON_KEY_1:
                break;
            case SMART_SERVO:
            {
                if(((report_mode == 0x01) &&
                    (abs(pre_smart_servo_pos_val - smart_servo_pos_val) > 5)) ||
                   (report_mode == 0x02))
                {
                    pre_smart_servo_pos_val = smart_servo_pos_val;
                    send_smart_pos(smart_servo_pos_val);
                }
                break;
            }
            default:
                break;
        }
    }
}

void device_neep_loop(void)
{
    if(device_type == DIS_MONOCHROME_LED)
    {
        digitalWrite(P3_2,1);
        digitalWrite(P1_5,1);
        digitalWrite(P1_4,1);
        if(led_loop_counter == 0)
        {
            digitalWrite(P2_5,monochrome_led_dis & 0x01);
            digitalWrite(P2_4,(monochrome_led_dis >> 1) & 0x01);
            digitalWrite(P2_3,(monochrome_led_dis >> 2) & 0x01);
            digitalWrite(P3_2,0);
            digitalWrite(P1_5,1);
            digitalWrite(P1_4,1);
            led_loop_counter = 1;
        }
        else if(led_loop_counter == 1)
        {
            digitalWrite(P2_5,(monochrome_led_dis >> 3) & 0x01);
            digitalWrite(P2_4,(monochrome_led_dis >> 4) & 0x01);
            digitalWrite(P2_3,(monochrome_led_dis >> 5) & 0x01);
            digitalWrite(P1_5,0);
            digitalWrite(P3_2,1);
            digitalWrite(P1_4,1);
            led_loop_counter = 2;
        }
        else if(led_loop_counter == 2)
        {
            digitalWrite(P2_5,(monochrome_led_dis >> 6) & 0x01);
            digitalWrite(P2_4,(monochrome_led_dis >> 7) & 0x01);
            digitalWrite(P2_3,(monochrome_led_dis >> 8) & 0x01);
            digitalWrite(P1_4,0);
            digitalWrite(P3_2,1);
            digitalWrite(P1_5,1);
            led_loop_counter = 0;
        }
    }
}

void device_neep_loop_in_sampling(void)
{
  int16_t speed;
  if(device_type == SMART_SERVO)
  {
		if(smart_servo_target_pos != smart_servo_cur_pos)
    {
      speed = pid_position_to_pwm();
      smart_servo_speed_update(speed);
    }
  }
}


