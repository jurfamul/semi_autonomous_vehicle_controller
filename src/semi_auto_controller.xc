/*
 * semi_auto_controller.xc
 *
 *  Created on: Nov 15, 2018
 *      Author: jurge_000
 */
//includes
#include <XS1.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <print.h>

//bit masks for control pins
//motor a (driver's side wheel if caster is in front)
#define AIN1_ON 0b0100      //clockwise control mask
#define AIN2_ON 0b0001      //counter clockwise control mask
//motor b (passenger's side wheel if caster is in front)
#define BIN1_ON 0b0010      //counterclockwise control mask
#define BIN2_ON 0b1000      //clockwise control mask
//both motors
#define ABIN1_ON 0b1100     //clockwise control mask
#define ABIN2_ON 0b0011     //counterclockwise control mask
//multi motor turn masks
#define ABIN3_ON 0b1001     //left turn mask
#define ABIN4_ON 0b0110     //right turn mask

//ports
out port oMotorPWMA = XS1_PORT_1P;      //driver's side wheel
out port oMotorPWMB = XS1_PORT_1I;      //passenger's side wheel
out port oMotorControl = XS1_PORT_4D;
out port oLED1 = XS1_PORT_1A;
out port oLED2 = XS1_PORT_1D;
out port oSTB = XS1_PORT_1O;
in port iEncoder = XS1_PORT_8B;
out port oWiFiRX = XS1_PORT_1F;
in port iWiFiTX = XS1_PORT_1H;

//defines
#define TICKS_PER_SEC (XS1_TIMER_HZ)
#define TICKS_PER_MS (XS1_TIMER_HZ/1000)
#define TICKS_PER_US (XS1_TIMER_HZ/1000000)
#define PIN_HIGH (1)
#define PIN_LOW (0)
#define PWM_FRAME_TICKS (TICKS_PER_MS)
#define LED_FREQUENCY (TICKS_PER_SEC / 8)
#define LINE_DELAY (TICKS_PER_SEC)
#define BAUD_RATE (9600)
#define BUFFER_SIZE (128)
#define SIGNALS_PER_EDGE (20)
#define SIGNALS_PER_TURN (20)

//message struct
typedef struct {
    char data[BUFFER_SIZE];
} message_t;

//multi motor duty_cycle struct
typedef struct {
    int left_duty_cycle;
    int right_duty_cycle;
}motor_cmd_t;

//method prototypes
void run_wifi_program();
void uart_transmit_byte(out port oPort, char value, unsigned int baudrate);
char uart_recieve_byte(in port iPort, unsigned int baudrate);
void uart_transmit_bytes(out port oPort, const char values[], unsigned int baudrate);
void uart_to_console_task(chanend trigger_chan, chanend motor_cmd, chanend encoder_chan);
void output_task(chanend trigger_chan);
void line(const char buffer[]);
void multi_motor_task(out port oLeftPWM, out port oRightPWM,
        out port oMotorControl, chanend in_motor_cmd_chan);
void encoder_task(in port iEncoder, out port oLED1, out port oLED2, chanend out_encoder_chan);

//main
int main()
{
    oWiFiRX <: PIN_HIGH;
    oSTB <: PIN_HIGH;
    chan trigger_chan;
    chan motor_cmd;
    chan encoder_chan;

    par {
        uart_to_console_task(trigger_chan, motor_cmd, encoder_chan);
        output_task(trigger_chan);
        multi_motor_task(oMotorPWMA, oMotorPWMB, oMotorControl, motor_cmd);
        encoder_task(iEncoder, oLED1, oLED2, encoder_chan);
    }
    return 0;
}

//method definitions
char uart_recieve_byte(in port iPort, unsigned int baudrate)
{
    unsigned int time;
    unsigned int bitRate = TICKS_PER_SEC / baudrate;
    char value;
    timer tmr;

    // wait for start bit
    iPort when pinseq(0) :> void;
    tmr :> time;
    time += bitRate / 2;

    // input data bits
    for (int i = 0; i < 8; i++)
    {
        time += bitRate;
        tmr when timerafter(time) :> void;
        iPort :> >> value;
    }

    //input stop bit
    time += bitRate;
    tmr when timerafter(time) :> void;
    iPort :> void;

    return value;
}

void uart_transmit_byte(out port oPort, char value, unsigned int baudrate)
{
    unsigned int time;
    unsigned int bitRate = TICKS_PER_SEC / baudrate;
    timer tmr;

    tmr :> time;

    // output start bit
    oPort <: 0;
    time += bitRate;
    tmr when timerafter(time) :> void;

    //output data bits
    for (int i = 0; i < 8; i++)
    {
        oPort <: >>value;
        time += bitRate;
        tmr when timerafter(time) :> void;
    }

    // output end bit
    oPort <: 1;
    time += bitRate;
    tmr when timerafter(time) :> void;

}

void uart_transmit_bytes(out port oPort, const char values[], unsigned int baudrate)
{
    unsigned int iter = 0;
    char temp = values[iter];

    while (temp != '\0')
    {
        uart_transmit_byte(oPort, temp, baudrate);
        temp = values[++iter];
    }
}

void uart_to_console_task(chanend trigger_chan, chanend out_motor_cmd, chanend in_encoder_chan)
{
    char buffer[BUFFER_SIZE];
    message_t trigger_message;
    motor_cmd_t oMotorCmd;
    unsigned int encoder_in = 0;
    unsigned int left_signals = 0;
    unsigned int right_signals = 0;
    unsigned int current_signal = 0;
    unsigned int iter = 0;
    timer tmr;
    unsigned int t;
    unsigned int current_time;
    unsigned int wait_to_stop;

    while(1)
    {
        buffer[iter++] = uart_recieve_byte(iWiFiTX, BAUD_RATE);
        if (buffer[iter - 1] == '\n' || buffer[iter - 1] == '\r' || iter == BUFFER_SIZE - 1)
        {
            buffer[iter - 1] = '\0';
            iter = 0;
            //printstrln(buffer);
            tmr :> t;
            t += TICKS_PER_MS * 100;
            tmr when timerafter(t) :> void;
            //printf("%d", buffer[0]);
        }

        if (strcmp("lua: cannot open init.lua", buffer) == 0)
        {
            strcpy(trigger_message.data, "run_wifi_setup");
            trigger_chan <: trigger_message;
            iter = 0;
            tmr :> t;
            t += TICKS_PER_MS * 100;
            tmr when timerafter(t) :> void;
        }
        else if (strcmp("w", buffer) == 0)
        {
            unsigned int current_signals = 0;
            strcpy(trigger_message.data, "command: full forward");
            trigger_chan <: trigger_message;
            oMotorCmd.left_duty_cycle = 100;
            oMotorCmd.right_duty_cycle = 75;
            out_motor_cmd <: oMotorCmd;
            while (current_signals < SIGNALS_PER_EDGE)
            {
                unsigned int current_signal = 0;
                in_encoder_chan :> current_signal;
                if (current_signal == 1)
                {
                    left_signals++;
                }
                else
                {
                    right_signals++;
                }
                current_signals++;
            }
            oMotorCmd.left_duty_cycle = 0;
            oMotorCmd.right_duty_cycle = 0;
            out_motor_cmd <: oMotorCmd;
            iter = 0;
        }
        else if (strcmp("F", buffer) == 0)
        {
            unsigned int current_signals = 0;
            strcpy(trigger_message.data, "command: half forward");
            trigger_chan <: trigger_message;
            oMotorCmd.left_duty_cycle = 50;
            oMotorCmd.right_duty_cycle = 25;
            out_motor_cmd <: oMotorCmd;
            while (current_signals <= SIGNALS_PER_EDGE)
            {
                unsigned current_signal = 0;
                in_encoder_chan :> current_signal;
                if (current_signal == 1)
                {
                    left_signals++;
                }
                else
                {
                    right_signals++;
                }
                current_signals++;
            }
            oMotorCmd.left_duty_cycle = 0;
            oMotorCmd.right_duty_cycle = 0;
            out_motor_cmd <: oMotorCmd;
            iter = 0;
        }
        else if (strcmp("f", buffer) == 0)
        {
            unsigned int current_signals = 0;
            strcpy(trigger_message.data, "command: semi-auto half forward");
            trigger_chan <: trigger_message;
            oMotorCmd.left_duty_cycle = 70;
            oMotorCmd.right_duty_cycle = 15;
            out_motor_cmd <: oMotorCmd;
            current_signal = 0;
            tmr :> current_time;
            tmr :> wait_to_stop;
            wait_to_stop += TICKS_PER_SEC;
            while (1)
            {
                select {
                case in_encoder_chan :> current_signal :
                    tmr :> current_time;
                    tmr :> wait_to_stop;
                    wait_to_stop += TICKS_PER_SEC;
                    break;
                default :
                    tmr :> current_time;
                    break;
                }

                if (current_time > wait_to_stop)
                {
                    oMotorCmd.left_duty_cycle = 0;
                    oMotorCmd.right_duty_cycle = 0;
                    out_motor_cmd <: oMotorCmd;
                    break;
                }
            }
            iter = 0;
        }
        else if (strcmp("s", buffer) == 0)
        {
            unsigned int current_signals = 0;
            strcpy(trigger_message.data, "command: full reverse");
            trigger_chan <: trigger_message;
            oMotorCmd.left_duty_cycle = -95;
            oMotorCmd.right_duty_cycle = -75;
            out_motor_cmd <: oMotorCmd;
            while (current_signals <= SIGNALS_PER_EDGE)
            {
                unsigned int current_signal = 0;
                in_encoder_chan :> current_signal;
                if (current_signal == 1)
                {
                    left_signals++;
                }
                else
                {
                    right_signals++;
                }
                current_signals++;
            }
            oMotorCmd.left_duty_cycle = 0;
            oMotorCmd.right_duty_cycle = 0;
            out_motor_cmd <: oMotorCmd;
            iter = 0;
        }
        else if (strcmp("R", buffer) == 0)
        {
            unsigned int current_signals = 0;
            strcpy(trigger_message.data, "command: half reverse");
            trigger_chan <: trigger_message;
            oMotorCmd.left_duty_cycle = -45;
            oMotorCmd.right_duty_cycle = -25;
            out_motor_cmd <: oMotorCmd;
            while (current_signals <= SIGNALS_PER_EDGE)
            {
                unsigned current_signal = 0;
                in_encoder_chan :> current_signal;
                if (current_signal == 1)
                {
                    left_signals++;
                }
                else
                {
                    right_signals++;
                }
                current_signals++;
            }
            oMotorCmd.left_duty_cycle = 0;
            oMotorCmd.right_duty_cycle = 0;
            out_motor_cmd <: oMotorCmd;
            iter = 0;
        }
        else if (strcmp("r", buffer) == 0)
        {
            unsigned int current_signals = 0;
            strcpy(trigger_message.data, "command: semi-auto half reverse");
            trigger_chan <: trigger_message;
            oMotorCmd.left_duty_cycle = -45;
            oMotorCmd.right_duty_cycle = -25;
            out_motor_cmd <: oMotorCmd;
            current_signal = 0;
            tmr :> current_time;
            tmr :> wait_to_stop;
            wait_to_stop += TICKS_PER_SEC;
            while (1)
            {
                select {
                case in_encoder_chan :> current_signal :
                    tmr :> current_time;
                    tmr :> wait_to_stop;
                    wait_to_stop += TICKS_PER_SEC;
                    break;
                default :
                    tmr :> current_time;
                    break;
                }

                if (current_time > wait_to_stop)
                {
                    oMotorCmd.left_duty_cycle = 0;
                    oMotorCmd.right_duty_cycle = 0;
                    out_motor_cmd <: oMotorCmd;
                    break;
                }
            }
            iter = 0;
        }
        else if (strcmp("<", buffer) == 0)
        {
            unsigned int current_signals = 0;
            strcpy(trigger_message.data, "command: turn 90 degrees left");
            trigger_chan <: trigger_message;
            oMotorCmd.left_duty_cycle = -40;
            oMotorCmd.right_duty_cycle = 30;
            out_motor_cmd <: oMotorCmd;
            while (current_signals < SIGNALS_PER_TURN)
            {
                unsigned current_signal = 0;
                in_encoder_chan :> current_signal;
                if (current_signal == 1)
                {
                    left_signals++;
                }
                else
                {
                    right_signals++;
                }
                current_signals++;
            }
            oMotorCmd.left_duty_cycle = 0;
            oMotorCmd.right_duty_cycle = 0;
            out_motor_cmd <: oMotorCmd;
            iter = 0;
        }
        else if (strcmp(",", buffer) == 0)
        {
            unsigned int current_signals = 0;
            strcpy(trigger_message.data, "command: semi-auto turn left");
            trigger_chan <: trigger_message;
            oMotorCmd.left_duty_cycle = -40;
            oMotorCmd.right_duty_cycle = 30;
            out_motor_cmd <: oMotorCmd;
            current_signal = 0;
            tmr :> current_time;
            tmr :> wait_to_stop;
            wait_to_stop += TICKS_PER_SEC;
            while (1)
            {
                select {
                case in_encoder_chan :> current_signal :
                    tmr :> current_time;
                    tmr :> wait_to_stop;
                    wait_to_stop += TICKS_PER_SEC;
                    break;
                default :
                    tmr :> current_time;
                    break;
                }

                if (current_time > wait_to_stop)
                {
                    oMotorCmd.left_duty_cycle = 0;
                    oMotorCmd.right_duty_cycle = 0;
                    out_motor_cmd <: oMotorCmd;
                    break;
                }
            }
            iter = 0;
        }
        else if (strcmp(">", buffer) == 0)
        {
            unsigned int current_signals = 0;
            strcpy(trigger_message.data, "command: turn 90 degrees right");
            trigger_chan <: trigger_message;
            oMotorCmd.left_duty_cycle = 40;
            oMotorCmd.right_duty_cycle = -10;
            out_motor_cmd <: oMotorCmd;
            while (current_signals < SIGNALS_PER_TURN)
            {
                unsigned current_signal = 0;
                in_encoder_chan :> current_signal;
                if (current_signal == 1)
                {
                    left_signals++;
                }
                else
                {
                    right_signals++;
                }
                current_signals++;
            }
            oMotorCmd.left_duty_cycle = 0;
            oMotorCmd.right_duty_cycle = 0;
            out_motor_cmd <: oMotorCmd;
            iter = 0;
        }
        else if (strcmp(".", buffer) == 0)
        {
            unsigned int current_signals = 0;
            strcpy(trigger_message.data, "command: semi_auto turn right");
            trigger_chan <: trigger_message;
            oMotorCmd.left_duty_cycle = 60;
            oMotorCmd.right_duty_cycle = -10;
            out_motor_cmd <: oMotorCmd;
            current_signal = 0;
            tmr :> current_time;
            tmr :> wait_to_stop;
            wait_to_stop += TICKS_PER_SEC;
            while (1)
            {
                select {
                case in_encoder_chan :> current_signal :
                    tmr :> current_time;
                    tmr :> wait_to_stop;
                    wait_to_stop += TICKS_PER_SEC;
                    break;
                default :
                    tmr :> current_time;
                    break;
                }

                if (current_time > wait_to_stop)
                {
                    oMotorCmd.left_duty_cycle = 0;
                    oMotorCmd.right_duty_cycle = 0;
                    out_motor_cmd <: oMotorCmd;
                    break;
                }
            }
            iter = 0;
        }
        else if (strcmp("?", buffer) == 0)
        {
            char output_message[128];
            snprintf(output_message, 128, "left: %u, right: %u", left_signals, right_signals);
            strcpy(trigger_message.data, output_message);
            trigger_chan <: trigger_message;
            left_signals = 0;
            right_signals = 0;
            iter = 0;
        }
    }
}

void line(const char buffer[])
{
    unsigned int t;
    timer tmr;
    tmr :> t;
    t += LINE_DELAY /4;
    tmr when timerafter(t) :> void;
    uart_transmit_bytes(oWiFiRX, buffer, BAUD_RATE);
    t += LINE_DELAY / 4;
    tmr when timerafter(t) :> void;
    uart_transmit_bytes(oWiFiRX, "\r\n", BAUD_RATE);
}

void run_wifi_program()
{
    line("dofile('wifi.lua')");
}

void output_task(chanend trigger_chan)
{
    message_t trigger;

    while (1)
    {
        trigger_chan :> trigger;

        if (strcmp(trigger.data, "run_wifi_setup") == 0)
        {
            run_wifi_program();
        }
        else
        {
            line(trigger.data);
        }
    }
}

void multi_motor_task(out port oLeftPWM, out port oRightPWM,
        out port oMotorControl, chanend in_motor_cmd_chan)
{
    motor_cmd_t iMotorCmd;
    unsigned int cw_mask = ABIN1_ON;
    unsigned int lt_mask = ABIN3_ON;
    unsigned int rt_mask = ABIN4_ON;
    unsigned int ccw_mask = ABIN2_ON;
    unsigned int control_mask = cw_mask;
    int leftDutyCycle = 0;
    int rightDutyCycle = 0;
    unsigned int leftDutyCycleOut = 0;
    unsigned int rightDutyCycleOut = 0;
    unsigned int t;
    unsigned int rightT;
    unsigned int leftT;
    timer tmr;
    timer rtmr;
    timer ltmr;
    tmr :> t;
    rtmr :> rightT;
    ltmr :> leftT;

    oLeftPWM <: PIN_LOW;
    oRightPWM <:PIN_LOW;

    while (1)
    {
        int check = 1;
        oMotorControl <: control_mask;
        if (leftDutyCycleOut != 0)
        {
            oLeftPWM <: PIN_HIGH;
        }
        leftT += ((double)leftDutyCycleOut/100.0) * PWM_FRAME_TICKS;
        if (rightDutyCycleOut != 0)
        {
            oRightPWM <: PIN_HIGH;
        }
        rightT += ((double)rightDutyCycleOut/100.0) * PWM_FRAME_TICKS;
        t += PWM_FRAME_TICKS;

        while (check)
        {
            select
            {
             case in_motor_cmd_chan :> iMotorCmd :
                if (iMotorCmd.left_duty_cycle >= 0 && iMotorCmd.right_duty_cycle >= 0)
                {
                    control_mask = cw_mask;
                    leftDutyCycle = iMotorCmd.left_duty_cycle;
                    rightDutyCycle = iMotorCmd.right_duty_cycle;
                }
                else if (iMotorCmd.left_duty_cycle < 0 && iMotorCmd.right_duty_cycle >= 0)
                {
                    control_mask = lt_mask;
                    leftDutyCycle = iMotorCmd.left_duty_cycle;
                    rightDutyCycle = iMotorCmd.right_duty_cycle;
                }
                else if (iMotorCmd.left_duty_cycle >= 0 && iMotorCmd.right_duty_cycle < 0)
                {
                    control_mask = rt_mask;
                    leftDutyCycle = iMotorCmd.left_duty_cycle;
                    rightDutyCycle = iMotorCmd.right_duty_cycle;
                }
                else
                {
                    control_mask = ccw_mask;
                    leftDutyCycle = iMotorCmd.left_duty_cycle;
                    rightDutyCycle = iMotorCmd.right_duty_cycle;
                }
                leftDutyCycleOut = abs(leftDutyCycle);
                rightDutyCycleOut = abs(rightDutyCycle);
                if (leftDutyCycleOut > 100)
                {
                    leftDutyCycleOut = 100;
                }
                if (rightDutyCycleOut > 100)
                {
                    rightDutyCycleOut = 100;
                }
                break;
            case ltmr when timerafter(leftT) :> void :
                oLeftPWM <: PIN_LOW;
                leftT += PWM_FRAME_TICKS;
                break;
            case rtmr when timerafter(rightT) :> void :
                oRightPWM <: PIN_LOW;
                rightT += PWM_FRAME_TICKS;
                break;
            case tmr when timerafter(t) :> void :
                oLeftPWM <: PIN_LOW;
                oRightPWM <: PIN_LOW;
                check = 0;
                break;
            }
        }
    }
}

void encoder_task(in port iEncoder, out port oLED1, out port oLED2, chanend out_encoder_chan)
{
    char encoderInput;
    char SIG1Status;
    char SIG2Status;
    int SIG1Change = 1;
    int SIG2Change = 2;
    char SIG1Mask = 0B00000001;
    char SIG2Mask = 0B00000010;
    unsigned int t;
    timer tmr;
    iEncoder :> encoderInput;
    SIG1Status = encoderInput & SIG1Mask;
    SIG2Status = encoderInput & SIG2Mask;
    tmr :> t;

    while(1)
    {
        iEncoder when pinsneq(encoderInput) :> encoderInput;
        if ((encoderInput & SIG1Mask) != SIG1Status)
        {
            if ((encoderInput & SIG1Mask) == 1)
            {
                oLED1 <: PIN_HIGH;
            }
            else
            {
                oLED1 <: PIN_LOW;
            }
            out_encoder_chan <: SIG1Change;
        }
        if ((encoderInput & SIG2Mask) != SIG2Status)
        {
            if ((encoderInput & SIG2Mask) == 1)
            {
                oLED2 <: PIN_HIGH;
            }
            else
            {
                oLED2 <: PIN_LOW;
            }
            out_encoder_chan <: SIG2Change;
        }
    }
}


