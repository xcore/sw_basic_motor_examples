// Copyright (c) 2011, XMOS Ltd., All rights reserved
// This software is freely distributable under a derivative of the
// University of Illinois/NCSA Open Source License posted in
// LICENSE.txt and at <http://github.xcore.com/>

#define CMD_RAMP 0
#define CMD_SET_MOTOR_SPEED 1
#define CMD_GET_MOTOR_SPEED 2
#define CMD_POSITION 3
#define DC_MOTOR


/** \brief Structure holding ramp parameters
*
* \param startSpeed The speed at which to start the motor, given as period
* \param targetSpeed The speed at which to stop ramping, given as period
* \param direction The direction in which to turn the motor, either 1 or 0
* \param rampPeriod Period at which the speed will increase
* \param acceleration The speed increase per rampPeriod, if ramping down, set this negative
*/
typedef struct ramp_parameters {
    int startSpeed;
    int targetSpeed;
    int direction;
    int rampPeriod;       //Multiple of period for main loop i.e. if main period = 10ns and rampPeriod = 10, actual period for ramp is 100ns
    int acceleration;     //This is the speed increase per ramp_period, If ramping down, set this negative
} ramp_parameters;


/** \brief Thread to send commands to the motor control thread
* Example client function to send commands to the motor control thread 
* The following commands are available:
* CMD_RAMP
* CMD_SET_MOTOR_SPEED 
*
* \param c_control Channel to the motor control thread
*/
void DCcontroller (chanend c_control);


/** \brief Main control loop for DC motor
* 
*
* \param c_wd Channel to communicate with the watchdog thread
* \param c_speed[] Set of channels for communcation with display
* \param c_pwm Channel to send new duty cycles to PWM thread
* \param c_control Channel for receiving control commands
*/
void motors( chanend c_wd, chanend c_speed[], chanend c_pwm, chanend c_control);
