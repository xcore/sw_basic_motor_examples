// Copyright (c) 2011, XMOS Ltd., All rights reserved
// This software is freely distributable under a derivative of the
// University of Illinois/NCSA Open Source License posted in
// LICENSE.txt and at <http://github.xcore.com/>

#define CMD_NUMBER_STEPS 0
#define CMD_SET_MOTOR_SPEED 1
#define CMD_MOTOR_OFF 2

#define FORWARD 0
#define REVERSE 1


/** \brief Send commands to the motor control thread
 *
 * Example user thread to send commands to the motor thread.
 * Valid commands are CMD_SET_MOTOR_SPEED, CMD_NUMBER_STEPS
 * or CMD_MOTOR_OFF.
 *
 *  \param c_control The motor control channel
 *
 */ 
void controller(chanend c_control);

/** \brief Recieve ADC values from ADC 7265
 *
 *  This implements the interface to the 7265 ADC device.  It takes a single channel to the 
 *  ADC.
 *
 *  \param c_adc the ADC server control channel
 *
 */ 
void getWindingADC ( chanend c_adc);


/** \brief Set the PWM duty cycles according to input
* Given PIOutput from the PI controller ( two values, one for each winding)
* transform into output duty cycles for each high side MOSFET (4)
* winding 0 on A, 2A ; winding 1 on B, C
* 
* duty consists of high ports x 4 followed by low ports x 4
*
* \param PIOutput[] the output from the PI controller
* \param decayMode[] enumeration of decay mode for each winding
* \param c_pwm channel for communication with the pwm server
*/
void setWindingPWM(int PIOutput[], enum decay decayMode[],  chanend c_pwm);

/** \brief Apply a PI controller to the reference and actual currents.
* Given a reference current input and actual winding current, returns
* the corresponding output. Also saves the sum, last error and last output.
*
* \param referenceI Reference current for particular coil
* \param actualI Current from ADC
* \param &sum
* \param &lastError
* \param &lastOutput
*/
int applyPI(int referenceI,int actualI, int &sum, int &lastError, int &lastOutput);

/** \brief Move the motor by a single step.
* Move motor by one step, this uses microsteps to provide positions between steps
* 
* The output is refined by using a cosine table to lookup individual PWM
* duty cycles corresponding to  each microstep. These duty cycles are then output to the FETs.
* 
*
* \param c_pwm Channel to PWM server
* \param &step Current step in sequence
* \param microPeriod The period for each microstep
* \param c_adc Channel to ADC Server
*/
void singleStep(chanend c_pwm, unsigned int &step, unsigned microPeriod, chanend c_adc);

/** \brief Main control loop for stepper motor control.
* 
* Starts up watchdog and calibrates ADC. The thread then pauses to allow the system
* to settle.
* The main functionality is implemented in a select statement. If a command is received 
* from the client thread then state variables are modified. A timer is also run. If one step
* period has elapsed then the singleStep function is called depending on state variables.
* 
* \param c_pwm Channel to PWM server
* \param c_control Channel to client thread
* \param c_wd Channel to watchdog server
* \param c_adc Channel to ADC server
*/
void motor(chanend c_pwm, chanend c_control, chanend c_wd, chanend c_adc);

