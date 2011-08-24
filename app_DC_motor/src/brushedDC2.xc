// Copyright (c) 2011, XMOS Ltd., All rights reserved
// This software is freely distributable under a derivative of the
// University of Illinois/NCSA Open Source License posted in
// LICENSE.txt and at <http://github.xcore.com/>

#include <print.h>
#include <platform.h>
#include <stdlib.h>
#include <syscall.h>
#include <xs1.h>
#include <stdio.h>
#include <xscope.h>
#include "watchdog.h"
#include "config.h"
#include "pwm_singlebit_port.h"
#include "adc.h"
#include "shared_io.h"

/*
* Connections as follows:
* Encoder A = Encoder A
* Encoder B = Encoder B
* +ve Motor = PH A
* -ve Motor = PH B
*/

//Defines for PWM library
//Period for PWM supposedly (Resolution * 20 * Timestep) in ns?
#define 	RESOLUTION	256
#define 	TIMESTEP	2
#define 	PERIOD		100000
#define 	NUM_PORTS 	2
#define 	NUM_PINS 	2
#define     NUM_MOTORS  2

//Ramp parameters
//#define RAMP_UP 1
#define 	RAMP_START 	50
#define 	RAMP_END 	300
#define 	RAMP_PERIOD 100000000   // 1s

//PID controller parameters
#define 	PID_PERIOD	100000   	// 1ms = 1kHz
#define 	ONE_SECOND	100000000 	// 1000ms
#define 	K_P        	5000
#define 	K_I        	10000

//PWM clock and Watchdog port
on stdcore[MOTOR_CORE] : clock pwm_clk = XS1_CLKBLK_1;
on stdcore[MOTOR_CORE] : out port i2c_wd = PORT_I2C_WD_SHARED;

//High sides of half-bridges
on stdcore[MOTOR_CORE] : out buffered port:32 motor_DC_hi[4] = {PORT_M1_HI_A, PORT_M1_HI_B, PORT_M2_HI_A, PORT_M2_HI_B};

//Low sides of half-bridges
on stdcore[MOTOR_CORE] : out port motor_DC_lo[4] = {PORT_M1_LO_A, PORT_M1_LO_B, PORT_M2_LO_A, PORT_M2_LO_B};

//Motor Encoders on Motor 1 and Motor 2
on stdcore[MOTOR_CORE] : in port encoder[2] = {PORT_M1_ENCODER, PORT_M2_ENCODER};

//ADC ports
on stdcore[MOTOR_CORE]: out port ADC_SCLK = PORT_ADC_CLK;
on stdcore[MOTOR_CORE]: buffered out port:32 ADC_CNVST = PORT_ADC_CONV;
on stdcore[MOTOR_CORE]: buffered in port:32 ADC_DATA = PORT_ADC_MISO;
on stdcore[MOTOR_CORE]: in port ADC_SYNC_PORT = XS1_PORT_16A;
on stdcore[MOTOR_CORE]: clock adc_clk = XS1_CLKBLK_2;

/* core with LCD and BUTTON interfaces */
on stdcore[INTERFACE_CORE]: lcd_interface_t lcd_ports = { PORT_DS_SCLK, PORT_DS_MOSI, PORT_DS_CS_N, PORT_CORE1_SHARED };
on stdcore[INTERFACE_CORE]: in port p_btns[4] = {PORT_BUTTON_A, PORT_BUTTON_B, PORT_BUTTON_C, PORT_BUTTON_D};





void client( chanend c_wd, chanend c_speed[], chanend c) {

    timer t, t_pid, t_ramp, t_speed, t_pwm, t_pwm2;
    unsigned rotor[2] = {0,0};
    int time, time_pid, time_speed, time_ramp, time_pwm[2], ts, lastA[2] = {0,0}, current_rpm[2] = {0,0}, rotations_old_speed[NUM_MOTORS], j, cmd;
    int speed_desired[NUM_MOTORS] = {40,40}, speed_current[NUM_MOTORS]={0,0}, speed_actual[NUM_MOTORS]={0,0}, speed_previous[NUM_MOTORS] = {0,0};
    int rotations_old[NUM_MOTORS] = {0,0}, pid_I[NUM_MOTORS] = {0,0}, pid_P[NUM_MOTORS] = {0,0}, error[NUM_MOTORS], motor_forward[NUM_MOTORS] = {0,0};
    int rotations[NUM_MOTORS] = {0,0}, duty[NUM_MOTORS] = {0,0},  motor_on_period[2] ={0,0}, motor_switch[2] = {0,0};
    unsigned int duties[4] = {0,0,0,0};

    //Wait 0.5s to ensure watchdog works correctly
    t :> ts;
    t when timerafter ( ts + 50000000 ) :> ts;
    c_wd <: WD_CMD_START;

    #ifdef RAMP_UP
    speed_desired = RAMP_START;
    #endif

    //Ensure all low sides are zero
    for (j=0; j<(NUM_MOTORS*2); j++)
        motor_DC_lo[j] <: 0;

    //Initialise timers
    t :> time;
    t_pid :> time_pid;
    t_ramp :> time_ramp;
    t_speed :> time_speed;
    time += PERIOD;
    time_pid += PID_PERIOD;
    time_speed += ONE_SECOND;

    xscope_config_io(XSCOPE_IO_BASIC);

	//Turn on low sides of M1 phase A and M2 phase A
	//TODO: Initial direction control
    for (j = 0; j < NUM_MOTORS; j++)    
        motor_DC_lo[((2*j)+1)] <: 1;
    
	while (1) {
        select {
            //Loop for updating PWM duty cycle
            case t when timerafter (time) :> void:
                for (j=0; j<NUM_MOTORS; j++) {
				    //If the duty cycle has gone negative, reverse current direction
                    if (duty[j] < 0) {
                    //TODO: Don't call the pwm routine twice
                        duties[0 + (2*j)] = 0;
                        pwmSingleBitPortSetDutyCycle(c, duties, (NUM_MOTORS*2));
                        motor_DC_lo[1 + (2*j)] <: 0;
                        motor_DC_lo[0 + (2*j)] <: 1;
                        duties[1+(2*j)] = -duty[j];
                    }
				//Otherwise set up for normal operation
                    else {
                        duties[0 + (2*j)] = 0;
                        pwmSingleBitPortSetDutyCycle(c, duties, (NUM_MOTORS*2));
                        motor_DC_lo[0+(2*j)] <: 0;
                        motor_DC_lo[1+(2*j)] <: 1; 
                        duties[0+(2*j)] = duty[j];
                    }
                }
                pwmSingleBitPortSetDutyCycle(c, duties, (NUM_MOTORS*2));
                time += PERIOD;
                break;

			//If ramping up the speed then increment desired speed every ramp period
            //TODO:Multi motors

            #ifdef RAMP_UP
            case t_ramp when timerafter(time_ramp) :> void:
                if (speed_desired[0] < RAMP_END) {
                    speed_desired[0]++;
                }
                time_ramp += RAMP_PERIOD;
                break;
            #endif

			//Either increment or decrement the number of rotations depending on encoder.
			//TODO: overflow?

            case encoder[0] when pinsneq(rotor[0]) :>rotor[0]:
                //if A = 1 and lastA = 0
                if (((rotor[0]>>2)==1) && (lastA[0] == 0)) {
                    //if B=0
                    if ((rotor[0]>>1)<3)
                        rotations[0]++;
                    else
                        rotations[0]--;
                }
				//set up for next loop                
				lastA[0] = (rotor[0] >> 2);
                break;

            case encoder[1] when pinsneq(rotor[1]) :>rotor[1]:
                //if A = 1 and lastA = 0
                if (((rotor[1]>>2)==1) && (lastA[1] == 0)) {
                    //if B=0
                    if ((rotor[1]>>1)<3)
                        rotations[1]++;
                    else
                        rotations[1]--;
                }
				//set up for next loop                
				lastA[1] = (rotor[1] >> 2);
                break;

            case t_pid when timerafter(time_pid) :> void:
                for (j=0;j<NUM_MOTORS;j++) {
			        // Calculate the speed with first order filter
				    speed_current[j] = ( rotations[j] - rotations_old[j]) * ( ONE_SECOND / PID_PERIOD );
				    speed_actual[j] = ( ( speed_current[j] * 100 ) + ( speed_previous[j] * 9000) ) / 10000;
				    speed_previous[j] = speed_actual[j];
				
				    // Calculate the error
				    error[j] = speed_desired[j] - speed_actual[j];

				    rotations_old[j] = rotations[j];
				
				    // Stop the motors if the desired speed is 0 and reset the integrator
				    if ( ( speed_desired[j] == 0 ) && ( speed_actual[j] > -200 ) && ( speed_actual[j] < 200 ) )
				    {
					    error[j] = 0;
					    pid_I[j] = 0;
				    }
											
				    // Calculate the integrator
				    if ( ( duty[j] > -250 ) && ( duty[j] < 250 ) )
				    {
					    pid_I[j] = pid_I[j] + ( K_I * error[j] / ( ONE_SECOND / PID_PERIOD ) );
				    }

				    // Set output
				    pid_P[j] = K_P * error[j];
				    duty[j] = (pid_P[j] + pid_I[j]) >> 12;
			        
				    // Limit the motor speed to 100% (out of 256)
				    if ( duty[j] < -255 )	{ duty[j] = -255; }
				    else if ( duty[j] > 255 )	{ duty[j] = 255; }

				    // Setup the next PID loop
				    time_pid += PID_PERIOD;
                }
				break;

            //calculate RPM for display
            case t_speed when timerafter(time_speed) :> void:
                for (int i = 0; i < NUM_MOTORS; i++) {
                    current_rpm[i] = (((rotations[i]-rotations_old_speed[i])*120)/624);
                    rotations_old_speed[i] = rotations[i];
                }
                time_speed += ONE_SECOND;
                break;
            //Process a command received from the display
            case c_speed[0] :> cmd:
			    if (cmd == CMD_GET_IQ)
			    {
				    c_speed[0] <: current_rpm[0];
				    c_speed[0] <: speed_desired[0];
			    }
			    else if (cmd == CMD_SET_SPEED)
			    {

				    c_speed[0] :> speed_desired[0];
			    }
			    break;
            //Process a command received from the display
            case c_speed[1] :> cmd:
			    if (cmd == CMD_GET_IQ2)
			    {
				    c_speed[1] <: current_rpm[1];
			    }
			    else if (cmd == CMD_SET_SPEED2)
			    {
				    c_speed[1] :> speed_desired[1];
			    }
			    break;
        }
    }
}



int main(void) {

    chan c_wd, c_speed[2], c;


	par {
    	on stdcore[MOTOR_CORE] : do_wd(c_wd, i2c_wd) ;
    	on stdcore[MOTOR_CORE] : client( c_wd, c_speed, c);
        on stdcore[INTERFACE_CORE] : display_shared_io_motor( c_speed[0], c_speed[1], lcd_ports, p_btns);
        on stdcore[MOTOR_CORE] : pwmSingleBitPort(c, pwm_clk, motor_DC_hi, (NUM_MOTORS*2), RESOLUTION, TIMESTEP,1);
	}
	return 0;
}

