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

#define USE_XSCOPE

#ifdef USE_XSCOPE
#include <xscope.h>
#endif

#include "watchdog.h"
#include "config.h"
#include "pwm_singlebit_port.h"
#include "adc.h"
#include "shared_io.h"
#include "DC.h"

/*
* Connections as follows:
* Encoder A = Encoder A
* Encoder B = Encoder B
* +ve Motor = PH A
* -ve Motor = PH B
*/


//Defines for PWM library
//Period for PWM should be (Resolution * 20 * Timestep) in ns?
#define 	RESOLUTION	    256
#define 	TIMESTEP	    2

// Loop time for the control loop
//
//   At 160rpm 1:52 gear -> 8320rpm / 60 -> 138revs per sec * 12 -> 1664 encoder counts per second
//   At 100rpm 1:52 gear -> 5200rpm / 60 -> 80revs per sec * 12  -> 1040 encoder counts per second
//   So we need a speed analysis loop of slower than 1040 Hz
//
#define 	PERIOD		    1000000
#define 	ONE_SECOND	    100000000 	// 1000ms

//PID controller parameters
#define 	K_P        	    2000
#define 	K_I        	    2000

// The gear ratio of the gearbox
#define GEAR_RATIO 54

// The number of complete encoder cycles that occurs in each revolution of the spindle
#define ENCODER_COUNTS_PER_REV	12

#define MAX_DUTY_CYCLE 160

//PWM clock and Watchdog port
on stdcore[INTERFACE_CORE] : out port i2c_wd = PORT_WATCHDOG;

// PWM
on stdcore[MOTOR_CORE] : clock pwm_clk = XS1_CLKBLK_1;
on stdcore[MOTOR_CORE] : out buffered port:32 motor_DC_hi[4] = {PORT_M1_HI_A, PORT_M1_HI_B, PORT_M2_HI_A, PORT_M2_HI_B};
on stdcore[MOTOR_CORE] : out port motor_DC_lo[4] = {PORT_M1_LO_A, PORT_M1_LO_B, PORT_M2_LO_A, PORT_M2_LO_B};

//Motor Encoders on Motor 1 and Motor 2
on stdcore[MOTOR_CORE] : in port encoder[2] = {PORT_M1_HALLSENSOR, PORT_M2_HALLSENSOR};

// core with LCD and BUTTON interfaces
on stdcore[INTERFACE_CORE]: lcd_interface_t lcd_ports = { PORT_SPI_CLK, PORT_SPI_MOSI, PORT_SPI_SS_DISPLAY, PORT_SPI_DSA };
on stdcore[INTERFACE_CORE]: in port p_btns = PORT_BUTTONS;

static const char sensor_pattern[2][4]  = { { 0xd, 0x9, 0xb, 0xf }, { 0xf, 0xb, 0x9, 0xd } };

//Example of control
void controller (chanend c_control) {
    
    c_control <: CMD_SET_MOTOR_SPEED;
    c_control <: 0;
    c_control <: 80;
    c_control <: 0;
    
#if NUM_MOTORS > 1
    c_control <: CMD_SET_MOTOR_SPEED;
    c_control <: 1;
    c_control <: 80;
    c_control <: 1;
#endif

}

void motors( chanend c_wd, chanend c_speed[], chanend c, chanend c_control) {
    
    //TODO: Store vars in struct to better allow for more motors
    timer t, t_ramp, t_speed;
    int time, time_speed, time_ramp, ts, j, cmd;
    unsigned rotor[NUM_MOTORS];
    int current_rpm[NUM_MOTORS];
    int rotations_old_speed[NUM_MOTORS];
    int speed_desired[NUM_MOTORS];
    int speed_current[NUM_MOTORS];
    int speed_actual[NUM_MOTORS];
    int speed_previous[NUM_MOTORS];
    int rotations_old[NUM_MOTORS];
    int pid_I[NUM_MOTORS];
    int pid_P[NUM_MOTORS];
    int error[NUM_MOTORS];
    int rotations[NUM_MOTORS];
    int duty[NUM_MOTORS];
    unsigned int duties[NUM_MOTORS*2];
    
    int doRamp[NUM_MOTORS];
    int direction[NUM_MOTORS];
    int whichMotor;
    int rampPeriodCount[NUM_MOTORS];
    ramp_parameters rampParam[NUM_MOTORS];


    for (unsigned int n=0; n<NUM_MOTORS; ++n) {
    	rotor[n] = 0;
        current_rpm[n] = 0;
        speed_desired[n] = 0;
        speed_current[n] = 0;
        speed_actual[n] = 0;
        speed_previous[n] = 0;
        rotations_old[n] = 0;
        rotations_old_speed[n] = 0;
        pid_I[n] = 0;
        pid_P[n] = 0;
        rotations[n] = 0;
        duty[n] = 0;
        duties[2*n+0] = 0;
        duties[2*n+1] = 0;
        rampPeriodCount[n] = 0;
        direction[n] = 0;
        doRamp[n] = 0;
    }


    //Wait 0.5s to ensure watchdog works correctly
    t :> ts;
    t when timerafter ( ts + 100000000 ) :> ts;
    c_wd <: WD_CMD_START;

    //Ensure all low sides are zero
    for (j=0; j<(NUM_MOTORS*2); j++)
        motor_DC_lo[j] <: 0;

    //Initialise timers
    t :> time;
    t_ramp :> time_ramp;
    t_speed :> time_speed;
    time += PERIOD;
    time_speed += ONE_SECOND;

#ifdef USE_XSCOPE
    xscope_register(2,
    	XSCOPE_CONTINUOUS, "Reference 1", XSCOPE_UINT, "n",
    	XSCOPE_CONTINUOUS, "Reference 2", XSCOPE_UINT, "n");
    xscope_config_io(XSCOPE_IO_BASIC);
#endif

	while (1) {
        select {

            //Loop for updating PWM duty cycle
            case t when timerafter (time) :> void:
                for (j=0;j<NUM_MOTORS;j++) {

			        // Calculate the speed with first order filter (speed in encoder counts per second)
				    speed_current[j] = (rotations[j] - rotations_old[j]) * (ONE_SECOND / PERIOD);
				    speed_actual[j] = ( ( speed_current[j] * 1000 ) + ( speed_previous[j] * 9000) ) / 10000;
				    speed_previous[j] = speed_actual[j];

				    // Calculate the error
				    error[j] = (speed_desired[j] * GEAR_RATIO * ENCODER_COUNTS_PER_REV / 60) - speed_actual[j];
				    rotations_old[j] = rotations[j];

				    // Stop the motors if the desired speed is 0 and reset the integrator
				    if ( ( speed_desired[j] == 0 ) && ( speed_actual[j] > -200 ) && ( speed_actual[j] < 200 ) )
				    {
					    error[j] = 0;
					    pid_I[j] = 0;
				    }
											
				    // Calculate the integrator
				    pid_I[j] += error[j]  * ( K_I / ( ONE_SECOND / PERIOD ) );

				    // Set output
				    pid_P[j] = K_P * error[j];
				    duty[j] = (pid_P[j] + pid_I[j]) >> 12;
			        
				    // Limit the motor speed to 100% (out of 256)
				    if ( duty[j] < -MAX_DUTY_CYCLE )	{ duty[j] = -MAX_DUTY_CYCLE; }
				    else if ( duty[j] > MAX_DUTY_CYCLE )	{ duty[j] = MAX_DUTY_CYCLE; }

				    if (j==0) {
				    	xscope_probe_data(0, error[j]);
				    	xscope_probe_data(1, duty[j]);
				    }
				    
				    //If we're going backwards then invert the duty cycle
				    if (direction[j])  { 
				        duty[j] = -duty[j];
				    }

                    //deal with ramping
                    if (doRamp[j]) {
                        if (rampPeriodCount[j] == rampParam[j].rampPeriod) {
                            speed_desired[j] += rampParam[j].acceleration;
                            rampPeriodCount[j] = 0;
                        }
                        //If we've reached the target speed, stop ramping
                        //TODO: What if targetSpeed is not a multiple of acceleration
                        if (speed_desired[j] == rampParam[j].targetSpeed)
                            doRamp[j] = 0;
                    }

				    // If the duty cycle has gone negative, reverse current direction
                    if (duty[j] < 0) {
                        duties[0 + (2*j)] = 0; // high A
                        pwmSingleBitPortSetDutyCycle(c, duties, (NUM_MOTORS*2));
                        motor_DC_lo[1 + (2*j)] <: 0; // low B
                        motor_DC_lo[0 + (2*j)] <: 1; // low A
                        duties[1 + (2*j)] = -duty[j]; // high B
                    }
                    // Otherwise set up for normal operation
                    else {
                        duties[1 + (2*j)] = 0; // high B
                        pwmSingleBitPortSetDutyCycle(c, duties, (NUM_MOTORS*2));
                        motor_DC_lo[0 + (2*j)] <: 0; // low A
                        motor_DC_lo[1 + (2*j)] <: 1; // low B
                        duties[0 + (2*j)] = duty[j]; // high A
                    }
                    rampPeriodCount[j]++; 
                }

                pwmSingleBitPortSetDutyCycle(c, duties, (NUM_MOTORS*2));
                time += PERIOD;
                break;

            //Process a command from the control thread
            case c_control :> cmd:
                //Received a command to ramp
                if (cmd == CMD_RAMP) {
                    c_control :> whichMotor;
                    c_control :> rampParam[whichMotor];
                    
                    doRamp[whichMotor] = 1;
                    speed_desired[whichMotor] = rampParam[whichMotor].startSpeed;
                    direction[whichMotor] = rampParam[whichMotor].direction;
                }
                else if (cmd == CMD_SET_MOTOR_SPEED) {
                    c_control :> whichMotor;
                    c_control :> speed_desired[whichMotor];
                    c_control :> direction[whichMotor];
                }
                else if (cmd == CMD_GET_MOTOR_SPEED) {
                    c_control <: current_rpm[0];
                }
                break;
            
            // Hall B and Hall C are wired to the motor on bits 1 and 2
            // so hall pattern is F->B->9->D
            case (int n=0; n<NUM_MOTORS; n++) encoder[n] when pinseq(sensor_pattern[direction[n]][rotor[n]]) :> cmd:
				if (cmd == 0xF) {
					rotations[n]++;
				}
				rotor[n] = (rotor[n]+1) & 0x3;
                break;

            //calculate RPM for display
            case t_speed when timerafter(time_speed) :> void:
                for (int i = 0; i < NUM_MOTORS; i++) {
                    current_rpm[i] = ((rotations[i]-rotations_old_speed[i]) * 60 * 10) / (GEAR_RATIO * ENCODER_COUNTS_PER_REV);
                    rotations_old_speed[i] = rotations[i];
                }
                time_speed += (ONE_SECOND/10);
                break;

            //Process a command received from the display
            case (int n=0; n<NUM_MOTORS; ++n) c_speed[n] :> cmd:
			    if (cmd == CMD_GET_IQ)
			    {
				    c_speed[n] <: current_rpm[n];
				    c_speed[n] <: speed_desired[n];
			    }
			    else if (cmd == CMD_SET_SPEED)
			    {
				    c_speed[n] :> speed_desired[n];
			    }
			    else if (cmd == CMD_DIR)
			    {
			    	direction[n] = !direction[n];
			    }
			    break;
        }
    }
}



int main(void) {

    chan c_wd, c_speed[2], c, c_control;


	par {
    	on stdcore[INTERFACE_CORE] : do_wd(c_wd, i2c_wd) ;
        on stdcore[INTERFACE_CORE] : controller(c_control);
    	on stdcore[MOTOR_CORE] : motors( c_wd, c_speed, c, c_control);
        on stdcore[INTERFACE_CORE] : display_shared_io_motor( c_speed[0], c_speed[1], lcd_ports, p_btns);
        
        on stdcore[MOTOR_CORE] : pwmSingleBitPort(c, pwm_clk, motor_DC_hi, (NUM_MOTORS*2), RESOLUTION, TIMESTEP,1);
	}
	return 0;
}


