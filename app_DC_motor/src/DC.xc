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
//#include <xscope.h>
#include "watchdog.h"
#include "config.h"
#include "pwm_singlebit_port.h"
#include "shared_io.h"
#include "DC.h"

/*
* Connections as follows:
* Encoder A = Encoder A
* Encoder B = Encoder B
* +ve Motor = PH A
* -ve Motor = PH B
*/



//PWM clock and Watchdog port
on stdcore[MOTOR_CORE] : clock pwm_clk = XS1_CLKBLK_1;
on stdcore[INTERFACE_CORE]: out port i2c_wd = PORT_WATCHDOG;

//High sides of half-bridges
on stdcore[MOTOR_CORE] : out buffered port:32 motor_DC_hi[4] = {PORT_M1_HI_A, PORT_M1_HI_B, PORT_M2_HI_A, PORT_M2_HI_B};

//Low sides of half-bridges
on stdcore[MOTOR_CORE] : out port motor_DC_lo[4] = {PORT_M1_LO_A, PORT_M1_LO_B, PORT_M2_LO_A, PORT_M2_LO_B};

//Motor Encoders on Motor 1 and Motor 2
on stdcore[MOTOR_CORE] : in port encoder[2] = {PORT_M1_ENCODER, PORT_M2_ENCODER};


// core with LCD and BUTTON interfaces
on stdcore[INTERFACE_CORE]: lcd_interface_t lcd_ports = { PORT_SPI_CLK, PORT_SPI_MOSI, PORT_SPI_SS_DISPLAY, PORT_SPI_DSA };
on stdcore[INTERFACE_CORE]: in port p_btns = PORT_BUTTONS;
on stdcore[INTERFACE_CORE]: out port p_leds = PORT_LEDS;



//Example of control
void DCcontroller (chanend c_control) {
    ramp_parameters rampParam = {40,50,0,1000,1};
    ramp_parameters rampParam2 = {40,160,1,500,1};
    
    /*c_control <: CMD_RAMP;
    c_control <: 0;             //Which motor to ramp
    c_control <: rampParam;*/
    
    /*c_control <: CMD_RAMP;
    c_control <: 1;
    c_control <: rampParam2;*/
    
    c_control <: CMD_SET_MOTOR_SPEED;
    c_control <: 0;
    c_control <: 5;
    c_control <: 0;
    
    c_control <: CMD_SET_MOTOR_SPEED;
    c_control <: 1;
    c_control <: 5;
    c_control <: 1;

}

void motors( chanend c_wd, chanend c_speed[], chanend c_pwm, chanend c_control) {
    
    //TODO: Store vars in struct to better allow for more motors
    timer t, t_ramp, t_speed;
    unsigned rotor[2] = {0,0};
    int time, time_speed, time_ramp, ts, lastA[2] = {0,0}, current_rpm[2] = {0,0}, rotations_old_speed[NUMBER_OF_MOTORS], j, cmd;
    int speed_desired[NUMBER_OF_MOTORS] = {0,0}, speed_current[NUMBER_OF_MOTORS]={0,0}, speed_actual[NUMBER_OF_MOTORS]={0,0}, speed_previous[NUMBER_OF_MOTORS] = {0,0};
    int rotations_old[NUMBER_OF_MOTORS] = {0,0}, pid_I[NUMBER_OF_MOTORS] = {0,0}, pid_P[NUMBER_OF_MOTORS] = {0,0}, error[NUMBER_OF_MOTORS];
    int rotations[NUMBER_OF_MOTORS] = {0,0}, duty[NUMBER_OF_MOTORS] = {0,0};
    unsigned int duties[4] = {0,0,0,0};
    
    int doRamp[NUMBER_OF_MOTORS] = {0,0}, direction[NUMBER_OF_MOTORS] = {0,0}, whichMotor, rampPeriodCount[NUMBER_OF_MOTORS] = {0,0};
    ramp_parameters rampParam[NUMBER_OF_MOTORS];
    
    //calculate once as optimisation
    int period_per_second = ONE_SECOND / PERIOD;

    //Wait 0.5s to ensure watchdog works correctly
    t :> ts;
    t when timerafter ( ts + 100000000 ) :> ts;
    c_wd <: WD_CMD_START;

    //Ensure all low sides are zero
    for (j=0; j<(NUMBER_OF_MOTORS*2); j++)
        motor_DC_lo[j] <: 0;

    //Initialise timers
    t :> time;
    t_ramp :> time_ramp;
    t_speed :> time_speed;
    time += PERIOD;
    time_speed += ONE_SECOND;
    
   /*xscope_config_io(XSCOPE_IO_BASIC);
    
    xscope_register(4,
    XSCOPE_CONTINUOUS, "rot0", XSCOPE_UINT, "Value",
    XSCOPE_CONTINUOUS, "rot1", XSCOPE_UINT, "Value",
    XSCOPE_CONTINUOUS, "rotor0", XSCOPE_UINT, "Value",
    XSCOPE_CONTINUOUS, "rotor1", XSCOPE_UINT, "Value");*/

	//Turn on low sides of M1 phase A and M2 phase A
    for (j = 0; j < NUMBER_OF_MOTORS; j++)    
        motor_DC_lo[((2*j)+1)] <: 1;
    
	while (1) {
        select {
            //Loop for updating PWM duty cycle
            case t when timerafter (time) :> void:
                for (j=0;j<NUMBER_OF_MOTORS;j++) {

			        // Calculate the speed with first order filter
				    speed_current[j] = ( rotations[j] - rotations_old[j]) * ( period_per_second );
				    speed_actual[j] = ( ( speed_current[j] * 1000 ) + ( speed_previous[j] * 9000) ) / 10000;
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
					    pid_I[j] += error[j]  * ( K_I / ( period_per_second ) );
				    }

				    // Set output
				    pid_P[j] = K_P * error[j];
				    duty[j] = (pid_P[j] + pid_I[j]) >> 12;
			        
				    // Limit the motor speed to 100% (out of 256)
				    if ( duty[j] < -55 )	{ duty[j] = -55; }
				    else if ( duty[j] > 55 )	{ duty[j] = 55; }
				    
				    //If we're going backwards then invert the duty cycle
				    if (direction[j])  { 
				        duty[j] = -duty[j]; 
				    }
                }      
                //xscope_probe_data(0, duty[0]);
                
                for (j=0; j<NUMBER_OF_MOTORS; j++) {
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
				    //If the duty cycle has gone negative, reverse current direction
                    if (duty[j] < 0) {
                        duties[0 + (2*j)] = 0;
                        pwmSingleBitPortSetDutyCycle(c_pwm, duties, (NUMBER_OF_MOTORS*2));
                        motor_DC_lo[1 + (2*j)] <: 0;
                        motor_DC_lo[0 + (2*j)] <: 1;
                        duties[1+(2*j)] = -duty[j];
                    }                    
				//Otherwise set up for normal operation
                    else {
                        duties[0 + (2*j)] = 0;
                        pwmSingleBitPortSetDutyCycle(c_pwm, duties, (NUMBER_OF_MOTORS*2));
                        motor_DC_lo[0+(2*j)] <: 0;
                        motor_DC_lo[1+(2*j)] <: 1; 
                        duties[0+(2*j)] = duty[j];
                    }
                    rampPeriodCount[j]++; 
                }

                pwmSingleBitPortSetDutyCycle(c_pwm, duties, (NUMBER_OF_MOTORS*2));
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
                    c_control :> whichMotor;
                    c_control <: current_rpm[whichMotor];
                }
                /*else if (cmd == CMD_POSITION)
                    c_control <: targetPosition*/
                break;
            

            case encoder[0] when pinsneq(rotor[0]) :> rotor[0]:
                //if A = 1 and lastA = 0
                if (((0x1 & (rotor[0] >> 0))==1) && (lastA[0] == 0)) {
                    //if B=0
                    if ((0x1 & (rotor[0] >> 1)) == 0)
                        rotations[0]++;
                    else
                        rotations[0]++;//--;
                }
				//set up for next loop                
				lastA[0] = (0x1 & (rotor[0] >> 0));
				
                /*xscope_probe_data(0, rotations[0]);
                xscope_probe_data(1, rotations[1]);
                xscope_probe_data(2, rotor[0]);
                xscope_probe_data(3, rotor[1]);*/
                
                break;

            case encoder[1] when pinsneq(rotor[1]) :>rotor[1]:
                //if A = 1 and lastA = 0
                //if A = 1 and lastA = 0
                if (((0x1 & (rotor[1] >> 0))==1) && (lastA[1] == 0)) {
                    //if B=0
                    if ((0x1 & (rotor[1] >> 1)) == 0)
                        rotations[1]++;
                    else
                        rotations[1]++;//--;
                }
				//set up for next loop                
				lastA[1] = (0x1 & (rotor[1] >> 0));
				
                /*xscope_probe_data(2, rotor[0]);
                xscope_probe_data(3, rotor[1]);*/
                break;

            //calculate RPM for display
            case t_speed when timerafter(time_speed) :> void:
                for (int i = 0; i < NUMBER_OF_MOTORS; i++) {
                    current_rpm[i] = (((rotations[i]-rotations_old_speed[i])*120)/624);
                    rotations_old_speed[i] = rotations[i];
                }
                time_speed += ONE_SECOND;
                break;
                
            //Process a command received from the display for motor 1
            //TODO: Possible to combine into one case statement?
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
			    else if (cmd == CMD_DIR)
			    {
			        direction[0] = !direction[0];
			    }
			    break;
			    
            //Process a command received from the display for motor 2
            case c_speed[1] :> cmd:
			    if (cmd == CMD_GET_IQ)
			    {
				    c_speed[1] <: current_rpm[1];
				    c_speed[1] <: speed_desired[1];
			    }
			    else if (cmd == CMD_SET_SPEED)
			    {
				    c_speed[1] :> speed_desired[1];
			    }
			    else if (cmd == CMD_DIR)
			    {
			        direction[1] = !direction[1];
			    }
			    break;
        }
    }
}

int main(void) {

    chan c_wd, c_speed[2], c_pwm, c_control;


	par {
    	on stdcore[INTERFACE_CORE] : do_wd(c_wd, i2c_wd) ;
        on stdcore[INTERFACE_CORE] : DCcontroller(c_control);
    	on stdcore[MOTOR_CORE] : motors( c_wd, c_speed, c_pwm, c_control);
        on stdcore[INTERFACE_CORE] : display_shared_io_manager( c_speed, lcd_ports, p_btns, p_leds);
        
        on stdcore[MOTOR_CORE] : pwmSingleBitPort(c_pwm, pwm_clk, motor_DC_hi, (NUMBER_OF_MOTORS*2), RESOLUTION, TIMESTEP,1);
	}
	return 0;
}

