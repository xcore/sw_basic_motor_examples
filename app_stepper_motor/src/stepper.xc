// Copyright (c) 2011, XMOS Ltd, All rights reserved
// This software is freely distributable under a derivative of the
// University of Illinois/NCSA Open Source License posted in
// LICENSE.txt and at <http://github.xcore.com/>


#include <print.h>
#include <platform.h>
#include <stdlib.h>
#include <syscall.h>
#include <xs1.h>
#include <stdio.h>
#include "watchdog.h"
#include <xscope.h>
#include "config.h"
#include "pwm_singlebit_port.h"
#include "adc.h"
#include "tables.h"
#include "stepper.h"

on stdcore[MOTOR_CORE] : clock pwm_clk = XS1_CLKBLK_1;
on stdcore[MOTOR_CORE]: out port i2c_wd = PORT_I2C_WD_SHARED;

//Buffered port for high side of half bridges on motors
on stdcore[MOTOR_CORE] : out buffered port:32 motor_hi_ports[4] = { PORT_M1_HI_A, PORT_M2_HI_A, PORT_M1_HI_B, PORT_M1_HI_C};
//Low sides of half bridges
on stdcore[MOTOR_CORE] : out port motor_lo_ports[4] = { PORT_M1_LO_A, PORT_M2_LO_A, PORT_M1_LO_B, PORT_M1_LO_C};


on stdcore[MOTOR_CORE]: out port ADC_SCLK = PORT_ADC_CLK;
on stdcore[MOTOR_CORE]: buffered out port:32 ADC_CNVST = PORT_ADC_CONV;
on stdcore[MOTOR_CORE]: buffered in port:32 ADC_DATA = PORT_ADC_MISO;
on stdcore[MOTOR_CORE]: in port ADC_SYNC_PORT = XS1_PORT_16A;
on stdcore[MOTOR_CORE]: clock adc_clk = XS1_CLKBLK_2;

//Period for PWM supposedly (Resolution * 20 * Timestep) in ns REF clock?
#define RESOLUTION 256
#define TIMESTEP 2
#define PERIOD (2000*256)
#define NUM_PORTS 4
#define NUM_PINS 4
#define NO_STEPS 4

#define MICROSTEPPERIOD 10000



// STEP_SIZE defines the number of microsteps to be used:
// SINE_SIZE / STEP_SIZE gives the number of microsteps
// {256,128,64,32,16,8,4,2 or 1} are valid values
#define STEP_SIZE 16

#define SINE_SIZE 256

//Chopping current
//#define CHOP_CURRENT 1
#define IMAX 500        //max phase current in milliamps

//Maximum limited current in terms of ADC
#define IMAX_ADC (int)((8192*IMAX)/6250)

//#define OPEN_LOOP_CHOPPING 1

#ifdef OPEN_LOOP_CHOPPING
int v_ref = 24
int r_winding = 44
//Maximum theoretical current without any limiting in terms of ADC
unsigned iUnlimited = (unsigned int)(((8192*v_ref*1000)/(6250*r_winding)))
#else
int iUnlimited = IMAX_ADC;
#endif



int direction = 1;
int testvar;
int maxRefI = 255;

timer microstepTimer;
int microstepTime;

void controller(chanend c_control) {
    /*c_control <: CMD_SET_MOTOR_SPEED;
    c_control <: 1000000;   //Step Period 
    c_control <: 0;         //Direction*/

    c_control <: CMD_NUMBER_STEPS;
    c_control <: 1000000;
    c_control <: 100;
    c_control <: 1;
}


/*
 * 0	0 	0	255
 * 255	0	0	0
 * 0	0	255	0
 * 0	255	0	0
 * 0	0	0	255
 *
 *
 * Try to achieve 90 degree phase difference between coils:
 */
 
 

void singleStep(chanend c_pwm, chanend c_adc, unsigned int duties[], unsigned int &step, unsigned microPeriod) {
    int adc_values[4] = {0,0,0,0};
    int finalStep;


    //Go through for each value checking
    if (step == SINE_SIZE*4)
        finalStep = SINE_SIZE;
    else 
        finalStep = step + SINE_SIZE;
    while (step != finalStep) {
        select {
            case microstepTimer when timerafter (microPeriod) :> void:
                printf("direction is %d", direction);
                //0 - 90 degrees
                if ((step >= 0) && (step < SINE_SIZE)) {
                	if (step == 0) {
                		motor_lo_ports[0+(2*direction)] <: 0;
			            motor_lo_ports[1+(2*direction)] <: 1;
			            motor_lo_ports[2-(2*direction)] <: 1;
			            motor_lo_ports[3-(2*direction)] <: 0;
                        //hack to make sure no ports left on at 4 duty cycle
                        duties[1+(2*direction)] = 0;
                	}
                    duties[0+(2*direction)] = (((((step256[(SINE_SIZE-1)-step]*IMAX_ADC)>>15)*(RESOLUTION-1))/iUnlimited));
                    duties[3-(2*direction)] = (((((step256[step]*IMAX_ADC))>>15)*(RESOLUTION-1))/iUnlimited);
                }
                //90-180 degrees
                if ((step >= SINE_SIZE) && (step < (SINE_SIZE*2))) {
                    if (step == SINE_SIZE) {
                        motor_lo_ports[0+(2*direction)] <: 0;
                        motor_lo_ports[1+(2*direction)] <: 1;
                        motor_lo_ports[2-(2*direction)] <: 0;
                        motor_lo_ports[3-(2*direction)] <: 1;
                        //hack to make sure no ports left on at 4 duty cycle
                        duties[3-(2*direction)] = 0;
                    }
                    duties[2-(2*direction)] = ((((step256[((SINE_SIZE*2)-1)-step]*IMAX_ADC)>>15)*(RESOLUTION-1))/iUnlimited);
                    duties[0+(2*direction)] = (((((step256[step- SINE_SIZE]*IMAX_ADC)>>15)*(RESOLUTION-1))/iUnlimited));
                }
                //180-270 degrees
                if ((step >= (SINE_SIZE*2)) && (step < (SINE_SIZE*3))) {
                    if (step == (SINE_SIZE * 2)) {
                       	motor_lo_ports[0+(2*direction)] <: 1;
			            motor_lo_ports[1+(2*direction)] <: 0;
			            motor_lo_ports[2-(2*direction)] <: 0;
			            motor_lo_ports[3-(2*direction)] <: 1;
                        //hack to make sure no ports left on at 4 duty cycle
                        duties[0+(2*direction)] = 0;
                    }
                    duties[1+(2*direction)] = ((((step256[((SINE_SIZE*3)-1)-step]*IMAX_ADC)>>15)*(RESOLUTION-1))/iUnlimited);
                    duties[2-(2*direction)] = (((((step256[step- (SINE_SIZE*2)]*IMAX_ADC))>>15)*(RESOLUTION-1))/iUnlimited);
                }
                //270-360 degrees
                if ((step >= (SINE_SIZE*3)) && (step < (SINE_SIZE*4))) {
                    if (step == (SINE_SIZE * 3)) {
                		motor_lo_ports[0+(2*direction)] <: 1;
                		motor_lo_ports[1+(2*direction)] <: 0;
			            motor_lo_ports[2-(2*direction)] <: 1;
			            motor_lo_ports[3-(2*direction)] <: 0;
                        //hack to make sure no ports left on at 4 duty cycle
                        duties[2-(2*direction)] = 0;
                    }
                    duties[3-(2*direction)] = ((((step256[((SINE_SIZE*4)-1)-step]*IMAX_ADC)>>15)*(RESOLUTION-1))/iUnlimited);
                    duties[1+(2*direction)] = (((((step256[step- (SINE_SIZE*3)]*IMAX_ADC))>>15)*(RESOLUTION-1))/iUnlimited);
                }
                //only on last step
                if (step == SINE_SIZE*4) {
                	duties[3-(2*direction)] = ((((step256[(SINE_SIZE*4) - step]*IMAX_ADC)>>15)*(RESOLUTION-1))/iUnlimited);
                	duties[1+(2*direction)] = (((((step256[step- (SINE_SIZE*3) -1]*IMAX_ADC))>>15)*(RESOLUTION-1))/iUnlimited);
                }

            #ifdef CHOP_CURRENT
                //Chop current
                for (int i=0; i < 4; i++) {
                    if (duties[i] > maxRefI)
                        duties[i] = maxRefI;
                }
                c_adc <: 0;
                    
                slave {
                    c_adc :> adc_values[0];
                    c_adc :> adc_values[3];
                    c_adc :> adc_values[1];
                    c_adc :> adc_values[2];
                }
                
                //This decreases the maximum duty cycle until the ADC never goes above maximum value
                for (int i = 0; i < 4; i++) {
                    if (adc_values[i] > IMAX_ADC) {
                        maxRefI = duties[0];
                        for (int j=1; j< 4;j++) {
                            if (duties[j] > maxRefI)
                                maxRefI = duties[j]-1;
                        }
                    }
                }
                
            #endif

                pwmSingleBitPortSetDutyCycle(c_pwm, duties, 4);
                //printf("Duty set, %d, %d, %d, %d\n", duties[0], duties[1], duties[2], duties[3]);

                if (step == SINE_SIZE*4)
                    step = 0;
                else
                    step += STEP_SIZE; 
                
                break;    
         }
    }
}




void motor(chanend c_pwm, chanend c_adc, chanend c_wd, chanend c_control) {
    unsigned int duties[4] = {0,0,0,0};
    unsigned int step = 0, noSteps = 0, stepCounter = 0, stepPeriod = 10000, doSteps = 0, doSpeed = 0;
    int cmd;

	timer t;
    int time;
    xscope_config_io(XSCOPE_IO_BASIC);
    xscope_register (1, XSCOPE_CONTINUOUS , " Ia " , XSCOPE_UINT , " Current Sense " );

    microstepTimer :> microstepTime;
    //delay to make sure the ADC is calibrated before starting output and watchdog is enabled
    t :> time;
    t when timerafter (time+100000000) :> time;
    c_wd <: WD_CMD_START;
    time += PERIOD;

    while (1) {
        select {
        
            case c_control :> cmd:
                if (cmd == CMD_SET_MOTOR_SPEED) {
                    doSteps = 0;
                    doSpeed = 1;
                    c_control :> stepPeriod;
                    c_control :> direction;
                }
                if (cmd == CMD_NUMBER_STEPS) {
                    doSpeed = 0;
                    doSteps = 1;
                    c_control :> stepPeriod;
                    c_control :> noSteps;
                    c_control :> direction;
                }
                break;
                    
            //loop for speed
            case t when timerafter (time) :> void:
                if (doSpeed) {
                    singleStep(c_pwm, c_adc, duties, step, MICROSTEPPERIOD);
                }
                else if (doSteps) {
                    if (stepCounter == noSteps) {
                        doSteps = 0;
                        stepCounter = 0;
                    }
                    else {
                        singleStep(c_pwm, c_adc, duties, step, MICROSTEPPERIOD);
                        stepCounter++;
                    }
                }
                time += stepPeriod;
                break;

        }
    }
}




int main(void) {

    chan c_pwm, c_wd, c_control, c_adc;


	par {
	    on stdcore[INTERFACE_CORE] : controller(c_control);
    	on stdcore[MOTOR_CORE] : motor(c_pwm, c_adc, c_wd, c_control);
		on stdcore[MOTOR_CORE] : pwmSingleBitPort(c_pwm, pwm_clk, motor_hi_ports, 4, RESOLUTION, TIMESTEP,1);
		on stdcore[MOTOR_CORE] : {
		    /*xscope_register (9,
		    XSCOPE_CONTINUOUS , " Ia " , XSCOPE_UINT , " Current Sense " ,
			XSCOPE_CONTINUOUS , " Ib " , XSCOPE_UINT , " Current Sense " ,
			XSCOPE_CONTINUOUS , " Ic " , XSCOPE_UINT , " Current Sense " ,
			XSCOPE_CONTINUOUS , " Id " , XSCOPE_UINT , " Current Sense " ,
			XSCOPE_CONTINUOUS , " test" , XSCOPE_UINT , " tester ",
		    XSCOPE_CONTINUOUS , " 0 " , XSCOPE_UINT , " Hi Side " ,
			XSCOPE_CONTINUOUS , " 1 " , XSCOPE_UINT , " Hi Side " ,
			XSCOPE_CONTINUOUS , " 2 " , XSCOPE_UINT , " Hi Side " ,
			XSCOPE_CONTINUOUS , " 3 " , XSCOPE_UINT , " Hi Side " );*/
			adc_with_scope( c_adc, adc_clk, ADC_SCLK, ADC_CNVST, ADC_DATA);
		}
    	on stdcore[MOTOR_CORE] : do_wd(c_wd, i2c_wd);
	}
	return 0;
}

