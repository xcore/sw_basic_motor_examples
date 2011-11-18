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
#include <xscope.h>

#include "watchdog.h"
#include "config.h"
#include "pwm_singlebit_port.h"
#include "adc_7265.h"
#include "tables.h"
#include "stepper.h"

//Clock for PWM
on stdcore[MOTOR_CORE] : clock pwm_clk = XS1_CLKBLK_1;

//Watchdog port
on stdcore[INTERFACE_CORE]: out port i2c_wd = PORT_WATCHDOG;

//Buffered port for high and low sides of four half bridges
on stdcore[MOTOR_CORE] : out buffered port:32 motor_ports[8] = { PORT_M1_HI_A, PORT_M2_HI_A, PORT_M1_HI_B, PORT_M2_HI_B, PORT_M1_LO_A, PORT_M2_LO_A, PORT_M1_LO_B, PORT_M2_LO_B};
#define M1_HI_A 0
#define M2_HI_A 1
#define M1_HI_B 2
#define M2_HI_B 3
#define M1_LO_A 4
#define M2_LO_A 5
#define M1_LO_B 6
#define M2_LO_B 7


//Ports for ADC
on stdcore[MOTOR_CORE]: out port ADC_SCLK = PORT_ADC_CLK;
on stdcore[MOTOR_CORE]: port ADC_CNVST = PORT_ADC_CONV;
on stdcore[MOTOR_CORE]: buffered in port:32 ADC_DATA_A = PORT_ADC_MISOA;
on stdcore[MOTOR_CORE]: buffered in port:32 ADC_DATA_B = PORT_ADC_MISOB;
on stdcore[MOTOR_CORE]: out port ADC_MUX = PORT_ADC_MUX;
on stdcore[MOTOR_CORE]: in port ADC_SYNC_PORT = XS1_PORT_16A;
on stdcore[MOTOR_CORE]: clock adc_clk = XS1_CLKBLK_2;

// Resolution of PWM, normally 256
#define RESOLUTION 256

// Determines frequency of PWM
#define TIMESTEP 12

// Initial step period
#define PERIOD 100000

// Time period for updating the PWM values
#define ADC_PERIOD 20000

//Anti wind-up gain
#define PI_ANTIWINDUP 500

#define PIGAIN1 1000
#define PIGAIN2 50

#define USE_XSCOPE

// The number of entries in the cosine lookup table
#define COS_SIZE 256

// STEP_SIZE defines the number of microsteps to be used:
// Note: must be an exact divisor of COS_SIZE
#define STEP_SIZE 16

// The number of microsteps
#define MICROSTEP_COUNT ((COS_SIZE) / STEP_SIZE)

//Sets the decay mode
//slow = fixed decay (always slow decay)
//fast = alternating decay (slow during rising current, fast during falling)
//fast decay provides higher speeds with slightly noisier operation
#define DECAY_MODE slow

//Open loop limits current according to theoretical current from R and V
//#define OPEN_LOOP_CHOPPING

// Max phase current in milliamps
// Note - only used if OPEN_LOOP_CHOPPING defined
#define IMAX 200
#define V_REF 12
#define R_WINDING 44

#define OPEN_LOOP_CHOPPED_DUTY_CYCLE ((IMAX << 13 * 255) / (((V_REF << 13) / (R_WINDING))*1000))

int output1 = 0, output2 = 0, finalStep, startupFlag = 1;

int refCurrentADC[2];

unsigned duty[8] = {0,0,0,0,0,0,0,0};

enum decay {fast, slow};
enum decay decayMode[2];

int PISum0 = 0, PISum1 = 0;
int PILastError0 = 0, PILastError1 = 0;

void controller(chanend c_control) {
    c_control <: CMD_SET_MOTOR_SPEED;
    c_control <: 1000000;   //Step Period 
    c_control <: FORWARD;     //Direction
}

 
//Can be called for both windings, just pass different sum and error vars.
//Should return a voltage output

//TODO Sort out previous outputs
int applyPI(int referenceI,int actualI, int &sum, int &lastError, int &lastOutput) {
   
    int currentError = referenceI - actualI;

    if (referenceI == 0 ) {
        sum = 0;
        currentError = 0;
        lastError = 0;
    }
    else {
        sum += ((currentError * PIGAIN1 - lastError * PIGAIN2)>>13)-(((sum-lastOutput) * PI_ANTIWINDUP)>>15);
    }
    
	if (sum > 255) {
	    sum = 255;
	}
	else if (sum < -255) {
	    sum = -255;
	}
    
	lastOutput = sum;
    lastError = currentError;

	return lastOutput;
}

/*
* Given PIOutput from the PI controller ( two values, one for each winding)
* transform into output duty cycles for each high side MOSFET (4)
* 
* winding 0 on motor port 1 phase A, motor port 2 phase A
* winding 1 on motor port 1 phase B, motor port 1 phase C
*
* duty consists of high ports x 4 followed by low ports x 4
*/
void setWindingPWM(const int refCurrentADC[], const int PIOutput[], const enum decay decayMode[],  chanend c_pwm) {

    
    //Winding 0
    if (refCurrentADC[0] > 0) {
    	int pi = (PIOutput[0] < 0) ? 0 : PIOutput[0];

        if (decayMode[0] == slow) {
            duty[M1_LO_A] = 255;
            duty[M2_LO_A] = 0;
        }
        else if (decayMode[0] == fast) {
            duty[M1_LO_A] = pi;
            duty[M2_LO_A] = 0;
        }
        duty[M1_HI_A] = 0;
        duty[M2_HI_A] = pi;
    }
    else if (refCurrentADC[0] < 0) {
    	int pi = (PIOutput[0] > 0) ? 0 : -PIOutput[0];

        if (decayMode[0] == slow) {
            duty[M1_LO_A] = 0;
            duty[M2_LO_A] = 255;
        }
        else if (decayMode[0] == fast) {
            duty[M1_LO_A] = 0;
            duty[M2_LO_A] = pi;
        }
        duty[M2_HI_A] = 0;
        duty[M1_HI_A] = pi;
    }
    else {
        duty[M1_LO_A] = 255;
        duty[M2_LO_A] = 255;
         
        duty[M2_HI_A] = 0;
        duty[M1_HI_A] = 0;
    }
    
    //Winding 1
    if (refCurrentADC[1] >0) {
    	int pi = (PIOutput[1] < 0) ? 0 : PIOutput[1];

        if (decayMode[1] == slow) {
            duty[M1_LO_B] = 255;
            duty[M2_LO_B] = 0;
        }
        else if (decayMode[1] == fast) {
            duty[M1_LO_B] = pi;
            duty[M2_LO_B] = 0;
        } 
        duty[M1_HI_B] = 0;
        duty[M1_HI_C] = pi;
    }
    else if (refCurrentADC[1] < 0) {
    	int pi = (PIOutput[1] > 0) ? 0 : -PIOutput[1];

        if (decayMode[1] == slow) {
            duty[M1_LO_B] = 0;
            duty[M2_LO_B] = 255;
        }
        else if (decayMode[1] == fast) {
            duty[M1_LO_B] = 0;
            duty[M2_LO_B] = pi;
        }
        duty[M1_HI_B] = pi;
        duty[M1_HI_C] = 0;
    }
    else {
        duty[M1_LO_B] = 255;
        duty[M2_LO_B] = 255;
       
        duty[M1_HI_B] = 0;
        duty[M1_HI_C] = 0;
    }

    pwmSingleBitPortSetDutyCycle(c_pwm, duty, 8);
}



/*
 * Ideally we would measure the coil currents using the ADC here.
 *
 * The ADC on the XMOS motor development board has six channels, grouped
 * as three pairs.  Each pair can be simultaneously samples by the dual
 * ADC.
 *
 * In a stepper motor scenario, a dual ADC is required to be able to sample
 * the coil currents in the two coils.  There are four coils current states
 * with respect to the 4 H-bridges that feed the two coils. These are:
 *
 *   coil1 A1H -> A2L, coil2 B1H -> B2L
 *   coil1 A1H -> A2L, coil2 B1L <- B2H
 *   coil1 A1L <- A2H, coil2 B1H -> B2L
 *   coil1 A1L <- A2H, coil2 B1L <- B2H
 *
 * Since we can only measure two of the low sides of the bridges at the same
 * time, we cannot arrange for the dual ADC to measure the two coil currents
 * without sampling each of the 4 low side half bridges.
 *
 * If the ADC sense resistors were measuring the current through both of the
 * low side bridges for each coil then we could do a simple ADC measurement.
 *
 */
{int, int } getWindingADC ( streaming chanend c_adc) {

	int windingCurrent[2] = {200,200};

	// We do not perform ADC measurements, and consequently do not do torque
	// control.
    return { windingCurrent[0], windingCurrent[1] };
}

/*
** 0	0 	0	255
** 255	0	0	0
** 0	0	255	0
** 0	255	0	0
** 0	0	0	255
*/    

void singleStep(chanend c_pwm, unsigned int &step, unsigned direction, unsigned microPeriod, streaming chanend c_adc) {

    int currentReference[2];

    timer microstepTimer, adc_timer;
    int microstepTime, adc_time;
    
    microstepTimer :> microstepTime;
    adc_timer :> adc_time;

    //If we've reached the end of a sine wave then reset the step count
    if (step > COS_SIZE*4) {
        finalStep = COS_SIZE;
        step = 0;
    }
    //Otherwise increment the final step by one full step.
    else 
        finalStep += COS_SIZE;

    while (step <= finalStep) {
        select {
            case microstepTimer when timerafter(microstepTime+microPeriod) :> void:

            	microstepTime += microPeriod;

                //0 - 90 degrees
                //winding 0  : -1 => 0  --  I Falling
                //winding 1  :  0 => 1  --  I Rising
                if ((step >= 0) && (step < COS_SIZE)) {
                    currentReference[0] = -((step256[step]*RESOLUTION)>>15);
                    currentReference[1] = (step256[(COS_SIZE-1)-step]*RESOLUTION)>>15;
                    decayMode[0] = DECAY_MODE;
                    decayMode[1] = slow;
                }
                //90-180 degrees
                //winding 0  :  0 => 1  --  I Rising
                //winding 1  :  1 => 0  --  I Falling
                if ((step >= COS_SIZE) && (step < (COS_SIZE*2))) {
                    currentReference[0] = (step256[(COS_SIZE*2-1)-step]*RESOLUTION)>>15;
                    currentReference[1] = (step256[step-COS_SIZE]*RESOLUTION)>>15;
                    decayMode[0] = slow;
                    decayMode[1] = DECAY_MODE;                   
                }
                //180-270 degrees
                //winding 0  :  1 => 0  --  I Falling
                //winding 1  :  0 => -1 --  I Rising
                if ((step >= (COS_SIZE*2)) && (step < (COS_SIZE*3))) {
                    currentReference[0] = (step256[step-COS_SIZE*2]*RESOLUTION)>>15;
                    currentReference[1] = -((step256[(COS_SIZE*3-1)-step]*RESOLUTION)>>15);
                    decayMode[0] = DECAY_MODE; 
                    decayMode[1] = slow;      
                }
                //270-360 degrees
                //winding 0  :  0 => -1 --  I Rising
                //winding 1  : -1 => 0  --  I Falling
                if ((step >= (COS_SIZE*3)) && (step < (COS_SIZE*4))) {
                    currentReference[0] = -((step256[(COS_SIZE*4-1)-step]*RESOLUTION)>>15);
                    currentReference[1] = -((step256[step - COS_SIZE*3]*RESOLUTION)>>15);
                    decayMode[0] = slow;
                    decayMode[1] = DECAY_MODE;
                }
                
                if (step == COS_SIZE*4) {
                    currentReference[0] = -((step256[step - COS_SIZE*4]*RESOLUTION)>>15);
                    currentReference[1] = (step256[(COS_SIZE*5-1)-step]*RESOLUTION)>>15;
                    decayMode[0] = DECAY_MODE;
                    decayMode[1] = slow;
                }
                
                //if we're starting up just raise the current in one coil to begin with
                if (startupFlag == 1) {
                    currentReference[0] = -((step256[(COS_SIZE-1)-step]*RESOLUTION)>>15);
                    currentReference[1] = 0;
                    decayMode[0] = slow;
                    decayMode[1] = slow;
                    //if we've finished starting up then reset the step count and turn off the flag
                    if (step == COS_SIZE - STEP_SIZE) {
                        step = 0;
                        startupFlag = 0;
                    }
                }
                
                //If direction is reverse then switch the coils for the current reference
                if (direction) {
                    int temp1 = currentReference[0];
                    currentReference[0] = currentReference[1];
                    currentReference[1] = temp1;
                }

                #ifdef OPEN_LOOP_CHOPPING
                    for (int j = 0; j < 2; j++) {
                        if (currentReference[j] > OPEN_LOOP_CHOPPED_DUTY_CYCLE)
                            currentReference[j] = OPEN_LOOP_CHOPPED_DUTY_CYCLE;
                        else if (currentReference[j] < -OPEN_LOOP_CHOPPED_DUTY_CYCLE)
                            currentReference[j] = -OPEN_LOOP_CHOPPED_DUTY_CYCLE;
                    }
                #endif
                
				refCurrentADC[0] = currentReference[0];
				refCurrentADC[1] = currentReference[1];
                
                step += STEP_SIZE; 
                break;    
                
            //Take the ADC values, apply the PI controller and set the pwm accordingly    
            case adc_timer when timerafter(adc_time) :> void:
				{
					int PIOutput[2] = {0,0};

					//int current[2];
					//{ current[0], current[1] } = getWindingADC(c_adc);
					//PIOutput[0] = applyPI(refCurrentADC[0], current[0], PISum0, PILastError0, output1 );
					//PIOutput[1] = applyPI(refCurrentADC[1], current[1], PISum1, PILastError1, output2 );

					// We do not do torque control because of the complexity of the dual channel ADC and MUX
					// on the motor control board
					PIOutput[0] = refCurrentADC[0];
					PIOutput[1] = refCurrentADC[1];

					setWindingPWM(refCurrentADC, PIOutput, decayMode, c_pwm);
#ifdef USE_XSCOPE
					xscope_probe_data(0,refCurrentADC[0]);
					xscope_probe_data(1,refCurrentADC[1]);
					xscope_probe_data(4,PIOutput[0]);
					xscope_probe_data(5,PIOutput[1]);
#endif
	                adc_time += ADC_PERIOD;
                }
                break;
         }
    }
}


void motor(chanend c_pwm, chanend c_control, chanend c_wd, streaming chanend c_adc) {
	// The duration of the step period determines the RPM
	unsigned int stepPeriod = 5000000;

    unsigned int step = 0, noSteps = 0, stepCounter = 0;

    unsigned  doSteps = 0, doSpeed = 1;

    int direction = FORWARD;

    int cmd;
	timer speed_demo_timer, t;
    int time, speed_demo_time;
    int microStepPeriod;
    
#ifdef USE_XSCOPE
    xscope_config_io(XSCOPE_IO_BASIC);
    xscope_register(8,
    XSCOPE_CONTINUOUS, "ADCReference 1", XSCOPE_UINT, "Value",
    XSCOPE_CONTINUOUS, "ADCReference 2", XSCOPE_UINT, "Value",
    XSCOPE_CONTINUOUS, "ADC Winding 1", XSCOPE_UINT, "Value",
    XSCOPE_CONTINUOUS, "ADC Winding 2", XSCOPE_UINT, "Value",
    XSCOPE_CONTINUOUS, "PI Output1", XSCOPE_UINT, "Value",
    XSCOPE_CONTINUOUS, "PI Output2", XSCOPE_UINT, "Value",
    XSCOPE_CONTINUOUS, "lastRef0", XSCOPE_UINT, "Value",
    XSCOPE_CONTINUOUS, "lastRef1", XSCOPE_UINT, "Value");
#endif
    
    //delay to make sure the ADC is calibrated before starting output and watchdog is enabled
    t :> time;
    t when timerafter (time+200000000) :> void;
    c_wd <: WD_CMD_START;

    printstrln("Starting");
    t :> time;

    speed_demo_timer :> speed_demo_time;
    microStepPeriod = stepPeriod / MICROSTEP_COUNT;
    
    while (1) {
        #pragma ordered
        select {
            case c_control :> cmd:            
                if (cmd == CMD_SET_MOTOR_SPEED) {
                    doSteps = 0;
                    doSpeed = 1;
                    c_control :> stepPeriod;
                    c_control :> direction; 
                    microStepPeriod = stepPeriod / MICROSTEP_COUNT;
                }
                if (cmd == CMD_NUMBER_STEPS) {
                    doSpeed = 0;
                    doSteps = 1;
                    c_control :> stepPeriod;
                    c_control :> noSteps;
                    c_control :> direction;
                    microStepPeriod = stepPeriod / MICROSTEP_COUNT;
                }
                if (cmd == CMD_MOTOR_OFF) {
                    enum decay tempDecay[2] = { slow, slow };
                    int tempRef[2] = { 0, 0 };
                    int tempAdc[2] = { 0, 0 };

                    //reset step count so we don't startup in the middle of a step
                    step = 0;
                    startupFlag = 1;
                    
                    //stop controller making any more steps
                    doSteps = 0;
                    doSpeed = 0;

                    setWindingPWM(tempAdc, tempRef, tempDecay, c_pwm);
                }
                break;
                    
            //loop for speed
            case t when timerafter(time) :> void:
                
                if (doSpeed) {
                    singleStep(c_pwm, step, direction, microStepPeriod, c_adc);
                }
                else if (doSteps) {
                    if (stepCounter == noSteps) {
                        doSteps = 0;
                        stepCounter = 0;
                    }
                    else {
                        singleStep(c_pwm, step, direction, microStepPeriod, c_adc);
                        stepCounter++;
                    }
                }
                time += stepPeriod;
                break;

        }
    }
}


int main(void) {

    chan c_control, c_pwm, c_wd, c_adc_trig[1];
    streaming chan  c_adc[1];

	par {
	    on stdcore[INTERFACE_CORE] : controller(c_control);
    	on stdcore[MOTOR_CORE] : motor(c_pwm, c_control, c_wd, c_adc[0]);
    	
		on stdcore[MOTOR_CORE] : pwmSingleBitPortTrigger(c_adc_trig[0], c_pwm, pwm_clk, motor_ports, 8, RESOLUTION, TIMESTEP, 1);

		on stdcore[MOTOR_CORE] : adc_7265_triggered(c_adc, c_adc_trig, adc_clk, ADC_SCLK, ADC_CNVST, ADC_DATA_A, ADC_DATA_B, ADC_MUX );

    	on stdcore[INTERFACE_CORE] : do_wd(c_wd, i2c_wd);
	}
	return 0;
}

