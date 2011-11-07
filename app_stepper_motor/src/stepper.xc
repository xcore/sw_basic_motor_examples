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
on stdcore[MOTOR_CORE] : out buffered port:32 motor_ports[8] = { PORT_M1_HI_A, PORT_M2_HI_A, PORT_M1_HI_B, PORT_M1_HI_C, PORT_M1_LO_A, PORT_M2_LO_A, PORT_M1_LO_B, PORT_M1_LO_C};

//Ports for ADC
on stdcore[MOTOR_CORE]: out port ADC_SCLK = PORT_ADC_CLK;
on stdcore[MOTOR_CORE]: port ADC_CNVST = PORT_ADC_CONV;
on stdcore[MOTOR_CORE]: buffered in port:32 ADC_DATA_A = PORT_ADC_MISOA;
on stdcore[MOTOR_CORE]: buffered in port:32 ADC_DATA_B = PORT_ADC_MISOB;
on stdcore[MOTOR_CORE]: out port ADC_MUX = PORT_ADC_MUX;
on stdcore[MOTOR_CORE]: in port ADC_SYNC_PORT = XS1_PORT_16A;
on stdcore[MOTOR_CORE]: clock adc_clk = XS1_CLKBLK_2;

#define RESOLUTION 256              //Resolution of PWM, normally 256                                          
#define TIMESTEP 12                 //Determines frequency of PWM
#define PERIOD 100000               //Initial step period

#define ADC_PERIOD 20000

#define PI_ANTIWINDUP 500           //Anti wind-up gain
#define PIGAIN1 1000                
#define PIGAIN2 50

#define USE_XSCOPE

// STEP_SIZE defines the number of microsteps to be used:
// COS_SIZE / STEP_SIZE gives the number of microsteps
// {256,128,64,32,16,8,4,2 or 1} are valid values
#define STEP_SIZE 16
#define COS_SIZE 256

//Sets the decay mode
//slow = fixed decay (always slow decay)
//fast = alternating decay (slow during rising current, fast during falling)
//fast decay provides higher speeds with slightly noisier operation
#define DECAY_MODE slow

#define V_REF 12
#define R_WINDING 44

//Only used if OPEN_LOOP_CHOPPING or CLOSED_LOOP_CHOPPING defined
#define IMAX 200      //max phase current in milliamps

//Open loop limits current according to theoretical current from R and V
//Closed loop takes ADC reading and limits current above IMAX
//#define OPEN_LOOP_CHOPPING 1
//#define CLOSED_LOOP_CHOPPING 1
//Maximum limited current in terms of ADC
#define IMAX_ADC ((8192*IMAX)/6250)


unsigned iScaled = (((8192*V_REF*1000)/(6250*R_WINDING)));
int iMaxADC = IMAX_ADC;
int choppedDutyCycle = ((IMAX << 13 * 255) / (((V_REF << 13) / (R_WINDING))*1000));
int temp1;
int direction = FORWARD, testvar, maxRefI = 255, output1 = 0, output2 = 0, debugValue = 0,debugValue1 = 0, finalStep, startupFlag = 1;

int windingCurrent[2] = {0,0}, adc_last[4] = {0,0,0,0}, refCurrentADC[2];

unsigned duty[8] = {0,0,0,0,0,0,0,0};

enum decay {fast, slow};
enum decay decayMode[2];

int PISum0 = 0, PISum1 = 0, PILastError0 = 0, PILastError1 = 0, PIOutput[2] = {0,0};

timer microstepTimer, adc_timer;
int microstepTime, adc_time;


void controller(chanend c_control) {
    
    timer ti;
    int titime;
    ti :> titime;
    
    c_control <: CMD_SET_MOTOR_SPEED;
    c_control <: 1000000;   //Step Period 
    c_control <: FORWARD;     //Direction
    
    
}

 
//Can be called for both windings, just pass different sum and error vars.
//Should return a voltage output

//TODO Sort out previous outputs
int applyPI(int referenceI,int actualI, int &sum, int &lastError, int &lastOutput) {
   
    int currentError;

	currentError = referenceI - actualI;

    if (referenceI == 0 ) {
        sum = 0;
        currentError = 0;
        lastError = 0;
    }
    else {
        sum += ((currentError * PIGAIN1 - lastError * PIGAIN2)>>13)-(((sum-lastOutput) * PI_ANTIWINDUP)>>15);
    }
    
	lastOutput = sum;
	
	if (lastOutput > 255) {
	    lastOutput = 255;
	    sum = 255;
	}
	else if (lastOutput < -255) {
	    lastOutput = -255;
	    sum = -255;
	}
	
    /*//check the values from ADC and limit current if necessary
    #ifdef CLOSED_LOOP_CHOPPING
        for (int j = 0; j < 2; j++) {
            if (actualI[j] >= iMaxADC)
                lastOutput =                 
    #endif*/
    
    
    lastError = currentError;

	return lastOutput;
}

/*
* Given PIOutput from the PI controller ( two values, one for each winding)
* transform into output duty cycles for each high side MOSFET (4)
* winding 0 on A, 2A ; winding 1 on B, C
* 
* duty consists of high ports x 4 followed by low ports x 4
*/
void setWindingPWM(int PIOutput[], enum decay decayMode[],  chanend c_pwm) {

    
    //Winding 0
    if (refCurrentADC[0] > 0) {
        if (PIOutput[0] < 0)
            PIOutput[0] = 0;
        if (decayMode[0] == slow) {
            duty[4] = 255;
            duty[5] = 0;
        }
        else if (decayMode[0] == fast) {
            duty[4] = PIOutput[0];
            duty[5] = 0;
        }
        duty[0] = 0;
        duty[1] = PIOutput[0];
    }
    else if (refCurrentADC[0] < 0) {
        if (PIOutput[0] > 0)
            PIOutput[0] = 0;
            
        if (decayMode[0] == slow) {
            duty[4] = 0;
            duty[5] = 255;
        }
        else if (decayMode[0] == fast) {
            duty[4] = 0;
            duty[5] = -PIOutput[0];
        }
        duty[1] = 0;
        duty[0] = -PIOutput[0];
    }
    else {
        duty[4] = 255;
        duty[5] = 255;
         
        duty[1] = 0;
        duty[0] = 0;
    }
    
    //Winding 1
    if (refCurrentADC[1] >0) {
        if (PIOutput[1] < 0)
            PIOutput[1] = 0;
        if (decayMode[1] == slow) {
            duty[6] = 255;
            duty[7] = 0;
        }
        else if (decayMode[1] == fast) {
            duty[6] = PIOutput[1];
            duty[7] = 0;
        } 
        duty[2] = 0;
        duty[3] = PIOutput[1];
    }
    else if (refCurrentADC[1] < 0) {
        if (PIOutput[1] > 0)
            PIOutput[1] = 0;
        if (decayMode[1] == slow) {
            duty[6] = 0;
            duty[7] = 255;
        }
        else if (decayMode[1] == fast) {
            duty[6] = 0;
            duty[7] = -PIOutput[1];
        }
        duty[2] = -PIOutput[1];
        duty[3] = 0;
    }
    else {
        duty[6] = 255;
        duty[7] = 255;
       
        duty[2] = 0;
        duty[3] = 0;
    }

    pwmSingleBitPortSetDutyCycle(c_pwm, duty, 8);

    
}

void getWindingADC ( streaming chanend c_adc) {
    int adc[4] = {200,200,200,200};
    //int temp;
    
    //Get the adc values
//    c_adc <: 6;
//    slave {
//        c_adc :> adc[0];
//        c_adc :> adc[1];
//        c_adc :> adc[2];
//        c_adc :> adc[3];
//        c_adc :> temp;
//        c_adc :> temp;
//    }
    
    //make sure we only use the positive part
    for (int i = 0; i < 4; i++)
        if (adc[i] < 0)
            adc[i] = 0;
    
    //If low[0] is on and the reference != 0 use adc[0] otherwise use negative of adc[3]
    if (duty[4] != 0 && refCurrentADC[0] != 0)
        windingCurrent[0] = adc[0];
    else
        windingCurrent[0] = -adc[3];

    //If low[2] on and reference != 0 use adc[1] else negative adc[2]
    if (duty[6] != 0 && refCurrentADC[1] != 0)
        windingCurrent[1] = adc[1];
    else 
        windingCurrent[1] = -adc[2];

    #ifdef USE_XSCOPE    
        xscope_probe_data(2, windingCurrent[0]);
        xscope_probe_data(3, windingCurrent[1]);
    #endif
        
}

/*
** 0	0 	0	255
** 255	0	0	0
** 0	0	255	0
** 0	255	0	0
** 0	0	0	255
*/    

void singleStep(chanend c_pwm, unsigned int &step, unsigned microPeriod, streaming chanend c_adc) {

    int currentReference[2];
    
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
            
            case microstepTimer when timerafter (microstepTime+microPeriod) :> microstepTime:
              
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
                    temp1 = currentReference[0];
                    currentReference[0] = currentReference[1];
                    currentReference[1] = temp1;
                }

                #ifdef OPEN_LOOP_CHOPPING
                    for (int j = 0; j < 2; j++) {
                        if (currentReference[j] > choppedDutyCycle)
                            currentReference[j] = choppedDutyCycle;
                        else if (currentReference[j] < -choppedDutyCycle)
                            currentReference[j] = -choppedDutyCycle;
                    }
                #endif
                
                #ifdef CLOSED_LOOP_CHOPPING
                    for (int j = 0; j < 2; j++) {
                        //Check if currentReference has dropped below the limiting value and reset the limit flag if so
                        if (((limitFlag[j] > 0) && (currentReference[j] < limitFlag[j])) || ((limitFlag[j] < 0) && (currentReference[j] > limitFlag[j]))) {
                            limitFlag[j] = 0;
                        }
                        //otherwise keep the reference current at the limit
                        if (limitFlag[j] != 0) {
                            currentReference[j] = limitFlag[j];
                        }
                    }
                #endif
                
                //Convert the currentReference to a scaled value on the same scale as the ADC
                //Winding 0
                if (currentReference[0] < 0)
                    refCurrentADC[0] = -((-currentReference[0] * iScaled) >> 8);
                else if (currentReference[0] == 0)
                    refCurrentADC[0] = 0;
                else
                    refCurrentADC[0] = (currentReference[0] * iScaled) >> 8;
                //Winding 1    
                if (currentReference[1] < 0)
                    refCurrentADC[1] = -((-currentReference[1] * iScaled) >> 8);
                else if (currentReference[1] == 0)
                    refCurrentADC[1] = 0;
                else
                    refCurrentADC[1] = ((currentReference[1] * iScaled) >> 8);
                
                //Store the last reference current so that we can revert to it if chopping closed loop
                #ifdef CLOSED_LOOP_CHOPPING
                    lastRefCurrent[0] = currentReference[0];
                    lastRefCurrent[1] = currentReference[1];  
                #endif
                
                step += STEP_SIZE; 
                break;    
                
            //Take the ADC values, apply the PI controller and set the pwm accordingly    
            case adc_timer when timerafter(adc_time) :> void:
                getWindingADC(c_adc);
                
                #ifdef CLOSED_LOOP_CHOPPING
                    for (int j = 0; j < 2; j++) {
                        if (((windingCurrent[j] >= iMaxADC) || (windingCurrent[j] <= -iMaxADC)) && (limitFlag[j] == 0))
                            limitFlag[j] = lastRefCurrent[j];
                    }
                #endif

                PIOutput[0] = applyPI(refCurrentADC[0], windingCurrent[0], PISum0, PILastError0, output1 );
                PIOutput[1] = applyPI(refCurrentADC[1], windingCurrent[1], PISum1, PILastError1, output2 );
                
                #ifdef USE_XSCOPE
                    xscope_probe_data(0,refCurrentADC[0]);
                    xscope_probe_data(1,refCurrentADC[1]); 

                    xscope_probe_data(4,PIOutput[0]);
                    xscope_probe_data(5,PIOutput[1]);
                #endif
                
                setWindingPWM(PIOutput, decayMode, c_pwm);

                
                adc_time += ADC_PERIOD;
                break;
         }
    }
}


void motor(chanend c_pwm, chanend c_control, chanend c_wd, streaming chanend c_adc) {
    unsigned int step = 0, noSteps = 0, stepCounter = 0, stepPeriod = 5000000, doSteps = 0, doSpeed = 1;
    int tempRef[2];
    enum decay tempDecay[2];
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
    
    microstepTimer :> microstepTime;
    //delay to make sure the ADC is calibrated before starting output and watchdog is enabled
    t :> time;
    t when timerafter (time+200000000) :> time;
    c_wd <: WD_CMD_START;
    
    time += PERIOD;
    adc_timer :> adc_time;
    adc_time +=100000;
    
    speed_demo_timer :> speed_demo_time;
    microStepPeriod = stepPeriod / ((COS_SIZE) / STEP_SIZE);
    
    while (1) {
        #pragma ordered
        select {
            case c_control :> cmd:            
                if (cmd == CMD_SET_MOTOR_SPEED) {
                    doSteps = 0;
                    doSpeed = 1;
                    c_control :> stepPeriod;
                    c_control :> direction; 
                    microStepPeriod = stepPeriod / ((COS_SIZE) / STEP_SIZE);
                }
                if (cmd == CMD_NUMBER_STEPS) {
                    doSpeed = 0;
                    doSteps = 1;
                    c_control :> stepPeriod;
                    c_control :> noSteps;
                    c_control :> direction;
                    microStepPeriod = stepPeriod / ((COS_SIZE) / STEP_SIZE);
                }
                if (cmd == CMD_MOTOR_OFF) {
                    //reset step count so we don't startup in the middle of a step
                    step = 0;
                    startupFlag = 1;
                    
                    //stop controller making any more steps
                    doSteps = 0;
                    doSpeed = 0;
                    
                    //TODO setup vars on init
                    for (int i = 0; i < 2; i++) {
                        tempRef[i] = 0;
                        tempDecay[i] = slow;
                    }
                    setWindingPWM(tempRef, tempDecay, c_pwm);
                }
                break;
                    
            //loop for speed
            case t when timerafter (time) :> void:
                
                if (doSpeed) {
                    singleStep(c_pwm, step, microStepPeriod, c_adc);
                }
                else if (doSteps) {
                    if (stepCounter == noSteps) {
                        doSteps = 0;
                        stepCounter = 0;
                    }
                    else {
                        singleStep(c_pwm, step, microStepPeriod, c_adc);
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

