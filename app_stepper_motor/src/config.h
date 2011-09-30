// Copyright (c) 2011, XMOS Ltd, All rights reserved
// This software is freely distributable under a derivative of the
// University of Illinois/NCSA Open Source License posted in
// LICENSE.txt and at <http://github.xcore.com/>


#ifndef CONFIG_H_
#define CONFIG_H_

// Define where everything is
#define INTERFACE_CORE 0
#define MOTOR_CORE 1

//Define ADC stuff

#define ADC_CALIB_POINTS 512
#define ADC_PERIOD 20000
#define ADC_NUMBER_OF_TRIGGERS NUMBER_OF_MOTORS

#define CHOPPING_DECREMENT 10;

#define NUMBER_OF_MOTORS 2;

#define RESOLUTION 256              //Resolution of PWM, normally 256                                          
#define TIMESTEP 12                //Determines frequency of PWM
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
#define DECAY_MODE fast

#define V_REF 24
#define R_WINDING 44

//Only used if OPEN_LOOP_CHOPPING or CLOSED_LOOP_CHOPPING defined
#define IMAX 600      //max phase current in milliamps

//Open loop limits current according to theoretical current from R and V
//Closed loop takes ADC reading and limits current above IMAX
//#define OPEN_LOOP_CHOPPING 1
//#define CLOSED_LOOP_CHOPPING 1
//Maximum limited current in terms of ADC
//since 14bit input and 1.5A range
#define IMAX_ADC ((16384*IMAX)/1500)


#endif /* CONFIG_H_ */


