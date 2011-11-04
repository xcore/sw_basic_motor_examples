// Copyright (c) 2011, XMOS Ltd., All rights reserved
// This software is freely distributable under a derivative of the
// University of Illinois/NCSA Open Source License posted in
// LICENSE.txt and at <http://github.xcore.com/>

#ifndef CONFIG_H_
#define CONFIG_H_

// Define where everything is
#define INTERFACE_CORE 0
#define MOTOR_CORE 1

//Define ADC stuff
#define ADC_TRIG_TOKEN 1
#define ADC_CALIB_POINTS 1024
#define ADC_PERIOD 200000

#define INITIAL_SET_SPEED 50
#define PWM_INC_DEC_VAL 10
#define MAX_RPM 250
#define MIN_RPM 10

#define NUM_MOTORS 2

#endif /* CONFIG_H_ */
