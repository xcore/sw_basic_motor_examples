// Copyright (c) 2011, XMOS Ltd., All rights reserved
// This software is freely distributable under a derivative of the
// University of Illinois/NCSA Open Source License posted in
// LICENSE.txt and at <http://github.xcore.com/>

#define CMD_RAMP 0
#define CMD_SET_MOTOR_SPEED 1
#define CMD_GET_MOTOR_SPEED 2
#define CMD_POSITION 3
#define DC_MOTOR

typedef struct ramp_parameters {
    int startSpeed;
    int targetSpeed;
    int direction;
    int rampPeriod;       //Multiple of period for main loop i.e. if main period = 10ns and rampPeriod = 10, actual period for ramp is 100ns
    int acceleration;     //This is the speed increase per ramp_period, If ramping down, set this negative
} ramp_parameters;


