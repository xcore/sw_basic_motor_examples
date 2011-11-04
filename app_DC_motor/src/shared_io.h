// Copyright (c) 2011, XMOS Ltd., All rights reserved
// This software is freely distributable under a derivative of the
// University of Illinois/NCSA Open Source License posted in
// LICENSE.txt and at <http://github.xcore.com/>
                                 
#ifndef SHARED_IO_H_
#define SHARED_IO_H_

#include <xccompat.h>
#include "config.h"
#include "lcd.h"

	// Individual command interfaces

	#define CMD_GET_VALS	1
	#define CMD_GET_IQ		2
	#define CMD_SET_SPEED	3
    #define CMD_DIR         4
	#define STEP_SPEED 		10
	#define _30_Msec		3000000
	#define MSec 100000

	void display_shared_io_motor( chanend c_lcd1, chanend c_lcd2, REFERENCE_PARAM(lcd_interface_t, p), port in btns);

#endif /* SHARED_IO_H_ */
