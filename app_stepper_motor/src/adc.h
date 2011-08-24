// Copyright (c) 2011, XMOS Ltd., All rights reserved
// This software is freely distributable under a derivative of the
// University of Illinois/NCSA Open Source License posted in
// LICENSE.txt and at <http://github.xcore.com/>


#include <xclib.h>
#include "config.h"

void do_6adc_calibration( chanend c_adc );

{int, int, int, int, int, int} get_6adc_vals_calibrated_int16( chanend c_adc );

void adc_ltc1408_triggered( chanend c_adc, clock clk, port out SCLK, buffered out port:32 CNVST, in buffered port:32 DATA, chanend c_trig, chanend ?c_logging0, chanend ?c_logging1, chanend ?c_logging2);

void adc_with_scope(chanend c_adc, clock clk, port out SCLK, buffered out port:32 CNVST, in buffered port:32 DATA);
