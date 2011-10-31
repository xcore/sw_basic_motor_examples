// Copyright (c) 2011, XMOS Ltd., All rights reserved
// This software is freely distributable under a derivative of the
// University of Illinois/NCSA Open Source License posted in
// LICENSE.txt and at <http://github.xcore.com/>


#include <xs1.h>
#include <stdlib.h>
#include <print.h>
//#include <xscope.h>
#include "config.h"
#include "adc.h"


static int Ia_calib = 0, Ib_calib = 0, Ic_calib = 0, Id_calib = 0, Ie_calib = 0, If_calib = 0;


void do_6adc_calibration( chanend c_adc )
{
	unsigned a,b,c,d,e,f;
	for (int i = 0; i < ADC_CALIB_POINTS; i++)
	{
		/* get ADC reading */
		c_adc <: 6;
		slave
		{
			c_adc :> a;
			c_adc :> b;
			c_adc :> c;
			c_adc :> d;
			c_adc :> e;
			c_adc :> f;
		}
		Ia_calib += a;
		Ib_calib += b;
		Ic_calib += c;
		Id_calib += d;
		Ie_calib += e;
		If_calib += f;
	}
	    Ia_calib = (Ia_calib >> 6);
		Ib_calib = (Ib_calib >> 6);
		Ic_calib = (Ic_calib >> 6);
	    Id_calib = (Id_calib >> 6);
		Ie_calib = (Ie_calib >> 6);
		If_calib = (If_calib >> 6);
}


{unsigned, unsigned, unsigned} get_adc_vals_raw( chanend c_adc )
{
	unsigned a, b, c;

	c_adc <: 0;

	slave
	{
		c_adc :> a;
		c_adc :> b;
		c_adc :> c;
	}

	return {a,b,c};
}

{int, int, int, int, int, int} get_6adc_vals_calibrated_int16( chanend c_adc )
{
	unsigned a, b, c, d, e, f;
	int Ia, Ib, Ic, Id, Ie, If;

	/* request and then receive adc data */
	c_adc <: 6;

	slave
	{
		c_adc :> a;
		c_adc :> b;
		c_adc :> c;
		c_adc :> d;
		c_adc :> e;
		c_adc :> f;
	}
	/* apply calibration offset */

	Ia = a - Ia_calib;
  	Ib = b - Ib_calib;
	Ic = c - Ic_calib;
	Id = d - Id_calib;

	return {Ia, Ib, Ic, Id, Ie, If};
}


static void configure_adc_ports_ltc1408(clock clk, port out SCLK, buffered out port:32 CNVST, in buffered port:32 DATA)
{
    configure_clock_rate_at_least(clk, 100, 10);

    configure_port_clock_output(SCLK, clk);
    configure_out_port(CNVST, clk, 0);
	configure_in_port(DATA, clk);

    set_port_sample_delay( DATA ); // clock in on falling edge

    start_clock(clk);
}


#pragma unsafe arrays
static void adc_get_data_ltc1408_singleshot( int adc_val[], unsigned offset, buffered out port:32 CNVST, in buffered port:32 DATA, clock clk )
{

	unsigned val1 = 0, val3 = 0, val5 = 0;

	stop_clock(clk);

	#define ADC_CONVERSION_TRIG (1<<31)
    CNVST <: ADC_CONVERSION_TRIG;
    clearbuf(DATA);
    start_clock(clk);
	DATA :> val1;
	CNVST <: 0;
	DATA :> val1;
	val1 = bitrev(val1);
	DATA :> val3;
	val3 = bitrev(val3);
	DATA :> val5;
    val5 = bitrev(val5);

	adc_val[offset+0] = 0x3FFF & (val1 >> 16);
	adc_val[offset+1] = 0x3FFF & (val1 >>  0);
	adc_val[offset+2] = 0x3FFF & (val3 >> 16);
	adc_val[offset+3] = 0x3FFF & (val3 >>  0);
	adc_val[offset+4] = 0x3FFF & (val5 >> 16);
	adc_val[offset+5] = 0x3FFF & (val5 >>  0);


}

/*
void adc_ltc1408_triggered( chanend c_adc, clock clk, port out SCLK, buffered out port:32 CNVST, in buffered port:32 DATA, chanend c_trig, chanend ?c_logging0, chanend ?c_logging1, chanend ?c_logging2)
{
	int adc_val[6];
	int cmd;
	unsigned char ct;

	timer t;
	unsigned ts;

	configure_adc_ports_ltc1408(clk, SCLK, CNVST, DATA);

	while (1)
	{
		select
		{

		case inct_byref(c_trig, ct):
			if (ct == ADC_TRIG_TOKEN)
			{
				t :> ts;
				t when timerafter(ts + 1740) :> ts;
				adc_get_data_ltc1408_singleshot( adc_val, 0, CNVST, DATA, clk );
			}
			break;
		case c_adc :> cmd:
			switch (cmd)
			{
			case 0:
				master {
					c_adc <: adc_val[0];
					c_adc <: adc_val[1];
					c_adc <: adc_val[2];
				}
				break;
			case 3:
				master {
					c_adc <: adc_val[3];
					c_adc <: adc_val[4];
					c_adc <: adc_val[5];
				}
				break;
			case 6:
				master {
					c_adc <: adc_val[0];
					c_adc <: adc_val[1];
					c_adc <: adc_val[2];
					c_adc <: adc_val[3];
					c_adc <: adc_val[4];
					c_adc <: adc_val[5];
				}
				break;
			}
		break;
		}

	}
}*/

/*
 * Need to use get 16 calibrated with c_adc to get values
 */

void adc_with_scope(chanend c_adc, clock clk, port out SCLK, buffered out port:32 CNVST, in buffered port:32 DATA)
{
	int adc_val[6];
	int cmd;

	timer t;
	int time;

	configure_adc_ports_ltc1408(clk, SCLK, CNVST, DATA);

	t :> time;

	//Calibrate the ADC
	for (int i = 0; i < ADC_CALIB_POINTS; i++)
	{
		//get ADC reading
		adc_get_data_ltc1408_singleshot( adc_val, 0, CNVST, DATA, clk );
        t when timerafter (time+2000) :> time;
        Ia_calib += adc_val[0];
		Ib_calib += adc_val[1];
		Ic_calib += adc_val[2];
		Id_calib += adc_val[3];
	}
        //If number of calibration points is changed then this needs changing
	    Ia_calib = (Ia_calib >> 10);
		Ib_calib = (Ib_calib >> 10);
		Ic_calib = (Ic_calib >> 10);
	    Id_calib = (Id_calib >> 10);
	time += ADC_PERIOD;
	while (1)
	{
		select
		{

		case t when timerafter (time) :> time:
			adc_get_data_ltc1408_singleshot( adc_val, 0, CNVST, DATA, clk );
			adc_val[0] = adc_val[0] - Ia_calib;
			adc_val[1] = adc_val[1] - Ib_calib;
			adc_val[2] = adc_val[2] - Ic_calib;
			adc_val[3] = adc_val[3] - Id_calib;
			/*xscope_probe_data(0, adc_val[0] );
			xscope_probe_data(1, adc_val[1] );*/
			time += ADC_PERIOD;
			break;
		case c_adc :> cmd:
				master {
					c_adc <: adc_val[0];
					c_adc <: adc_val[1];
					c_adc <: adc_val[2];
					c_adc <: adc_val[3];
					c_adc <: adc_val[4];
					c_adc <: adc_val[5];
				}
		break;
		}

	}
}

