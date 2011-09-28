/*
 * adc_7265.xc
 *
 *  Created on: Jul 6, 2011
 *  Author: A SRIKANTH
 */

#include <xs1.h>
#include <platform.h>
#include <xclib.h>
//#include <adc_common.h>
#include "config.h"
#include <adc_7265.h>

//#define ADC_FILTER_7265


#pragma xta command "analyze loop adc_7265_main_loop"
#pragma xta command "set required - 40 us"

// This array determines the mapping from trigger channel to which analogue input to select in the ADC mux
static int trigger_channel_to_adc_mux[2] = { 2, 0 };

// These are the calibration values
static unsigned calibration[3][2];

// Mode to say if we are currently calibrating the ADC
static int calibration_mode[2];

// Accumultor for the calibration average
static int calibration_acc[3][2];

static void configure_adc_ports_7265(clock clk, out port SCLK, port CNVST, in buffered port:32 DATA_A, in buffered port:32 DATA_B, out port MUX)
{
	// configure the clock to be 16MHz
    configure_clock_rate_at_least(clk, 16, 1);
    configure_port_clock_output(SCLK, clk);

    // ports require postive strobes, but the ADC needs a negative strobe. use the port pin invert function
    // to satisfy both
    configure_out_port(CNVST, clk, 1);
    set_port_inv(CNVST);
    CNVST <: 0;

    // configure the data ports to strobe data in to the buffer using the serial clock
	configure_in_port_strobed_slave(DATA_A, CNVST, clk);
	configure_in_port_strobed_slave(DATA_B, CNVST, clk);

	// sample the data in on falling edge of the serial clock
    set_port_sample_delay( DATA_A );
    set_port_sample_delay( DATA_B );

    // start the ADC serial clock port
    start_clock(clk);
}

#pragma unsafe arrays
static void adc_get_data_7265( int adc_val[], unsigned channel, port CNVST, in buffered port:32 DATA_A, in buffered port:32 DATA_B, out port MUX )
{
	unsigned val1 = 0, val3 = 0;
	unsigned ts;

	MUX <: channel;

	CNVST <: 1 @ts;
	ts += 16;
	CNVST @ts <: 0;

	endin(DATA_A);
	endin(DATA_B);

	DATA_A :> val1;
	DATA_B :> val3;

	val1 = bitrev(val1);
	val3 = bitrev(val3);

	val1 = val1 >> 2;
	val3 = val3 >> 2;

	val1 = 0x00000FFF & val1;
	val3 = 0x00000FFF & val3;

#ifdef ADC_FILTER_7265
	adc_val[0] = (adc_val[0] >> 1) + (val1 >> 1);
	adc_val[1] = (adc_val[1] >> 1) + (val3 >> 1);
#else
	adc_val[0] = val1;
	adc_val[1] = val3;
#endif

}
#pragma unsafe arrays
void adc_7265_4val_triggered( chanend c_adc, chanend c_trig, clock clk, out port SCLK, port CNVST, in buffered port:32 DATA_A, in buffered port:32 DATA_B, port out MUX )
{
	int adc_val[3][2];
	int cmd;
	unsigned char ct;

	timer t;
	unsigned ts;

	set_thread_fast_mode_on();

	configure_adc_ports_7265( clk, SCLK, CNVST, DATA_A, DATA_B, MUX );

	for (unsigned int c=0; c<3; ++c) {
		adc_val[c][0] = 0;
		adc_val[c][1] = 0;
	}
	for (int i=0; i < 512; i++) {
		adc_get_data_7265( adc_val[0], 0, CNVST, DATA_A, DATA_B, MUX );
		adc_get_data_7265( adc_val[1], 2, CNVST, DATA_A, DATA_B, MUX );
		adc_get_data_7265( adc_val[2], 4, CNVST, DATA_A, DATA_B, MUX );
		
	    calibration_acc[0][0] += adc_val[0][0];
	    calibration_acc[0][1] += adc_val[0][1];
	    calibration_acc[1][0] += adc_val[1][0];
	    calibration_acc[1][1] += adc_val[1][1];
	    calibration_acc[2][0] += adc_val[2][0];
        calibration_acc[2][1] += adc_val[2][1]; 
	}
	
    calibration[0][0] = calibration_acc[0][0] / 512;
    calibration[0][1] = calibration_acc[0][1] / 512;
    calibration[1][0] = calibration_acc[1][0] / 512;
    calibration[1][1] = calibration_acc[1][1] / 512;
    calibration[2][0] = calibration_acc[2][0] / 512;
    calibration[2][1] = calibration_acc[2][1] / 512;
    
	while (1)
	{
#pragma xta endpoint "adc_7265_main_loop"
#pragma ordered
		select
		{
		case inct_byref(c_trig, ct):
			if (ct == ADC_TRIG_TOKEN)
			{
				t :> ts;
				t when timerafter(ts + 700) :> ts;
				adc_get_data_7265( adc_val[0], 0, CNVST, DATA_A, DATA_B, MUX );
				adc_get_data_7265( adc_val[1], 2, CNVST, DATA_A, DATA_B, MUX );
				adc_get_data_7265( adc_val[2], 4, CNVST, DATA_A, DATA_B, MUX );
			}
			break;
		case c_adc :> cmd:
			master {
				unsigned a = adc_val[0][0] - calibration[0][0];
				unsigned b = adc_val[0][1] - calibration[0][1];
				unsigned c = adc_val[2][1] - calibration[2][1];
				unsigned d = adc_val[1][0] - calibration[1][0];
				c_adc <: a;
				c_adc <: b;
				c_adc <: c;
				c_adc <: d;
			}
		    break;
		}
	}
}

