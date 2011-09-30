/*
 * adc_7265.xc
 *
 *  Created on: Jul 6, 2011
 *  Author: A SRIKANTH
 */

#include <xs1.h>
#include <platform.h>
#include <xclib.h>
#include <adc_common.h>
#include <adc_7265.h>

#ifdef USE_XSCOPE
#include <xscope.h>
#endif


#define ADC_FILTER_7265

// This parameter needs to be tuned to move the ADC trigger point into the centre of the 'OFF' period.
// The 'test_pwm' application can be run in the simulator to tune the parameter.  Use the following
// command line:
//    xsim --vcd-tracing "-core stdcore[1] -ports" bin\Release\test_pwm.xe > trace.vcd
//
// Then open the 'Signals' and 'Waves' panes in the XDE, load the VCD file and look at the traces
// named 'PORT_M1_LO_A', 'PORT_M1_LO_B', 'PORT_M1_LO_C', and 'PORT_ADC_CONV'.  The ADC conversion
// trigger should go high in the centre of the low periods of all of the motor control ports. This
// occurs periodically, but an example can be found at around 94.8us into the simulaton.
#define ADC_TRIGGER_DELAY 700


#pragma xta command "analyze loop adc_7265_main_loop"
#pragma xta command "set required - 40 us"

// These are the calibration values
static unsigned calibration[6];

// Mode to say if we are currently calibrating the ADC
static int calibration_mode;

// Accumultor for the calibration average
static int calibration_acc[6];


static void configure_adc_ports_7265(clock clk, out port SCLK, port CNVST, in buffered port:32 DATA_A, in buffered port:32 DATA_B, out port MUX)
{
	// configure the clock to be 16MHz
    //configure_clock_rate_at_least(clk, 16, 1);
    configure_clock_rate_at_most(clk, 16, 1);
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

	val1 = val1 >> 4;
	val3 = val3 >> 4;

	val1 = 0x00000FFF & val1;
	val3 = 0x00000FFF & val3;

#ifdef USE_XSCOPE
	xscope_probe_data(0, val1);
	xscope_probe_data(1, val3);
#endif

#ifdef ADC_FILTER_7265
	adc_val[0] = (adc_val[0] >> 1) + (val1 >> 1);
	adc_val[1] = (adc_val[1] >> 1) + (val3 >> 1);
#else
	adc_val[0] = val1;
	adc_val[1] = val3;
#endif

}


//When MUX = 0, M1A and M1B
//When MUX = 2, M2A and M2B
//When MUX = 4, M2C and M1C

#pragma unsafe arrays
void adc_7265_6val_triggered( chanend c_adc, chanend c_trig, clock clk, out port SCLK, port CNVST, in buffered port:32 DATA_A, in buffered port:32 DATA_B, port out MUX )
{
	int adc_val0[2] = {0,0};
	int adc_val2[2] = {0,0};
	int adc_val4[2] = {0,0};
	
	int cmd;
	unsigned char ct;

	timer t;
	unsigned ts;

	set_thread_fast_mode_on();

	configure_adc_ports_7265( clk, SCLK, CNVST, DATA_A, DATA_B, MUX );

	
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
				t when timerafter(ts + ADC_TRIGGER_DELAY) :> ts;
				adc_get_data_7265( adc_val0, 0, CNVST, DATA_A, DATA_B, MUX );
				adc_get_data_7265( adc_val2, 2, CNVST, DATA_A, DATA_B, MUX );
				adc_get_data_7265( adc_val4, 4, CNVST, DATA_A, DATA_B, MUX );
				if (calibration_mode > 0) {
					calibration_mode--;
					calibration_acc[0] += adc_val0[0];
					calibration_acc[1] += adc_val0[1];
					calibration_acc[2] += adc_val2[0];
					calibration_acc[3] += adc_val2[1];
					calibration_acc[4] += adc_val4[0];
					calibration_acc[5] += adc_val4[1];																						
					if (calibration_mode == 0) {
						calibration[0] = calibration_acc[0] >> 9;
						calibration[1] = calibration_acc[1] >> 9;
						calibration[2] = calibration_acc[2] >> 9;
						calibration[3] = calibration_acc[3] >> 9;
						calibration[4] = calibration_acc[4] >> 9;
						calibration[5] = calibration_acc[5] >> 9;
					}
				}
			}
			break;
		case c_adc :> cmd:
			if (cmd == 1) {
				calibration_mode = 512;
				calibration_acc[0]=0;
				calibration_acc[1]=0;
				calibration_acc[2]=0;
				calibration_acc[3]=0;	
				calibration_acc[4]=0;
				calibration_acc[5]=0;								
			} else {
				master {
					unsigned a = adc_val0[0] - calibration[0];
					unsigned b = adc_val0[1] - calibration[1];
					unsigned c = adc_val2[0] - calibration[2];
					unsigned d = adc_val2[1] - calibration[3];
					unsigned e = adc_val4[0] - calibration[4];
					unsigned f = adc_val4[1] - calibration[5];
					c_adc <: a;
					c_adc <: b;
					c_adc <: c;
					c_adc <: d;
					c_adc <: e;
					c_adc <: f;
				}
			}
			break;
		}
	}
}

