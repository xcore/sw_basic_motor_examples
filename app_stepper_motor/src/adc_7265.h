/*
 * adc_7265.h
 *
 *  Created on: Jul 6, 2011
 *      Author: A SRIKANTH
 */

#ifndef ADC_7265_H_
#define ADC_7265_H_

//#include <adc_common.h>

/** \brief Implements the AD7265 triggered ADC service
 *
 *  This implements the AD hardware interface to the 7265 ADC device.  It reads 6 ADC values in successive pairs
 *
 *  \param c_adc the ADC server control channel
 *  \param c_trig the channel to recieve triggers from the PWM module
 *  \param clk an XCORE clock to provide clocking to the ADC
 *  \param SCLK the external clock pin on the ADC
 *  \param CNVST the convert strobe on the ADC
 *  \param DATA_A the first data port on the ADC
 *  \param DATA_B the second data port on the ADC
 *  \param MUX a port to allow the selection of the analogue MUX input
 *
 */
 
void adc_7265_6val_triggered( chanend c_adc, chanend c_trig, clock clk, out port SCLK, port CNVST, in buffered port:32 DATA_A, in buffered port:32 DATA_B, port out MUX );

#endif /* ADC_7265_H_ */
