// Copyright (c) 2011, XMOS Ltd., All rights reserved
// This software is freely distributable under a derivative of the
// University of Illinois/NCSA Open Source License posted in
// LICENSE.txt and at <http://github.xcore.com/>

#include "xs1.h"
#include "config.h"
#include "lcd.h"
#include "shared_io.h"
#include "stdio.h"


/* Manages the display, buttons and shared ports. */
void display_gpio( chanend c_lcd1, chanend c_lcd2, REFERENCE_PARAM(lcd_interface_t, p), port in btns)
{
	unsigned int time, speed1 = 0, speed2 = 0, set_speed = INITIAL_SET_SPEED;

	unsigned int value = 0;
	unsigned int btn_en = 0;
	unsigned toggle = 1;
	char my_string[50];
	unsigned ts,temp=0;
	timer timer_1,timer_2;

	/* Initiate the LCD ports */
	lcd_ports_init(p);

	/* Get the initial time value */
	timer_1 :> time;

	/* Loop forever processing commands */
	while (1)
	{
		select
		{
		    /* Timer event at 10Hz */
			case timer_1 when timerafter(time + 10000000) :> time:
		        /* Get the motor 1 speed and motor 2 speed */
				c_lcd1 <: CMD_GET_IQ;
				c_lcd1 :> speed1;
				c_lcd1 :> set_speed;
#if NUM_MOTORS > 1
				c_lcd2 <: CMD_GET_IQ;
				c_lcd2 :> speed2;
				c_lcd2 :> set_speed;
#endif

		        /* Calculate the strings here */
		        /* Now update the display */
				lcd_draw_text_row( "  XMOS DC Motor Demo\n", 0, p );
				sprintf(my_string, "  Set Speed: %04d\n", set_speed );
				lcd_draw_text_row( my_string, 1, p );

				sprintf(my_string, "  Speed1 : 	 %04d RPM\n", speed1 );
				lcd_draw_text_row( my_string, 2, p );

				sprintf(my_string, "  Speed2 : 	 %04d RPM\n", speed2 );
				lcd_draw_text_row( my_string, 3, p );

		        /* Switch debouncing */
				if ( btn_en != 0) btn_en--;

				break;



			case !btn_en => btns when pinsneq(value) :> value:
				value = (~value & 0x0000000F);

				if (value == 0) {
					// Nothing
				}
				else if(value == 1)
				{
			        /* Increase the speed, by the increment */
					set_speed += PWM_INC_DEC_VAL;
					if (set_speed > MAX_RPM)
						set_speed = MAX_RPM;
		            /* Update the speed control loop */
					c_lcd1 <: CMD_SET_SPEED;
					c_lcd1 <: set_speed;
#if NUM_MOTORS > 1
					c_lcd2 <: CMD_SET_SPEED;
					c_lcd2 <: set_speed;
#endif

			        /* Increment the debouncer */
					btn_en = 4;
				}
				else if(value == 2)
				{
					set_speed -= PWM_INC_DEC_VAL;
			        /* Limit the speed to the minimum value */
					if (set_speed < MIN_RPM)
					{
						set_speed = MIN_RPM;
					}
			        /* Update the speed control loop */
					c_lcd1 <: CMD_SET_SPEED;
					c_lcd1 <: set_speed;
#if NUM_MOTORS > 1
					c_lcd2 <: CMD_SET_SPEED;
					c_lcd2 <: set_speed;
#endif

			        /* Increment the debouncer */
					btn_en = 4;
				}
				else if(value == 4)
				{
					toggle = !toggle;
					temp = set_speed;
			        /* to avoid jerks during the direction change*/
					while(set_speed > MIN_RPM)
					{
						set_speed -= STEP_SPEED;
			            /* Update the speed control loop */
						c_lcd1 <: CMD_SET_SPEED;
						c_lcd1 <: set_speed;
#if NUM_MOTORS > 1
						c_lcd2 <: CMD_SET_SPEED;
						c_lcd2 <: set_speed;
#endif
						timer_2 :> ts;
						timer_2 when timerafter(ts + _30_Msec) :> ts;
					}
					set_speed = 0;
			        /* Update the speed control loop */
					c_lcd1 <: CMD_SET_SPEED;
					c_lcd1 <: set_speed;
					c_lcd2 <: CMD_SET_SPEED;
					c_lcd2 <: set_speed;
			        /* Update the direction change */
					c_lcd1 <: CMD_DIR;
					c_lcd1 <: toggle;
					c_lcd2 <: CMD_DIR;
					c_lcd2 <: toggle;
			        /* to avoid jerks during the direction change*/
					while(set_speed < temp)
					{
						set_speed += STEP_SPEED;
						c_lcd1 <: CMD_SET_SPEED;
						c_lcd1 <: set_speed;
#if NUM_MOTORS > 1
						c_lcd2 <: CMD_SET_SPEED;
						c_lcd2 <: set_speed;
#endif
						timer_2 :> ts;
						timer_2 when timerafter(ts + _30_Msec) :> ts;
					}
					set_speed  = temp;
			        /* Update the speed control loop */
					c_lcd1 <: CMD_SET_SPEED;
					c_lcd1 <: set_speed;
#if NUM_MOTORS > 1
					c_lcd2 <: CMD_SET_SPEED;
					c_lcd2 <: set_speed;
#endif

			        /* Increment the debouncer */
					btn_en = 4;
				}
				break;

		}
	}
}


