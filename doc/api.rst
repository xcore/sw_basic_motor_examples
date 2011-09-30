.. _sec_api:

API
===

.. _sec_conf_defines:

DC Configuration Defines
---------------------

The file config.h must be provided in the application source
code. The following defines may be set up:

  **RESOLUTION**
  
    Allows the resolution of the PWM thread to be changed. This needs to be greater than or equal to the number of microsteps
  
  **TIMESTEP**
  
    Allows modification of the PWM period. Period (in ns) given by (10 * resolution) [if timestep = 0] or (timestep * 20 * resolution) [if timestep > 0]

  **PERIOD**
  
    Sets the period for the PI loop and PWM update.
  
  **NUMBER_OF_MOTORS**
  
    Sets the number of motors connected to the board.

  **K_P**
  
    The proportional gain of the PI controller.
  
  **K_I** 
  
    The Integral gain of the PI controller.

Stepper Configuration Defines
---------------------

The file config.h must be provided in the application source
code. The following defines may be set up:

  **RESOLUTION**
  
    Allows the resolution of the PWM thread to be changed. This needs to be greater than or equal to the number of microsteps
  
  **TIMESTEP**
  
    Allows modification of the PWM period. Period (in ns) given by (10 * resolution) [if timestep = 0] or (timestep * 20 * resolution) [if timestep > 0]
  
  
  **ADC_PERIOD**
  
    The period at which the ADC is checked and PI controller applied
  
  **PI_ANTIWINDUP**
    
    The value of the anti-windup gain used by the PI controller
  
  **PI_GAIN1**
  
    Gain value for PI controller
  
  **PI_GAIN2**
    
    Gain value for PI controller
  
  **USE_XSCOPE**
  
    If defined this enables xscope to be configured. If xscope is used -lxscope needs to be added to the makefile and the section of the .xn file for xscope must be uncommented
  
  **STEP_SIZE**
  
    This defines the number of microsteps to be used. The number of microsteps is given by the following formula: COS_SIZE (normally 256) / STEP_SIZE. Valid values of STEP_SIZE are {256,128,64,32,16,8,4,2 or 1}
  
  **V_REF**
  
    Gives the reference voltage of the power board. This is used in calculations for open loop chopping and to scale the reference current in line with ADC values. This is usually 24.
  
  **R_WINDING**
  
    The resistance per winding in the stepper motor. Also used in calculations for scaling reference current and open loop chopping.
  
  
  **DECAY_MODE**
  
    Allows the decay mode to be changed. If defined as ''fast'', alternating decay will be used. If defined as ''slow'', fixed decay is used.
  
  
  **IMAX**
  
    If chopping is used, this will be the maximum current allowable in milliamps.
  
  **OPEN_LOOP_CHOPPING**
  
    If defined this limits the current to the value defined in IMAX. This is done by calculating the duty cycle that would theoretically give a certain current and not allowing the duty cycle to rise above that current.
  
  **CLOSED_LOOP_CHOPPING**
  
    If defined this limits the current to the value defined in IMAX. Done by checking the ADC value and limiting the duty cycle when the current goes too high.
  
  
Stepper Motor Control
---------------------

.. doxygenfunction:: controller   

.. doxygenfunction:: getWindingADC  

.. doxygenfunction:: setWindingPWM   

.. doxygenfunction:: applyPI   

.. doxygenfunction:: singleStep 

.. doxygenfunction:: motor

DC Motor Control
----------------

.. doxygenstruct:: ramp_parameters

.. doxygenfunction:: DCcontroller   

.. doxygenfunction:: motors

ADC
---

Client functions
++++++++++++++++

.. doxygenfunction:: do_adc_calibration

.. doxygenfunction:: get_adc_vals_calibrated_int16

Server functions
++++++++++++++++

.. doxygenfunction:: adc_7265_6val_triggered


Watchdog Timer
--------------

.. doxygenfunction:: do_wd


LCD display and PHY reset
-------------------------

LCD
+++

.. doxygenstruct:: lcd_interface_t

.. doxygenfunction:: reverse

.. doxygenfunction:: itoa

.. doxygenfunction:: lcd_ports_init

.. doxygenfunction:: lcd_byte_out

.. doxygenfunction:: lcd_clear

.. doxygenfunction:: lcd_draw_image

.. doxygenfunction:: lcd_draw_text_row


