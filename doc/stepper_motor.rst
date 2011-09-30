Stepper Motor Example Application
=================================

The repository contains a folder called ''app_stepper_motor''. This contains all the code to show functionality of a stepper motor being run on the Motor Control Platform.

Components
+++++++++++++++++++++++++++++++++++++++++++++++++++++++

This application makes use of the following components:

   * PWM
   * ADC 
   * Watchdog timer

Motor Control Loop
~~~~~~~~~~~~~~~~~~

The main motor control code for this application can be located in ``src/stepper.xc``. The motor control thread is launched using the following function.

::

  motor(c_pwm, c_control, c_wd, c_adc);

The core functionality of this is to turn the motor by a single step at a particular frequency and / or for a particular number of steps.

After initially pausing and starting the watchdog the main loop is entered. The main loop responds to two events. The first event is a command received on the channel ''c_control''. The commands available are defined below.

  *CMD_SET_MOTOR_SPEED*  -  This command is followed by two parameters: The step period and direction (FORWARD or REVERSE)
  
  *CMD_NUMBER_STEPS*  -  This command is followed by three parameters: The step period, the number of steps to move and the direction (FORWARD or REVERSE)
  
  *CMD_MOTOR_OFF*  -  This command turns off current to the motor, it does this by setting the PWM value for low and high ports off


The second event that can be responded to is a timer which triggers every step period. This checks values of variable set by the previous event and calls the singlestep function accordingly.
  
Single Step Function
~~~~~~~~~~~~~~~~~~~~

The single step function turns the stepper motor by a single step using all previously defined configuration options. It is defined as follows:

::

  void singleStep(chanend c_pwm, unsigned int &step, unsigned microPeriod, chanend c_adc)
  
The function selects between two events, a timer which triggers every microstep period and a timer to get ADC values and apply the PI controller to the output. Each microstep period, the function calculates a reference current from the cosine lookup table and also sets the appropriate decay mode according to the configuration options and the current section of the cosine wave. The reference current is also scaled to a value in the same range as the ADC.

The commutation sequence for the high side FETs is shown below

::

     0	0  0  1
     1  0  0  0
     0	0  1  0
     0	1  0  0
     
Where FETs are shown as 1, a duty cycle is calculated according to the number of microsteps and any current limiting that is applied. The duty cycle is then sent to the PWM server and output to the FETs.

If the decay mode is alternating, whenever a state is encountered where the winding current is falling, fast decay is enabled. This works by applying the PWM signal originally used for the high side FETs to the corresponding low side FET, causing all FETs to be turned off when the high side is off.

The PWM is currently set to run at 18KHz with 8 bits of resolution.

Every ADC period the ADC returns the current in the windings and this is fed into the PI controller. Outputs from the PI controller are applied to the PWM.






