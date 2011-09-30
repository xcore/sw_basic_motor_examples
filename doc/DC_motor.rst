DC Motor Example Application
============================

The repository contains a folder called ''app_DC_motor''. This contains all the code to show functionality of two DC motor being run on the Motor Control Platform.

Components
+++++++++++++++++++++++++++++++++++++++++++++++++++++++

This application makes use of the following components:

   * PWM
   * Display 
   * Watchdog timer

Motor Control Loop
~~~~~~~~~~~~~~~~~~

The main motor control code for this application can be located in ``src/DC.xc``. The motor control thread is launched using the following function.

::

  motors( c_wd, c_speed, c, c_control);

The core functionality of this is to turn the function is to set the motors to particular speeds and hold at that speed through use of a PI controller.

After initially pausing and starting the watchdog the main loop is entered. The main loop responds to a number of events as follows:

  * When the main timer is triggered (at a period defined in PERIOD)
      - The PI controller is run against the current speed and the output is used to set the duty cycle for the PWM. If either motor is ramping then the speed is increased accordingly. Direction changes are also dealt with here.
  * When a command is received from the control thread over ''c_control''
      + The following commands may be sent:
          - CMD_RAMP  -  This is followed by a designation of which motor to use and a struct, ramp_parameters, as defined in ''DC.h''.
          - CMD_SET_MOTOR_SPEED  --  Sets the motor to run at a constant speed, parameters are which motor to use, the desired speed and the direction.
          - CMD_GET_MOTOR_SPEED  --  This command is followed by the motor number. The current speed of the motor is then sent over the channel.
  * The encoder input has changed
      - This increments or decrements the current number of rotations which is later used to calculate the speed of the motor. This is defined for both motors.
  * One second has passed
      - The RPM of the motor is calculated according to the gearing ratios and the current speed of the motor.
  * A command was received from the display
      - CMD_GET_IQ
          + Send the current speed and the desired speed to the display
      - CMD_SET_SPEED
          + Receive the required speed from the display
      - CMD_DIR
          + Reverse the direction of the motor.
