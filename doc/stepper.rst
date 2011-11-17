Stepper motor
+++++++++++++

A stepper motor has a pair of coils. The controller can move the spindle a very precise angle by puttint current
in each coil alternately.

The XMOS implementation supports micro-stepping, where the current in each coil is controlled propertionally, to
introduce angular steps of a fraction of the coil angle.

Physical connections
--------------------

A stepper motor has two independent coils.  Consequently, it requires 4 half-bridges.  The software is configured
to have the first coil attached to motor 1 ports A and B, and the second coil on motor 1 port C and motor 2 port A.

The stepper motor used during development of the algorithm is from Astrosyn, and is a 12V unit.

Software components
-------------------

The following threads are running on the motor platform.

+------+-----------------------------------+
| Core | Thread                            | 
+------+-----------------------------------+
|  0   | do_wd (watchdog)                  |
+------+-----------------------------------+
|  0   | controller (simple control)       |
+------+-----------------------------------+
|  1   | motors (motor control)            |
+------+-----------------------------------+
|  1   | adc_7265_triggered (adc)          |
+------+-----------------------------------+
|  1   | pwmSinglePortTrigger              |
+------+-----------------------------------+

The system requires the ADC component from the general motor control repository *sw_motor_control*.

Main control loop
-----------------

The main control loop has a period of 20 Hz.  There are two states which it can be in, speed control or step
counting.  The *doSpeed* and *doSteps* variables control which mode is current active.

The function *singleStep* moves the spindle by a single position.  It has arguments that are channel links to
the PWM and ADC modules, a microstepping period, and a step position.




