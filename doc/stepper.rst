Stepper motor
+++++++++++++

A stepper motor has a pair of coils. The controller can move the spindle a very precise angle by puttint current
in each coil alternately.

The XMOS implementation supports micro-stepping, where the current in each coil is controlled propertionally, to
introduce angular steps of a fraction of the coil angle.

Physical connections
--------------------

A stepper motor has two independent coils.  Consequently, it requires 4 half-bridges.  The software is configured
to have the first coil attached to motor 1 port A and motor 2 port A, and the second coil on motor 1 port B and motor 2 port B.

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

Although the *singleStep* function moves the rotor by one whole step, it performs a series of microsteps between
the two end points.  The number of microsteps are determined by a compile time constant *STEP_SIZE*.  This is the
number of positions through the *COS_SIZE* cosine table which each microstep will take.  By default, the cosine
table has 256 entries, so a *STEP_SIZE* of 256 will give no microstepping, 128 will give one intermediate microstep, and
16 will give 16 microsteps.

During the *singleStep* function, a timer provides control for the stepping operation, which generates reference coil
currents for the specific microstep.  A second timer, running faster, uses the ADC to measure the actual coil currents
and uses a PI controller to adjust the PWMs to produce the correct current.  This fast inner timer is a torque controller.

Running outside of the singleStep* function is the main *motor* control loop.  If it is running in speed control mode,
an outer timer is used to call the singleStep at regular intervals to rotate the rotor at the correct speed.  This loop
also contains channel communication endpoints for receiving control commands for an external control thread.

 


