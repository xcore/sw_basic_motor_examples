Brushed DC and Stepper motor examples
=====================================

Introduction
++++++++++++

Motors can be divided into different types.  XMOS provides a solution for electrically commutating Brushless DC Motors and Permanent Magnet Synchronous Motors, in the sw_motor_control open source repository.

The sw_basic_motor_examples provides two alternative uses for the XP_MC_LVM2 power and control boards.  The first provides speed control of two brushed DC motors.  The second provides position control of a two coil stepper motor.

Supported hardware
++++++++++++++++++

The XMOS XP_MC_LVM2 kit contains a control board and a power board.  These can be used to control both the DC and stepper motor examples.

Brushed DC motors come in many standard voltages. Stepper motors are more commonly 12V.  The motors which were used to build these demonstration applications are both 12V.

The power board in the XP_MC_LVM2 kit is designed for 24V, but will work with a 12V power supply with a single modification.  The undervoltage protection circuit needs modification.  Resistor R100, an XXX Kohm must be replaced with a XXX Kohm one.

.. toctree::

   dc-motor
   stepper



