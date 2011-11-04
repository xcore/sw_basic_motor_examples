Basic Motor Control Examples
..........................

:Stable release:  Unreleased

:Status:  Work in Progress

:Maintainer:  davidn@xmos.com

:Description:  Examples for basic control of stepper and DC motors


Key Features
============

* DC Motor:
    * Support for multiple motors
    * Basic speed control using PWM library
    * PID control using quadrature encoder for constant speed

* Stepper Motor:
    * Microstepping
    * Current monitoring with ADC
    * Chopping (open loop / closed loop)
    * Decay Modes - Alternating or fixed

To Do
=====

Known Issues
============

* The XMOS motor control board is designed for 24V motors.  To run the 12V brushed DC and
  12V stepper motors, the undervoltage section of the power board needs to be modified to
  prevent it from stopping the FETs.  The resistor R99, found near the centre of the power
  board, should be replaced by a 68K resistor.  This will convert the undervoltage protection
  to 11.7V.  Alternatively, removing resistor R100 will disable the undervoltage completely.

Required Repositories
================

* sc_pwm git\@github.com:xcore/sc_pwm.git
* xcommon git\@github.com:xcore/xcommon.git

Required Hardware
=================

* XP-DSC-BLDC development kit
* Bipolar stepper motor
* DC motor with quadrature encoder

Support
=======

Problems can be reported using the issues tab in github.
