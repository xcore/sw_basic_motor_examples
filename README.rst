Basic Motor Control Examples
..........................

:Stable release:  Unreleased

:Status:  Work in Progress

:Maintainer:  bitdivision

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

* Breakout the singleStep function to provide individual microstep positioning

Known Issues
============

* Currently does not work on the XP-MC-CTRL-L2 board

Required Repositories
================

* xcommon git\@github.com:xcore/xcommon.git

Required Hardware
=================

* XP-MC-CTRL-L2 development kit
* Bipolar stepper motor
* DC motor with quadrature encoder

Support
=======

Problems can be reported using the issues tab in github.
