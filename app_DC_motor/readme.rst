DC Motor Example
================

Description:

This directory contains the source for the DC motor example on the XP-DSC-BLDC board. 

Hardware: 

* XP-DSC-BLDC development kit

* A DC motor should be connected to either of the ports on the board and wired up as follows:
    * The ENC A and ENC B ports connected to encoder lines A and B 
    * +5V and GND connected to the encoder
    * Phase A and B connected to +ve and -ve motor lines


Interface:

* Buttons A and B can be used to increase and decrease the speed of the motor(s) respectively
* Button C is used to reverse the direction of the motor


Todo:

* Multiple motor control is sketchy. Code needs to be reworked to allow seperate control of motors and extensibility for more than 2 motors
* Position control
