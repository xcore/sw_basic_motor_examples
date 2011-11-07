Brushed DC Motor
++++++++++++++++

The directory *app_DC_motor* contains an application for speed control of two DC motors.

Physical connections
--------------------

The code uses a simple unipolar PWM to control the high sides of an H-bridge, the low sides being merely switched depending on the direction of rotation. Half-bridges A and B are used.

Speed detection uses the hall effect inputs, which are attached to magnetic quadrature encoders built into the motors.  The quadrature encoders have two channels and no index channel, so only speed is detected.  Hall signals A and B are connected to the two channels.

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
|  0   | display_gpio (user IO)            |
+------+-----------------------------------+
|  1   | motors (motor control)            |
+------+-----------------------------------+
|  1   | pwmSinglePort                     |
+------+-----------------------------------+

The system requires the single port PWM component from the *sc_pwm* open source repository.



Main control loop
-----------------

The main control loops is run in the *motors* thread function.

Speed detection
~~~~~~~~~~~~~~~

The Hall sensor port is used to monitor the motor speed.  The port is monitored using a port event, and as it passes one specific point in the 4 value cycle, the rotation count is increased.

The brushed DC motors that were used during development have a 52:1 gear unit on top of the spindle, and have 12 rotations of the encoder for a single revolution of the motor spindle.

Constants *GEAR_RATIO* and *ENCODER_COUNTS_PER_REV* set these values within the code.

External control
~~~~~~~~~~~~~~~~

There are two control paths for changing the speed of the motors.  The *display_gpio* thread function, which provides input and output via the LCD screen and buttons, has one channel in and out of the control loop.

The *controller* thread function has another channel input to the control loop for adjusting the motor speeds. This thread represents a more sophisticated control algorithm, although the example merely sets the initial speed.


PI control
~~~~~~~~~~

The core of the *motors* function periodically measures the speed of the motor, and uses a PI controller to alter the PWM duty cycle to keep the motor speed constant.





