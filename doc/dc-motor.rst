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

The frequency of measurement and update of the PI controller is defined by the constant *PERIOD*, which is initially set to 10Hz, to avoid quantization problems in speed measurement.


PWM configuration
~~~~~~~~~~~~~~~~~

The application uses the single port PWM controller from the *sc_pwm* module. This is capable of controlling multiple single bit PWM control lines.  In the case of the DC motor, four PWM lines are required, two for each motor.

The constant *RESOLUTION* defines the number of PWM periods in each cycle, and the constant *TIMESTEP* defines the duration of each period, in 20ns multiples.  Initially, *TIMESTEP* is set to 2, meaning that the duration of each PWM period is 40ns, and the *RESOLUTION* is set to 256, giving 256 PWM steps, for a total PWM cycle time of 5120ns.


Extending the demonstration for more motors
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

An XMOS core is capable of controlling many more than two motors.  The *sc_pwm* repository contains multiple versions of the PWM controller, allowing a variety of configurations to be used.

The *pwm_multibit_port* module allows the PWM to control multiple channels using a 4, 8 or 16 bit port.  Since each brushed DC motor uses 2 pins, an 8 bit port would allow the control of 4 motors.




