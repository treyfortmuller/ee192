# Checkoff 5

Benchtop velocity control of the motor with a simple proportional feedback controller. Uses the ADC to read optical encoder values to detect motor speed.

### Checkoff Descriptiom

Like checkpoint 3, power (for all systems) can come from either the benchtop supply or the battery. However, all signals (especially the motor driver PWM) must be generated from the microcontroller running your code. Your hardware should not be damaged during any of these tests.
* Demonstrate that you have velocity sensing working and the output in terms of some physical units (m/s, mm/s, etc). You may use the serial console to print speed data. Under constant speed (say, if you apply a constant PWM to the motor), this output should also be relatively constant.

* Your velocity sensing must work even if the motor is stalled (i.e. no encoder pulses). I should see a very low or zero velocity in that case.

* Show you have made an attempt at velocity control. The recommended target setpoint is 3 m/s, which should provide enough encoder counts for a somewhat stable control loop. It's fine if the applied PWM is noticeably jittering or if the actual velocity is inaccurate. However, if I load / stall the motor (with, say, a finger), the controller should compensate by applying a higher drive strength.