# Checkoff 3

Functioning motor drive and steering actuation, not simultaneous.

### Checkoff Description

For this checkpoint, you MUST use the benchtop power supply to power your system. Do not use the thin alligator clip connectors on your high current power supply. These have a high resistance (about 1 Ohm) which may cause your circuit to run inefficiently and burn out. The TA example car is left on the lab bench at the back of the room. Feel free to look at it- pay special attention to the lower gauge wire used for power, motor, and Mosfet connections. Also notice the custom xt60 (yellow) power supply adaptor, which uses thick, low gauge wire (you will want to make something like this for yourself). Finally, some of these high current power supplies say “5A limit” on the front panel. If your station has one of these power supplies make sure to connect the power cables to the terminals at the back of the supply box (see TA example for reference).

All control signals must be generated from your microcontroller running your code. You may use the serial terminal to send manual control signals (like setting the motor duty cycle).

This checkpoint is divided into two equal parts which may be checked off separately (for example, if you need to deploy different code for the motor driver and steering actuation).

Your hardware should not be damaged (i.e. components should not catch fire or smoke) during any of these tests.

C3.1: Demonstrate your motor drive circuit works (in any order, instructor's choice):
* Full on (40%)
* Full off (0%)
* 30% duty cycle
* 5s stall test at 30% duty cycle

C3.2: Demonstrate steering control:

* Turn left
* Turn right
* Center steering

You must also show your PWM waveforms on the oscilloscope.