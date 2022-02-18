# LBYCPA3 Battery Voltage and Percentage
The project intends to have a Lithium-Ion battery as its input toward the ADC. For the project to work,
a simple connection toward PORTC1, a shunt-resistor, and a Lithium-Ion battery connected to ground is required.

Note that the maximum voltage for the Lithium-Ion battery in this repository assumes 3.7 Nominal Voltage.
Because the voltage of the tested battery (PKCELL ISR18650) goes above that, the 100% battery is assumed at 3.8.

The microcontroller calculuates the battery percentage through a Quadratic Lagrange Interpolation on-the-fly. Please refer to line 125
in the function `estimateBatteryPercentage()` to view the calculation. 