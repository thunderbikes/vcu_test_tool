# VCU Test Tool

## About

This firmware is to be flashed to the VCU 1.0 board. 
You can then connect to the board at 11520 baud UART and enter the below commands.
If you want to use deleting, ensure your terminal uses ```<BS>``` instead of ```<DEL>```.
The justfile has a configuration for this if you have `tio' installed on your machine.

## Implemented Functions

### set <pin_name>

Will set the entered pin HIGH.

### reset <pin_name>

Will set the entered pin LOW.

### read <pin_name>

Will digital read the entered pin.

### help

I didn't write any documentation yet...
