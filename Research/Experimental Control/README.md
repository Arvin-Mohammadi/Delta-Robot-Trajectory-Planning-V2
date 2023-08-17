# Driver Communication Documentation
---------

The Driver model is: Kinco AC Servo FD-422

Overview:  
- Imports
- Constant Definition
- Serial Communication Configuration  

## Imports
---------

The dependencies are listed as the following. 
```
import numpy as np
import math
import timeit
import math
import time
import serial  ## for serial communication: pip install pyserial
import numpy as np
import struct
import scipy.io as sio ## for reading and writing MATLAB mat files
from numpy import linalg as LA
import matplotlib.pyplot as plt
from scipy.optimize import fsolve
import datetime
import keyboard
```

## Constants 
---------

The constants are explained in the code itself. The following code can be inserted in a loop that might cause a problem, so there will be an emergency stop for the robot by pressing "q" on the keyboard.

```
if keyboard.is_pressed('q'):
    Emergency_stop()
    Disable_all_drivers()
    print("Loop terminated by user.")
    break
```

## Serial Communication Configuration  
---------

The following snippet of code activates the serial ports to communicate with them:

```
Actuator_serial = serial.Serial(port='COM3',baudrate=38400,parity='N',stopbits=1, bytesize=8, timeout=None, xonxoff=False, rtscts=False,  writeTimeout=None, dsrdtr=False, interCharTimeout=None)
```

For more information on each parameter refer to: [pySerial](https://pyserial.readthedocs.io/en/latest/pyserial_api.html)


## Driver Functions 
---------

`chks_calculation(value)`

Calculates a checksum value from a list of integers 

</br>

`conv_to_hex(value)` 

Converts an integer value to a list of four bytes in hexadecimal format.

</br>

`Write_register(ID, CMD_data_volume, left_index, right_index, subindex, object_value)` 

Sends a command to write data to a register in the actuator.

</br>

`Drive_enable(ID)` 

enables the driver for a specific actuator.

</br>

`Drive_disable(ID)` 

disables the driver for a specific actuator.

</br>

`Operation_mode(ID, Mode)` 

sets the operation mode for an actuator.

</br>

`Current_mode(ID)` 

retrieves current mode data from an actuator.

</br>

`Target_speed_rpm(ID, Speed_rpm)` 

sets the target speed for an actuator in RPM.

</br>

`Torque_speed_limit(ID, max_current, max_speed)` 

sets torque and speed limits for an actuator.

</br>

`Target_torque(ID, Target_Torque_Nm)` 

sets the target torque for an actuator in Nm.

</br>

`Enable_all_drivers(mode)` 

enables all drivers and sets their operation mode.

</br>

`Disable_all_drivers()` 

disables all drivers.

</br>

`Emergency_stop()` 

stops all actuators.

</br>

`Position_convert(value)` 

converts position data from bytes to degrees.

</br>

`Velocity_convert(value)` 

converts velocity data from bytes to RPM.

</br>

`Torque_convert(value)` 

converts torque data from bytes to Nm.

</br>

`Position_actual(ID)` 

reads the actual position from an actuator.

</br>

`Velocity_actual_rpm(ID)` 

reads the actual velocity from an actuator.

</br>

`Torque_actual(ID)` 

reads the actual torque from an actuator.

</br>

`Read_all_positions()`

reads and prints all position data from actuators.

</br>

`Motion_z_endeffector(speed)` 

moves the end effector in the Z direction.

</br>

`Motion_y_endeffector(speed)` 

moves the end effector in the Y direction.

</br>

`Motion_x_endeffector(speed)` 

moves the end effector in the X direction.

</br>

`Homing()` 

performs homing for the actuators.

</br>


