# Driver Communication Documentation
---------

Overview: 
- Imports
- Constant Definition

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

The constants are explained in the code itself. 

```
max_tries = 20  ### maximum tries of driver for reading encoder position
Gear_ratio = 50 ####################### important------check before run
Rated_torque = 2.68  ### Motor Rated Torque
encoder_resolution = 10000  #unit: inc
L1= 0.305  ### Upper Arm (actuated link) length
L2= 0.595  ### Fore Arm (parallelogram link) length
r = 0.10   ### End-Effector radious
R = 0.13   ### base radious
r1 = L1/2 ### geometric center of actuated link
r2 = L2/2 ### geometric center of parallelogram link
```

The following code can be inserted in a loop that might cause a problem, so there will be an emergency stop for the robot by pressing "q" on the keyboard.

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
Actuator1_serial = serial.Serial(port='COM3',baudrate=38400,parity='N',stopbits=1, bytesize=8, timeout=None, xonxoff=False, rtscts=False,  writeTimeout=None, dsrdtr=False, interCharTimeout=None)
Actuator2_serial = serial.Serial(port='COM4',baudrate=38400,parity='N',stopbits=1, bytesize=8, timeout=None, xonxoff=False, rtscts=False,  writeTimeout=None, dsrdtr=False, interCharTimeout=None)
Actuator3_serial = serial.Serial(port='COM5',baudrate=38400,parity='N',stopbits=1, bytesize=8, timeout=None, xonxoff=False, rtscts=False,  writeTimeout=None, dsrdtr=False, interCharTimeout=None)
```

For more information on each parameter refer to: [pySerial](https://pyserial.readthedocs.io/en/latest/pyserial_api.html)


## Driver Functions 
---------



