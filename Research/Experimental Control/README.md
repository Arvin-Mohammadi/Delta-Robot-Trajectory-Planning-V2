# Control Manual
---------

The steps to use the robot are: 
1. Homing
2. Enable Drivers
2.5. Read Data
3. Move

## 1. Homing
---------
place the robot at home with the bars installed (as illustrated in the following images) 

![Untitled-2](https://github.com/ArthasMenethil-A/Delta-Robot-Trajectory-Planning/assets/69509720/06fe1085-7e7c-45a7-a0ab-360f40d02973)

## 2. Enable Drivers 
---------

Use the function `Enable_all_drivers(-3)` to enable all of the drivers in speed mode. This function also sets the offset which is for each motor as explained below: 

1. Motor1: 25 degree
2. Motor2: 27 degree
3. Motor3: 22 degree

**NOTE: Remove the bars after enabling. Do Not Move The Robot**


## 2.5. Read Data
---------

use each of the following lines of code to read the actual angles being read from the actuators, the calculated angles with offset applied to them, and FK, and IK of the robot: 

```
Position_actual(1)

Position_actual(2)

Position_actual(3)

Position_absolute_read(1)

Position_absolute_read(2)

Position_absolute_read(3)

Forward(Position_absolute_read(1), Position_absolute_read(2), Position_absolute_read(3))

Inverse(Forward(Position_absolute_read(1), Position_absolute_read(2), Position_absolute_read(3))[1], Forward(Position_absolute_read(1), Position_absolute_read(2), Position_absolute_read(3))[2], Forward(Position_absolute_read(1), Position_absolute_read(2), Position_absolute_read(3))[3])
```

## 3. Move
---------

After homing, enabling the drivers and removing the bars, it's ok to move the robot using the function `Goto_xyz(final_xyz, duration)`. In the Home position the coordinates should be [0, 0, -37], you can read this using the following code: 

```
Forward(Position_absolute_read(1), Position_absolute_read(2), Position_absolute_read(3))
```

You can move the robot to a position where each of the upper arms are horizontal using the following line: 

```
Goto_xyz([0, 0, -48], 8)
```

# Driver Communication Documentation
---------

The Driver model is: Kinco AC Servo FD-422

Overview:  
- Imports
- Constant Definition
- Serial Communication Configuration
- Driver Functions  

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

`Close_serial(value)`

Closes serial communcation. 

</br>

`Chks_calculation(value)`

Calculates a checksum value from a list of integers 

</br>

`Conv_to_hex(value)` 

Converts an integer value to a list of four bytes in hexadecimal format.

</br>

`Write_register(ID, CMD_data_volume, left_index, right_index, subindex, object_value)` 

Sends a command to write data to a register in the actuator.

</br>

`Drive_enable(ID)` 

Enables the driver for a specific actuator.

</br>

`Drive_disable(ID)` 

Disables the driver for a specific actuator. 

</br>

`Operation_mode(ID, Mode)` 

Sets the operation mode for an actuator. 

| Mode | Speed | Torque |
|------|-------|--------|
|Number| -3    | 4      |

</br>

`Current_mode(ID)` 

Retrieves current mode data from an actuator.

| Mode | Speed | Torque |
|------|-------|--------|
|Number| -3    | 4      |

</br>

`Target_speed_rpm(ID, Speed_rpm)` 

Sets the target speed for an actuator in RPM.

</br>

`Torque_speed_limit(ID, max_current, max_speed)` 

Sets torque and speed limits for an actuator.

</br>

`Target_torque(ID, Target_Torque_Nm)` 

Sets the target torque for an actuator in Nm.

</br>

`Enable_all_drivers(mode)` 

1. Asks if the Delta Robot is in home place (must input "Y" to continue) 
2. Enables all drivers and sets their operation mode.
3. Sets the offset
4. Asks if it is safe for the Delta Robot to move (must input "Y" to be able to move) 

| Mode | Speed | Torque |
|------|-------|--------|
|Number| -3    | 4      |

</br>

`Disable_all_drivers()` 

Disables all drivers.

</br>

`Emergency_stop()` 

Sets every motor speed to zero thus stopping all of the actuators. This function feeds the code snippet mentioned above for the interrupt. 

</br>

`Position_convert(value)` 

Converts position data from bytes to degrees.

</br>

`Velocity_convert(value)` 

Converts velocity data from bytes to RPM.

</br>

`Torque_convert(value)` 

Converts torque data from bytes to Nm.

</br>

`Position_actual(ID)`

Reads the angles from an actuator. 

**NOTE: the recieved angles are relative to the first time the robot's actuators are enabled everytime the robot is turned on.**

**Meaning that when you turned on the robot and enable all drivers, at that point the angles are zero, and every angle will be determined in relation to that initial angle**

</br>

`Velocity_actual_rpm(ID)` 

reads the velocity from an actuator.

</br>

`Torque_actual(ID)` 

reads the torque from an actuator.

</br>

`Read_all_positions()`

reads and prints all angles from actuators.

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

`_is_point_inside_triangle(P)` 

A helper function to determine if a point is in safe workspace (X, Y direction) or not.

</br>

`Goto_xyz(final_xyz, duration)`

Goes to the given point.

</br>

`Forward`

Calculates the forward kinematics 

</br>

`Inverse`

Calculated the inverse kinematics. (uses the helper function `_angle_yz`)

</br>



## PID control
---------

`Class: PID`

This class implements a PID controller. It is used to control systems with feedback and adjust a control input to maintain a desired setpoint. The class contains the following methods:

`__init__(self, Kp, Ki, Kd, setPoint=0, SampleTime=60)`

The constructor method initializes the PID controller with the given constants (Kp, Ki, Kd) and optional parameters. setPoint is the desired value that the system aims to achieve, and SampleTime is the time interval at which the controller computes its output.

`Compute(self, feedback)`

This method computes the control output based on the feedback value from the system. It calculates the proportional, integral, and derivative terms and combines them to produce the PID control output.

`Compute_torque_pid(self, feedback)`

Similar to the Compute method, but tailored for a specific torque-based PID control scenario.

`SetSampleTime(self, newSampleTime)`

Updates the sample time of the PID controller and adjusts the Ki and Kd constants accordingly.

`SetOutputLimits(self, minOut, maxOut)`

Sets the output limits of the PID controller to prevent excessive control outputs.

`DefineSetpoint(self, coord)`

Sets the desired setpoint for the PID controller.

`set_PID_constants(self, Kp, Ki, Kd)`

Allows you to change the PID constants after the controller has been initialized.

---------

`implement_PID(set_point_coord, feedback)`

This function takes a setpoint coordinate and feedback values as input and calculates the control outputs for three PID controllers using the Inverse Kinematics and the provided PID instances (pid1, pid2, pid3).

`implement_PID_torque(set_point_coord, feedback)`

Similar to implement_PID, but tailored for torque-based PID control.


## Motion Control
---------

`circle_motion(center,radi,height,n,period)`

The arguments are:
   - `center`: Coordinates of the center of the circular path.
   - `radi`: Radius of the circular path.
   - `height`: Height at which the circular motion is performed.
   - `n`: Number of times the circular motion should be performed.
   - `period`: Time period for one complete circular motion.

The motion is controlled using the PID controller. 

`spiral_motion(center,radi,starting_height,ending_height,n,period)`

The arguments are:
   - `center`: Coordinates of the center of the spiral.
   - `radi`: Radius of the spiral.
   - `starting_height`: Initial height for the spiral motion.
   - `ending_height`: Final height for the spiral motion.
   - `n`: Number of iterations of the spiral motion.
   - `period`: Time period for one complete iteration of the spiral motion.

The motion is controlled using the PID controller. 

