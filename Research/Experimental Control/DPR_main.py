

# =================================================================================================
# -- imports --------------------------------------------------------------------------------------
# =================================================================================================

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
import pygame
import keyboard
import sys 

from pygame import K_UP 
from pygame import K_DOWN 
from pygame import K_LEFT 
from pygame import K_RIGHT 
from pygame import K_SPACE
from pygame import K_w 
from pygame import K_q 
from pygame import K_s 


# =================================================================================================
# -- constants ------------------------------------------------------------------------------------
# =================================================================================================


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

global IS_MOVING
IS_MOVING = 0

## INTERRUPT CODE 
# if keyboard.is_pressed('q'):
#     Emergency_stop()
#     Disable_all_drivers()
#     print("Loop terminated by user.")
#     break


# =================================================================================================
# -- serial com config ----------------------------------------------------------------------------
# =================================================================================================


Actuator1_serial = serial.Serial(port='COM3',baudrate=38400,parity='N',stopbits=1, bytesize=8, timeout=None, xonxoff=False, rtscts=False,  writeTimeout=None, dsrdtr=False, interCharTimeout=None)
Actuator2_serial = serial.Serial(port='COM4',baudrate=38400,parity='N',stopbits=1, bytesize=8, timeout=None, xonxoff=False, rtscts=False,  writeTimeout=None, dsrdtr=False, interCharTimeout=None)
Actuator3_serial = serial.Serial(port='COM5',baudrate=38400,parity='N',stopbits=1, bytesize=8, timeout=None, xonxoff=False, rtscts=False,  writeTimeout=None, dsrdtr=False, interCharTimeout=None)

# =================================================================================================
# -- driver functions -----------------------------------------------------------------------------
# =================================================================================================

def Close_serial():
    Actuator1_serial.close()
    Actuator2_serial.close()
    Actuator3_serial.close()
    
    
def Chks_calculation(value) :
    
    value =  -1*sum(value)  
    value = 2**32 + value 
    chks = list(struct.unpack('>4B',struct.pack('>L',value)))      
    return chks[3]

#----------------------------------------------------------------------------------------------------------------------------
def Conv_to_hex(value) :
    
    if value < 0 :
        value = 2**32 + value  
    Hexadecimal = list(struct.unpack('>4B',struct.pack('>L',value)))
    return Hexadecimal

#----------------------------------------------------------------------------------------------------------------------------
def Write_register(ID, CMD_data_volume, left_index, right_index, subindex, object_value):
    value_Hex = Conv_to_hex(object_value)
    Down_master_send = [ID, CMD_data_volume, right_index, left_index, subindex, int(value_Hex[3]), int(value_Hex[2]), int(value_Hex[1]), int(value_Hex[0])]
    Down_master_send.append(Chks_calculation(Down_master_send))
    Down_master_send = bytearray(Down_master_send)

    if ID == 1 : 
        Actuator1_serial.write(Down_master_send)
    elif ID == 2 : 
        Actuator2_serial.write(Down_master_send) 
    elif ID == 3 : 
        Actuator3_serial.write(Down_master_send)
    
#----------------------------------------------------------------------------------------------------------------------------
def Drive_enable(ID):
    
    object_value = 0x2F
    value_Hex = Conv_to_hex(object_value)
    Down_master_send = [ID, 0x2B, 0x40, 0x60, 0x00, int(value_Hex[3]), int(value_Hex[2]), int(value_Hex[1]), int(value_Hex[0])]
    Down_master_send.append(Chks_calculation(Down_master_send))
    Down_master_send = bytearray(Down_master_send)
    
    if ID == 1 : 
        Actuator1_serial.write(Down_master_send)
    elif ID == 2 :
        Actuator2_serial.write(Down_master_send)
    elif ID == 3 :
        Actuator3_serial.write(Down_master_send)


#----------------------------------------------------------------------------------------------------------------------------
def Drive_disable(ID) :
    
    object_value = 0x06
    value_Hex = Conv_to_hex(object_value)
    Down_master_send = [ID, 0x2B, 0x40, 0x60, 0x00, int(value_Hex[3]), int(value_Hex[2]), int(value_Hex[1]), int(value_Hex[0])]
    Down_master_send.append(Chks_calculation(Down_master_send))
    Down_master_send = bytearray(Down_master_send)
    
    if ID == 1 : 
        Actuator1_serial.write(Down_master_send)
    elif ID == 2 :
        Actuator2_serial.write(Down_master_send)
    elif ID == 3 :
        Actuator3_serial.write(Down_master_send)


#----------------------------------------------------------------------------------------------------------------------------
def Operation_mode(ID,Mode):

    value_Hex = Conv_to_hex(Mode)
    Down_master_send = [ID, 0x2F, 0x60, 0x60, 0x00, int(value_Hex[3]), int(value_Hex[2]), int(value_Hex[1]), int(value_Hex[0])]
    Down_master_send.append(Chks_calculation(Down_master_send))
    Down_master_send = bytearray(Down_master_send)
  

    if ID == 1 : 
        Actuator1_serial.write(Down_master_send)
        Actuator1_serial.flushInput()
    if ID == 2 : 
        Actuator2_serial.write(Down_master_send)
        Actuator2_serial.flushInput()
    if ID == 3 : 
        Actuator3_serial.write(Down_master_send)
        Actuator3_serial.flushInput()

#----------------------------------------------------------------------------------------------------------------------------
def Current_mode(ID):
    Actuator1_serial.flushInput()
    
    Up_master_send = [ID, 0x40, 0x60, 0x60, 0x00, 0x0, 0x0, 0x0, 0x0]
    Up_master_send.append(Chks_calculation(Up_master_send))
    Up_master_send = bytearray(Up_master_send)
    
    
    if ID == 1 :
        Actuator1_serial.write(Up_master_send)
        Up_slave_response = list(Actuator1_serial.read(10))
        value = Up_slave_response
        [print('0x{0:0{1}X}'.format(_bytes, 2)[2:], end =" | ") for _bytes in Up_slave_response]
        data = ((value[8] * 16**6) +(value[7] * 16**4) +(value[6] * 16**2) + value[5])
        print(data)
        return data
    
    elif ID == 2:
        Actuator2_serial.write(Up_master_send)
        Up_slave_response = list(Actuator2_serial.read(10))
        value = Up_slave_response
        [print('0x{0:0{1}X}'.format(_bytes, 2)[2:], end =" | ") for _bytes in Up_slave_response]
        data = ((value[8] * 16**6) +(value[7] * 16**4) +(value[6] * 16**2) + value[5])
        print(data)
        return data
    
    elif ID == 3:
        Actuator3_serial.write(Up_master_send)
        Up_slave_response = list(Actuator3_serial.read(10))
        value = Up_slave_response
        [print('0x{0:0{1}X}'.format(_bytes, 2)[2:], end =" | ") for _bytes in Up_slave_response]
        data = ((value[8] * 16**6) +(value[7] * 16**4) +(value[6] * 16**2) + value[5])
        print(data)
        return data


#
        

#----------------------------------------------------------------------------------------------------------------------------
def Target_speed_rpm(ID,Speed_rpm):
    if Speed_rpm > 20:
        Speed_rpm = 20
    if Speed_rpm < -20:
        Speed_rpm = -20
    Speed_Decimal = math.floor(Speed_rpm*2730.66*Gear_ratio)  ###### remember to add 50 for gearbox effect
    object_value = Speed_Decimal
    value_Hex = Conv_to_hex(object_value)
    value_Hex = Conv_to_hex(object_value)
    Down_master_send = [ID, 0x23, 0xFF, 0x60, 0x00, int(value_Hex[3]), int(value_Hex[2]), int(value_Hex[1]), int(value_Hex[0])]
    Down_master_send.append(Chks_calculation(Down_master_send))
    Down_master_send = bytearray(Down_master_send)


    if ID == 1 : 
        Actuator1_serial.write(Down_master_send)
        Actuator1_serial.flushInput()
    elif ID == 2 : 
        Actuator2_serial.write(Down_master_send)
        Actuator2_serial.flushInput()
    elif ID == 3 : 
        Actuator3_serial.write(Down_master_send)
        Actuator3_serial.flushInput()


#----------------------------------------------------------------------------------------------------------------------------
def Torque_speed_limit (ID,max_current,max_speed):
    Write_register(ID,0x2b,0x60,0x80,0x0,max_speed)
    
def Target_torque (ID,Target_Torque_Nm):
    # Actuator1_serial.flushInput()
    '''rated torque for this type of motor is equal to: 2.68 nm'''
    if Target_Torque_Nm > 20:
        Target_Torque_Nm = 20
    if Target_Torque_Nm < -20:
        Target_Torque_Nm = -20
    Target_Torque_Decimal = math.floor( (Target_Torque_Nm*100*10) / (Rated_torque*Gear_ratio) )  
    object_value = Target_Torque_Decimal
    value_Hex = Conv_to_hex(object_value)
    Down_master_send = [ID, 0x2B, 0x71, 0x60, 0x00, int(value_Hex[3]), int(value_Hex[2]), int(value_Hex[1]), int(value_Hex[0])]
    Down_master_send.append(Chks_calculation(Down_master_send))
    Down_master_send = bytearray(Down_master_send)


    if ID == 1 : 
        Actuator1_serial.write(Down_master_send)
        Actuator1_serial.flushInput()
    if ID == 2 : 
        Actuator2_serial.write(Down_master_send)
        Actuator2_serial.flushInput()
    if ID == 3 : 
        Actuator3_serial.write(Down_master_send)
        Actuator3_serial.flushInput()
    

        
#----------------------------------------------------------------------------------------------------------------------------
global answer2

answer2 = 0

def Enable_all_drivers(mode):
    ## Torque mode ==> mode = 4
    ## Speed Mode ==> mode = -3
    
    global answer2
    
    answer1 = input("is robot in home position? (Y for yes)")
    
    if answer1.upper() != "Y":
        return
    
    Drive_enable(1)
    Drive_enable(2)
    Drive_enable(3)
    Operation_mode(1,mode)
    Operation_mode(2,mode)
    Operation_mode(3,mode)
    #Set speed rpms to 0 for safety
    Target_speed_rpm(1,0)
    Target_speed_rpm(2,0)
    Target_speed_rpm(3,0)
    
    Offset()
    
    answer2 = input("are bars removed? (Y for yes)")
    

#----------------------------------------------------------------------------------------------------------------------------
def Disable_all_drivers():
    Target_speed_rpm(1,0)
    Target_torque(1, 0)
    Drive_disable(1)
    # time.sleep(0.008)
    Target_speed_rpm(2,0)
    Target_torque(2, 0)
    Drive_disable(2)
    # time.sleep(0.008)
    Target_speed_rpm(3,0)
    Target_torque(3, 0)
    Drive_disable(3)
    # time.sleep(0.008)



#----------------------------------------------------------------------------------------------------------------------------    
def Emergency_stop(): # emergency stop 
    Target_speed_rpm(1,0)
    Target_speed_rpm(2,0)  
    Target_speed_rpm(3,0)

#----------------------------------------------------------------------------------------------------------------------------
def Position_convert(value):
    position_hex = ((value[8] * 16**6) +(value[7] * 16**4) +(value[6] * 16**2) + value[5])
    if value[8] >= 255:
        position_hex = -2**32 + position_hex
    position_deg = ((position_hex/10000)*360)/Gear_ratio   ##### remember to add 50 for gearbox
    return position_deg

#----------------------------------------------------------------------------------------------------------------------------
def Velocity_convert(value) :
    velocity_hex = ((value[8] * 16**6) +(value[7] * 16**4) +(value[6] * 16**2) + value[5])
    if value[8] >= 255:
        velocity_hex = -2**32 + velocity_hex
    velocity_rpm = (velocity_hex/2730.66)/Gear_ratio  ##### remember to add 50 for gearbox
    return velocity_rpm

#----------------------------------------------------------------------------------------------------------------------------
def Torque_convert(value)  :
    torque_hex = ((value[8] * 16**6) +(value[7] * 16**4) +(value[6] * 16**2) + value[5])
    if value[8] >= 255:
        torque_hex = -2**32 + torque_hex
    torque_nm = ( (torque_hex*Rated_torque*Gear_ratio) / (10*100) )    ##### remember to add 50 for gearbox
    # torque_nm = torque_hex    ##### remember to add 50 for gearbox

    return torque_nm

#----------------------------------------------------------------------------------------------------------------------------
def Position_actual(ID):

    Up_master_send = [ID, 0x40, 0x63, 0x60, 0x0, 0x0, 0x0, 0x0, 0x0]
    Up_master_send.append(Chks_calculation(Up_master_send))
    Up_master_send = bytearray(Up_master_send)
    num_of_tries = 0
    
    if ID == 1 :

        Actuator1_serial.flushInput()       
        Actuator1_serial.write(Up_master_send)
        Up_slave_response = list(Actuator1_serial.read(10))

        while len(Up_slave_response) !=10 :

            Up_slave_response = list(Actuator1_serial.read(10))
            print("Unable to read position data from Driver={}".format(ID))
            num_of_tries = num_of_tries + 1
            if num_of_tries == max_tries:
                print("Max Tries Unable to read position data from Driver={}".format(ID))
                break
            
    elif ID == 2 :

        Actuator2_serial.flushInput()      
        Actuator2_serial.write(Up_master_send)
        Up_slave_response = list(Actuator2_serial.read(10))

        while len(Up_slave_response) !=10 :

            Up_slave_response = list(Actuator2_serial.read(10))
            print("Unable to read position data from Driver={}".format(ID))
            num_of_tries = num_of_tries + 1
            if num_of_tries == max_tries:
                print("Max Tries Unable to read position data from Driver={}".format(ID))
                break       
            
    elif ID == 3 :

        Actuator3_serial.flushInput()   
        Actuator3_serial.write(Up_master_send)
        Up_slave_response = list(Actuator3_serial.read(10))

        while len(Up_slave_response) !=10 :

            Up_slave_response = list(Actuator3_serial.read(10))
            print("Unable to read position data from Driver={}".format(ID))
            num_of_tries = num_of_tries + 1
            if num_of_tries == max_tries:
                print("Max Tries Unable to read position data from Driver={}".format(ID))
                break
            
        
    if ( len(Up_slave_response)!=10 ):
        return -333333
    position_deg = Position_convert(Up_slave_response)
    # while abs(position_deg) > (5000/Gear_ratio) :
        # return -333333


    return position_deg 
    
    

    
#----------------------------------------------------------------------------------------------------------------------------
# global offset_pos
def Offset():
    global offset_pos
    # offset_1 = Position_absolute_read(1)
    # offset_2 = Position_absolute_read(2)
    # offset_3 = Position_absolute_read(3)

    
    offset_1 = -68
    offset_2 = -67.8816
    offset_3 = -67.93704000000001
    
    # offset_1 = 25
    # offset_2 = 27
    # offset_3 = 22

    
    offset_pos = [offset_1, offset_2, offset_3]
    
    print("This is offset:", offset_pos)
    
    # return offset_pos

    

#----------------------------------------------------------------------------------------------------------------------------    
def Position_absolute_read(ID):
    global offset_pos
    if 'offset_pos' not in globals():
        offset_pos = [0, 0, 0]
    Pos_relative_read = Position_actual(ID)
    num_of_tries = 0
    while Pos_relative_read == -333333:
        Pos_relative_read = Position_actual(ID)
        num_of_tries += 1
        if num_of_tries > max_tries :
            Emergency_stop()
            Disable_all_drivers()
            print("fault in position data readings from driver={}".format(ID))
            break
    # Position_abs = [0,0,0]
    # position_abs = Pos_relative_read - offset_pos[ID-1]
    position_abs =  Position_actual(ID) - offset_pos[ID-1]
    # print(position_abs)
    return position_abs


#----------------------------------------------------------------------------------------------------------------------------
def Velocity_actual_rpm(ID) :
    Actuator1_serial.flushInput()
    Actuator2_serial.flushInput()    
    Actuator3_serial.flushInput()
    '''torque: 6077'''
    '''speed:  606C'''
    Up_master_send = [ID, 0x40, 0x6C, 0x60, 0x0, 0x0, 0x0, 0x0, 0x0]
    Up_master_send.append(Chks_calculation(Up_master_send))
    Up_master_send = bytearray(Up_master_send)
    num_of_tries = 0
    
    
    if ID == 1 :
        # Actuator1_serial.flushInput()
        Actuator1_serial.write(Up_master_send)
        Actuator1_serial.flushInput()
        Up_slave_response = list(Actuator1_serial.read(10))
        Actuator1_serial.flushInput()   
        while len(Up_slave_response) !=10 :
            print("Unable to read velocity data from Driver={}".format(ID))
            Up_slave_response = list(Actuator1_serial.read(10))
            num_of_tries = num_of_tries + 1
            if num_of_tries == max_tries:
                print("Max Tries Unable to read velocity data from Driver={}".format(ID))
                break
    
    
    elif ID == 2 :
        # Actuator1_serial.flushInput()
        Actuator2_serial.write(Up_master_send)
        Actuator2_serial.flushInput()
        Up_slave_response = list(Actuator2_serial.read(10))
        Actuator2_serial.flushInput()   
        while len(Up_slave_response) !=10 :
            print("Unable to read velocity data from Driver={}".format(ID))
            Up_slave_response = list(Actuator2_serial.read(10))
            num_of_tries = num_of_tries + 1
            if num_of_tries == max_tries:
                print("Max Tries Unable to read velocity data from Driver={}".format(ID))
                break
    
    elif ID == 3 :
        # Actuator1_serial.flushInput()
        Actuator3_serial.write(Up_master_send)
        Actuator3_serial.flushInput()
        Up_slave_response = list(Actuator3_serial.read(10))
        Actuator3_serial.flushInput()   
        while len(Up_slave_response) !=10 :
            print("Unable to read velocity data from Driver={}".format(ID))
            Up_slave_response = list(Actuator3_serial.read(10))
            num_of_tries = num_of_tries + 1
            if num_of_tries == max_tries:
                print("Max Tries Unable to read velocity data from Driver={}".format(ID))
                break
    
   
            
        
    if ( len(Up_slave_response)!=10 ):
        return -666666
    velocity_rpm = Velocity_convert(Up_slave_response)
    # while abs(velocity_rpm) > (3000/Gear_ratio) :
        # return -6666666

    return velocity_rpm


        
#----------------------------------------------------------------------------------------------------------------------------
def Velocity_read_rpm(ID):
    velocity_rpm = Velocity_actual_rpm(ID)
    num_of_tries = 0
    while velocity_rpm == -666666 :
        velocity_rpm = Velocity_actual_rpm(ID)
        num_of_tries += 1
        if num_of_tries > max_tries :
            Emergency_stop()
            Disable_all_drivers()
            print("fault in velocity data readings from driver={}".format(ID))
            break
    return velocity_rpm

#----------------------------------------------------------------------------------------------------------------------------

def Torque_actual(ID):
    Actuator1_serial.flushInput()
    Actuator2_serial.flushInput()    
    Actuator3_serial.flushInput()

    '''torque: 6077'''
    Up_master_send = [ID, 0x40, 0x77, 0x60, 0x0, 0x0, 0x0, 0x0, 0x0]
    Up_master_send.append(Chks_calculation(Up_master_send))
    Up_master_send = bytearray(Up_master_send)
    num_of_tries = 0
    
    if ID == 1 :
        # Actuator1_serial.flushInput()
        Actuator1_serial.write(Up_master_send)
        Actuator1_serial.flushInput()
        Up_slave_response = list(Actuator1_serial.read(10))
        Actuator1_serial.flushInput()   
        while len(Up_slave_response) !=10 :
            print("Unable to read torque data from Driver={}".format(ID))
            Up_slave_response = list(Actuator1_serial.read(10))
            num_of_tries = num_of_tries + 1
            if num_of_tries == max_tries:
                print("Max Tries Unable to read torque data from Driver={}".format(ID))
                break
            

    
    elif ID == 2 :
        # Actuator1_serial.flushInput()
        Actuator2_serial.write(Up_master_send)
        Actuator2_serial.flushInput()
        Up_slave_response = list(Actuator2_serial.read(10))
        Actuator2_serial.flushInput()   
        while len(Up_slave_response) !=10 :
            print("Unable to read torque data from Driver={}".format(ID))
            Up_slave_response = list(Actuator2_serial.read(10))
            num_of_tries = num_of_tries + 1
            if num_of_tries == max_tries:
                print("Max Tries Unable to read torque data from Driver={}".format(ID))
                break
            
            
    elif ID == 3 :
        # Actuator1_serial.flushInput()
        Actuator3_serial.write(Up_master_send)
        Actuator3_serial.flushInput()
        Up_slave_response = list(Actuator3_serial.read(10))
        Actuator3_serial.flushInput()   
        while len(Up_slave_response) !=10 :
            print("Unable to read torque data from Driver={}".format(ID))
            Up_slave_response = list(Actuator3_serial.read(10))
            num_of_tries = num_of_tries + 1
            if num_of_tries == max_tries:
                print("Max Tries Unable to read torque data from Driver={}".format(ID))
                break
        
    

    if ( len(Up_slave_response)!=10 ):
        return -999999
    torque_nm = Torque_convert(Up_slave_response)
    # while abs(torque_nm) > (3000*Gear_ratio) :
        # return -999999

    return torque_nm
     
        
def Torque_read_nm(ID):
    torque_nm = Torque_actual(ID)
    num_of_tries = 0
    while torque_nm == -999999 :
        torque_nm = Torque_actual(ID)
        num_of_tries += 1
        if num_of_tries > max_tries :
            Emergency_stop()
            Disable_all_drivers()
            print("fault in torque data readings from driver={}".format(ID))
            break
    return torque_nm


#----------------------------------------------------------------------------------------------------------------------------

def Read_all_positions():
    print("Position 1 (deg)=", Position_absolute_read(1))
#    time.sleep(.01)
    print("Position 2 (deg)=", Position_absolute_read(2))
#    time.sleep(.01)  
    print("Position 3 (deg)=", Position_absolute_read(3))
#    time.sleep(.008)  


#----------------------------------------------------------------------------------------------------------------------------
def Motion_z_endeffector(speed):
    Target_speed_rpm(1,speed)
    Target_speed_rpm(2,speed)
    Target_speed_rpm(3,speed)
        
def Motion_y_endeffector(speed):
    Target_speed_rpm(1,-1*speed)
    Target_speed_rpm(2,speed)
    Target_speed_rpm(3,speed)

def Motion_x_endeffector(speed):
    Target_speed_rpm(1,speed)
    Target_speed_rpm(2,speed)
    Target_speed_rpm(3,-1*speed)

def Motion_theta1(speed):
    Target_speed_rpm(1, speed)
    Target_speed_rpm(2, 0)
    Target_speed_rpm(3, 0)
    
def Motion_theta2(speed):
    Target_speed_rpm(1, 0)
    Target_speed_rpm(2, speed)
    Target_speed_rpm(3, 0)

def Motion_theta3(speed):
    Target_speed_rpm(1, 0)
    Target_speed_rpm(2, 0)
    Target_speed_rpm(3, speed)
#----------------------------------------------------------------------------------------------------------------------------

def _is_point_inside_triangle(P):

    # Set vertices for the triangle
    vertices = np.array([[15, 35], [-35, -5], [25, -33]])

    A, B, C = vertices

    # Calculate the vectors from A to the test point P
    v0 = C - A
    v1 = B - A
    v2 = P - A

    # Calculate dot products
    dot00 = np.dot(v0, v0)
    dot01 = np.dot(v0, v1)
    dot02 = np.dot(v0, v2)
    dot11 = np.dot(v1, v1)
    dot12 = np.dot(v1, v2)

    # Calculate barycentric coordinates
    inv_denom = 1.0 / (dot00 * dot11 - dot01 * dot01)
    u = (dot11 * dot02 - dot01 * dot12) * inv_denom
    v = (dot00 * dot12 - dot01 * dot02) * inv_denom

    # Check if the point is inside the triangle
    return (u >= 0) and (v >= 0) and (u + v <= 1)


# final_xyz = [x_final, y_final, z_final]
def Goto_xyz(final_xyz, duration, trajectory='4567'):
    
    global answer2 

    # # safety
    # if answer2.upper() != "Y":
    #     return 

    # if not(_is_point_inside_triangle(final_xyz[0:2]) and (final_xyz[2] <= -37) and (final_xyz[2] >= -70)):
    #     return 
    
    start_time = datetime.datetime.now()
    
    # print("Start Time is: ", start_time)

    # init values 
    current_position    = [0, 0, 0]
    last_position       = [0, 0, 0]
    distance            = [0, 0, 0]
        
    E, current_position[0], current_position[1], current_position[2] = Forward(Position_absolute_read(1), Position_absolute_read(2), Position_absolute_read(3))

    print("Current Position is: ", current_position)

    # calculating the distance from the goal
    for i in range(3):
        distance[i] = final_xyz[i] - current_position[i]
    
    print("distance:", distance)
        
    dtime = 0 

    while dtime<duration:

        # safety 
        if keyboard.is_pressed('q'):
            Emergency_stop()
            print("Loop terminated by user!")
            break

        # checking the passed time 
        last_time = datetime.datetime.now()
        dtime = (last_time - start_time).total_seconds()

        if dtime>=duration:
            Motion_z_endeffector(0)
            Motion_z_endeffector(0)
            break

        # trajectory 
        tau = dtime/duration
        if trajectory=='4567':
            s = trajectory_4567(duration, 0, start_time)
        elif trajectory=='345':
            s = trajectory_345(duration, 0, start_time)
        elif trajectory=='trapezoidal':
            s = trajectory_trapezoidal(duration, 0, start_time)

        for i in range(3): 
            last_position[i] = s*distance[i] + current_position[i]

        in1 = Position_absolute_read(1)
        in2 = Position_absolute_read(2)
        in3 = Position_absolute_read(3)

        feedback = [in1, in2, in3]

        system_input = implement_PID(last_position, feedback)

        Target_speed_rpm(1, system_input[0])
        Target_speed_rpm(2, system_input[1])
        Target_speed_rpm(3, system_input[2])


def trajectory_4567(time_map,time_shift,start_time): #return value within 0 , 1
    last_time=datetime.datetime.now()
    dtime=(last_time-start_time).total_seconds()
    tau=(dtime-time_shift)/time_map
    s=-20*(tau**7)+70*(tau**6)-84*(tau**5)+35*(tau**4)
    return s


def trajectory_345(time_map, time_shift, start_time):
    last_time = datetime.datetime.now()
    dtime = (last_time - start_time).total_seconds()
    tau = (dtime - time_shift)/time_map
    s = 6*(tau**5) - 15*(tau**4) + 10*(tau**3)
    return s 

def trajectory_trapezoidal(time_map, time_shift, start_time):
    last_time = datetime.datetime.now()
    dtime = (last_time - start_time).total_seconds()
    tau = (dtime - time_shift)/time_map

    # acceleration = 4.5 and v_max = 1.5 and T = 1 

    if tau<=1/3 and tau>=0:
        s = 2.25*tau**2
    elif tau<=2/3 and tau>=1/3:
        s = 1.5*tau - 0.25 
    elif tau<=1 and tau>=2/3:
        s = - 1.25 + 4.5*tau - 2.25*tau**2

    return s 

#----------------------------------------------------------------------------------------------------------------------------

# path = [[x_1, y_1, z_1], [x_2, y_2, z_2]]
def mltp_3point(path, duration):

    global answer2 

    # init values 
    current_position    = [0, 0, 0]
    last_position       = [0, 0, 0]

    E, current_position[0], current_position[1], current_position[2] = Forward(Position_absolute_read(1), Position_absolute_read(2), Position_absolute_read(3))

    print("Current Position is: ", current_position)

    # path should be: [[x0, y0, z0], [x1, y1, z1], [x2, y2, z2]]
    path = current_position + path 
    path_transpose = path.T

    print("This  is path:", path)

    start_time = datetime.datetime.now()
    dtime = 0 

    while dtime<duration:

        # safety 
        if keyboard.is_pressed('q'):
            Emergency_stop()
            print("Loop terminated by user!")
            break

        # checking the passed time 
        last_time = datetime.datetime.now() 
        dtime = (last_time - start_time).total_seconds()

        if dtime >= duration: 
            Motion_z_endeffector(0)
            break 

        # trajectory 
        for i in range(3):
             last_position[i] = _trajectory_3point(time, path_transpose[i])


        in1 = Position_absolute_read(1)
        in2 = Position_absolute_read(2)
        in3 = Position_absolute_read(3)

        feedback = [in1, in2, in3]

        system_input = implement_PID(last_position, feedback)

        Target_speed_rpm(1, system_input[0])
        Target_speed_rpm(2, system_input[1])
        Target_speed_rpm(3, system_input[2])



# points = [q0, q1, q2] and not normalized
def _trajectory_3point(time, points):

    [q0, q1, q2] = points 

    last_time = datetime.datetime.now()
    dtime = (last_time - start_time).total_seconds()

    # acceleration = 4.5 and v_max = 1.5 and T = 1 

    a0 = q0 
    a4 = 256*q1 - 163*q0 - 93*q2 
    a5 = 596*q0 - 1024*q1 + 428*q2 
    a6 = 1536*q1 - 838*q0 - 698*q2
    a7 = 532*q0 - 1024*q1 + 492*q2 
    a8 = 256*q1 - 128*q0 - 128*q2 

    s = a8*dtime**8 + a7*dtime**7 + a6*dtime**6 + a5*dtime**5 + a4*dtime**4 + a0

    return s 

#----------------------------------------------------------------------------------------------------------------------------

# path = [[x1, y1, z1], [x2, y2,z2], ... [xn, yn, zn]]
# def mltp_cubic_spline(path, duration):

#     global answer2

#     # init values 
#     current_position    = [0, 0, 0]
#     last_position       = [0, 0, 0]

#     E, current_position[0], current_position[1], current_position[2] = Forward(Position_absolute_read(1), Position_absolute_read(2), Position_absolute_read(3))

#     print("Current Position is: ", current_position)

#     # the path should become: [[x0, y0, z0], [x1, y1, z1], ... [xn, yn, zn]]
#     path = current_position + path 
#     path_transpose = path.T
#     n = path.shape[0]
    
#     print("This is the path:", path, "\nn=", n)
        
#     start_time = datetime.datetime.now()
#     dtime = 0 

#     while dtime<duration:
        
#         # safety 
#         if keyboard.is_pressed('q'):
#             Emergency_stop()
#             print("Loop terminated by user!")
#             break

#         # checking the passed time 
#         last_time = datetime.datetime.now()
#         dtime = (last_time - start_time).total_seconds()


#         if dtime>=duration:
#             Motion_z_endeffector(0)
#             break

#         for i in range(3): 
#             last_position[i] = _trajectory_cubic_spline(time, path_transpose[i], duration)
 
#         in1 = Position_absolute_read(1)
#         in2 = Position_absolute_read(2)
#         in3 = Position_absolute_read(3)

#         feedback = [in1, in2, in3]

#         system_input = implement_PID(last_position, feedback)

#         Target_speed_rpm(1, system_input[0])
#         Target_speed_rpm(2, system_input[1])
#         Target_speed_rpm(3, system_input[2])


# def _trajectory_cubic_spline(time, points, duration):

#     # the number of points 
#     n = points.shape[0]
#     T = duration

#     # calculating coefficient matrix 
#     coeff = CubicSplineCoeff(points, T)


#     return s 


#----------------------------------------------------------------------------------------------------------------------------



def EE_manual_controller(movement_speed=0.1):
    pygame.init()

    # Set up the display
    screen = pygame.display.set_mode((400, 300))
    pygame.display.set_caption("DPR EE Manual Controller")

    # Keep track of key states
    key_up_pressed = False
    key_down_pressed = False
    key_left_pressed = False
    key_right_pressed = False
    key_w_pressed = False
    key_s_pressed = False
    key_q_pressed = False

    is_key_pressed =    key_up_pressed or key_down_pressed or key_left_pressed or key_right_pressed \
                        or key_w_pressed or key_s_pressed

    running = True

    while running:
        for event in pygame.event.get():

            # checking if any key is pressed 
            is_key_pressed =    key_up_pressed or key_down_pressed or key_left_pressed or key_right_pressed \
                                or key_w_pressed or key_s_pressed

            if event.type == pygame.KEYDOWN and event.key == K_q:
                Motion_z_endeffector(0)
                pygame.quit()
                sys.exit()
                return 
            if event.type == pygame.KEYDOWN and event.key == K_SPACE:
                print(Forward(Position_absolute_read(1), Position_absolute_read(2), Position_absolute_read(3)))
            elif event.type == pygame.KEYDOWN and not is_key_pressed:
                
                if event.key == K_UP:
                    print("key up is pressed")
                    Motion_y_endeffector(movement_speed)
                    key_up_pressed = True
                elif event.key == K_DOWN:
                    print("key down is pressed")
                    Motion_y_endeffector(-movement_speed)
                    key_down_pressed = True
                elif event.key == K_LEFT:
                    print("key left is pressed")
                    Motion_x_endeffector(movement_speed)
                    key_left_pressed = True
                elif event.key == K_RIGHT:
                    print("key right is pressed")
                    Motion_x_endeffector(-movement_speed)
                    key_right_pressed = True
                elif event.key == K_w:
                    print("key w is pressed")
                    Motion_z_endeffector(-movement_speed)
                    key_w_pressed = True
                elif event.key == K_s:
                    print("key s is pressed")
                    Motion_z_endeffector(movement_speed)
                    key_s_pressed = True

            elif event.type == pygame.KEYUP:
                if event.key == K_UP and not (is_key_pressed and not key_up_pressed):
                    print("key up is released")
                    Motion_z_endeffector(0)
                    key_up_pressed = False
                elif event.key == K_DOWN and not (is_key_pressed and not key_down_pressed):
                    print("key down is released")
                    Motion_z_endeffector(0)
                    key_down_pressed = False
                elif event.key == K_LEFT and not (is_key_pressed and not key_left_pressed):
                    print("key left is released")
                    Motion_z_endeffector(0)
                    key_left_pressed = False
                elif event.key == K_RIGHT and not (is_key_pressed and not key_right_pressed):
                    print("key right is released")
                    Motion_z_endeffector(0)
                    key_right_pressed = False
                elif event.key == K_w and not (is_key_pressed and not key_w_pressed):
                    print("key w is released")
                    Motion_z_endeffector(0)
                    key_w_pressed = False
                elif event.key == K_s and not (is_key_pressed and not key_s_pressed):
                    print("key s is released")
                    Motion_z_endeffector(0)
                    key_s_pressed = False

    pygame.quit()


#----------------------------------------------------------------------------------------------------------------------------
 
e= (1/math.tan(np.deg2rad(30))) * 20
f= (1/math.tan(np.deg2rad(30))) * 26
re = 59.5
rf =  30.9

#s      = 165*2
sqrt3  = math.sqrt(3.0)
pi     = 3.141592653
sin120 = sqrt3 / 2.0
cos120 = -0.5
tan60  = sqrt3
sin30  = 0.5
tan30  = 1.0 / sqrt3

def Forward(theta1, theta2, theta3):
    x0 = 0.0
    y0 = 0.0
    z0 = 0.0
    
    t = (f-e) * tan30 / 2.0
    dtr = pi / 180.0
    
    theta1 *= dtr
    theta2 *= dtr
    theta3 *= dtr
    
    y1 = -(t + rf*math.cos(theta1) )
    z1 = -rf * math.sin(theta1)
    
    y2 = (t + rf*math.cos(theta2)) * sin30
    x2 = y2 * tan60
    z2 = -rf * math.sin(theta2)
    
    y3 = (t + rf*math.cos(theta3)) * sin30
    x3 = -y3 * tan60
    z3 = -rf * math.sin(theta3)
    
    dnm = (y2-y1)*x3 - (y3-y1)*x2
    
    w1 = y1*y1 + z1*z1
    w2 = x2*x2 + y2*y2 + z2*z2
    w3 = x3*x3 + y3*y3 + z3*z3
    
    # x = (a1*z + b1)/dnm
    a1 = (z2-z1)*(y3-y1) - (z3-z1)*(y2-y1)
    b1= -( (w2-w1)*(y3-y1) - (w3-w1)*(y2-y1) ) / 2.0
    
    # y = (a2*z + b2)/dnm
    a2 = -(z2-z1)*x3 + (z3-z1)*x2
    b2 = ( (w2-w1)*x3 - (w3-w1)*x2) / 2.0
    
    # a*z^2 + b*z + c = 0
    a = a1*a1 + a2*a2 + dnm*dnm
    b = 2.0 * (a1*b1 + a2*(b2 - y1*dnm) - z1*dnm*dnm)
    c = (b2 - y1*dnm)*(b2 - y1*dnm) + b1*b1 + dnm*dnm*(z1*z1 - re*re)

    d = b*b - 4.0*a*c
    if d < 0.0:
        return [1,0,0,0] 
    
    z0 = -0.5*(b + math.sqrt(d)) / a
    x0 = (a1*z0 + b1) / dnm
    y0 = (a2*z0 + b2) / dnm

    return [0,x0,y0,z0]

# Inverse kinematics

def _angle_yz(x0, y0, z0, theta=None):
    y1 = -0.5*0.57735*f # f/2 * tg 30
    y0 -= 0.5*0.57735*e # shift center to edge
    # z = a + b*y
    a = (x0*x0 + y0*y0 + z0*z0 + rf*rf - re*re - y1*y1) / (2.0*z0)
    b = (y1-y0) / z0

    d = -(a + b*y1)*(a + b*y1) + rf*(b*b*rf + rf)
    if d<0:
        return [1,0] 

    yj = (y1 - a*b - math.sqrt(d)) / (b*b + 1) 
    zj = a + b*yj
    theta = math.atan(-zj / (y1-yj)) * 180.0 / pi + (180.0 if yj>y1 else 0.0)
    
    return [0,theta] # return error, theta

def Inverse(x0, y0, z0):
    theta1 = 0
    theta2 = 0
    theta3 = 0
    status = _angle_yz(x0,y0,z0)

    if status[0] == 0:
        theta1 = status[1]
        status = _angle_yz(x0*cos120 + y0*sin120,
                                   y0*cos120-x0*sin120,
                                   z0,
                                   theta2)
    if status[0] == 0:
        theta2 = status[1]
        status = _angle_yz(x0*cos120 - y0*sin120,
                                   y0*cos120 + x0*sin120,
                                   z0,
                                   theta3)
    theta3 = status[1]

    return [status[0],theta1,theta2,theta3]




# =================================================================================================
# -- PID ------------------------------------------------------------------------------------------
# =================================================================================================


import datetime

class PID:
    def __init__(self,Kp,Ki,Kd,setPoint=0,SampleTime=60): #sample time is in millisconds
        if Kp<0 or Kd<0 or Ki<0:
            print("invalid pid constants")
            return
        self.SampleTime=SampleTime
        sampleTimeInSec=SampleTime/1000 
        self.kp=Kp
        self.ki=Ki*sampleTimeInSec
        self.kd=Kd/sampleTimeInSec
        self.lastError=0
        self.integralTerm=0 #used for I term
        self.integralTerm_torque = 0 #used for I term
        self.lastTime=datetime.datetime.now()
#        startTime=startTime.seconds()*1000
        self.minOutput=0
        self.maxOutput=0
        self.Error=0


    def Compute(self,feedback):
        presentTime=datetime.datetime.now()
        timeChange=(presentTime-self.lastTime).total_seconds()*1000

        if timeChange>self.SampleTime: #if a time interval equal to sample time has passed
            #Compute Working Error Variables
            self.Error = self.setPoint-feedback
            # print("this is error: " + str(self.Error))
            dError = self.Error-self.lastError #error- last error
            
            self.integralTerm = self.integralTerm+self.ki*self.Error

            derivativeTerm = self.kd*dError
            
            proportionalTerm = self.kp*self.Error
            
            PIDOutput = self.integralTerm + derivativeTerm + proportionalTerm
            
            if self.maxOutput != 0 and PIDOutput>self.maxOutput:
                PIDOutput=self.maxOutput
                
            elif self.minOutput != 0 and PIDOutput<self.minOutput:
                PIDOutput=self.minOutput
                
            return PIDOutput
        
        self.lastTime = presentTime
            
        
    def Compute_torque_pid(self,feedback):
        
        # Mass_matrix = I_bt + m_ee_total*(J_x).T * J_x

        # Torque_grav = -(J_x).T * ( m_ee_total + (3/2)*m_forearm )*[0 , 0 , 9.81] - L1*(0.5*m_upper_arm + m_middle_joint + 0.5*m_forearm)*9.81*[np.cos(theta_des_actlink(1)) , np.cos(theta_des_actlink(2)) , np.cos(theta_des_actlink(3))] 

        
        presentTime=datetime.datetime.now()
        timeChange=(presentTime-self.lastTime).total_seconds()*1000

        if timeChange>self.SampleTime: #if a time interval equal to sample time has passed
            
            #Compute Working Error Variables
            self.Error = self.setPoint-feedback
            # print("this is error: " + str(self.Error))
            dError = self.Error - self.lastError #error- last error
            
            self.integralTerm_torque = -self.integralTerm + self.ki*self.Error*4

            derivativeTerm_torque = self.kd*dError*-15
            
            proportionalTerm_torque = self.kp*self.Error*-12
            
            PIDOutput_torque = -(self.integralTerm_torque + derivativeTerm_torque + proportionalTerm_torque )
            
            if self.maxOutput!=0 and PIDOutput_torque >self.maxOutput:
                PIDOutput_torque = self.maxOutput
            elif self.minOutput!=0 and PIDOutput_torque <self.minOutput:
                PIDOutput_torque=self.minOutput
            return PIDOutput_torque
        
        self.lastTime=presentTime
        
        
        
        
        

    def SetSampleTime(self,newSampleTime):
        ratio = newSampleTime/self.SampleTime
        self.ki = self.ki*ratio
        self.kd = self.kd/ratio
        self.SampleTime = newSampleTime

    def SetOutputLimits(self,minOut,maxOut):
        self.minOutput = minOut
        self.maxOutput = maxOut
        
    def DefineSetpoint(self,coord):
        self.setPoint = coord
        
        
    def set_PID_constants(self,Kp,Ki,Kd):
        self.kp = Kp
        self.ki = Ki
        self.kd = Kd
        

pids=[]
kp = 0.1
ki = 0.01
kd = 0.01
#3 pids for 3 angles

positionReadFailed=0

pid1=PID(kp,ki,kd)
pid2=PID(kp,ki,kd)
pid3=PID(kp,ki,kd)

def implement_PID(set_point_coord,feedback):
    controllerOutput = []
    #converting xyz coord to angle by inverse kinematics
    E,theta1,theta2,theta3=Inverse(set_point_coord[0],set_point_coord[1],set_point_coord[2])
    
    # theta1 = set_point_coord[0]
    # theta2 = set_point_coord[1]
    # theta3 = set_point_coord[2]
    
    
    #system input is the return value of controller
    # print("thetaas : 1: {}, 2: {}, 3: {} \n".format(theta1,theta2,theta3))
    pid1.DefineSetpoint(theta1)
    pid2.DefineSetpoint(theta2)
    pid3.DefineSetpoint(theta3)
    
#    in1=Pos_Actual_r(1)
#    in2=Pos_Actual_r(2)
#    in3=Pos_Actual_r(3)
#    print("ins: {}, {} , {} ".format(in1,in2,in3))
#    if not (in1>=0) or not (in2>=0) or not (in3>=0):
#        return positionReadFailed

    controllerOutput.append(pid1.Compute(feedback[0]))
    controllerOutput.append(pid2.Compute(feedback[1]))
    controllerOutput.append(pid3.Compute(feedback[2]))
    
    # print("controlleroutput : 1: {}, 2: {}, 3: {} \n".format(controllerOutput[0],controllerOutput[1],controllerOutput[2]))

    return controllerOutput
    
def implement_PID_torque(set_point_coord,feedback):
    controllerOutput_torque = []
    #converting xyz coord to angle by inverse kinematics
    E,theta1,theta2,theta3=Inverse(set_point_coord[0],set_point_coord[1],set_point_coord[2])
    #system input is the return value of controller
    # print("thetaas : 1: {}, 2: {}, 3: {} \n".format(theta1,theta2,theta3))
    
    
    
    # pid1.DefineSetpoint(theta1)
    # pid2.DefineSetpoint(theta2)
    # pid3.DefineSetpoint(theta3)
    
#    in1=Pos_Actual_r(1)
#    in2=Pos_Actual_r(2)
#    in3=Pos_Actual_r(3)
#    print("ins: {}, {} , {} ".format(in1,in2,in3))
#    if not (in1>=0) or not (in2>=0) or not (in3>=0):
#        return positionReadFailed

    
    controllerOutput_torque.append(pid1.Compute_torque_pid(feedback[0]))
    controllerOutput_torque.append(pid2.Compute_torque_pid(feedback[1]))
    controllerOutput_torque.append(pid3.Compute_torque_pid(feedback[2]))
    
    
    # print("controlleroutput : 1: {}, 2: {}, 3: {} \n".format(controllerOutput[0],controllerOutput[1],controllerOutput[2]))

    return controllerOutput_torque
    



# =================================================================================================
# -- motion ---------------------------------------------------------------------------------------
# =================================================================================================


def circle_motion(center,radi,height,n,period):  #position and lengths in cm -- time in seconds
    first_time=datetime.datetime.now()
    dtime=0
    
    position_feedback_1 = []
    position_feedback_2 = []
    position_feedback_3 = []
    
    velocity_feedback_1 = []
    velocity_feedback_2 = []
    velocity_feedback_3 = []
    
    Controller1_history = []
    Controller2_history = []
    Controller3_history = []

    
    torque_feedback_1 = []
    torque_feedback_2 = []
    torque_feedback_3 = []
    
    torque_cmd_his_1 = []
    torque_cmd_his_2 = []
    torque_cmd_his_3 = []
        
    x_des = []
    y_des = []
    z_des = []
    
    
    while dtime<n*period:
        last_time=datetime.datetime.now()
        dtime=(last_time-first_time).total_seconds()
        
        px = radi * np.cos(np.pi*2/period*dtime) + center[0] 
        py = radi * np.sin(np.pi*2/period*dtime) + center[1]
        pz=height
        
       
         
        x_des.append(px)
        y_des.append(py)
        z_des.append(pz)
        
        desired_path = np.array([x_des, y_des, z_des])
#        go_to([px,py,pz])
####

        
        in1 = Position_absolute_read(1)        
        in2 = Position_absolute_read(2)        
        in3 = Position_absolute_read(3)
        
        position_feedback_1.append(in1)
        position_feedback_2.append(in2)
        position_feedback_3.append(in3)
        position_feedback = np.array([position_feedback_1,position_feedback_2,position_feedback_3])        

        
        
        feedback = [in1, in2, in3]
        
        system_input = implement_PID([px,py,pz],feedback) 
        
        torque_control = implement_PID_torque([px,py,pz],feedback) 
        
        torque_cmd_his_1.append(torque_control[0]-7)
        torque_cmd_his_2.append(torque_control[1]-7)
        torque_cmd_his_3.append(torque_control[2]-7)
        torque_cmd_his = np.array([torque_cmd_his_1, torque_cmd_his_2, torque_cmd_his_3])
        
        # Target_torque(1,torque_control[0]-5)
        # # time.sleep(0.001)
        # Target_torque(2,torque_control[1]-5)
        # # time.sleep(0.001)
        # Target_torque(3,torque_control[2]-5)


        
        Target_speed_rpm(1,system_input[0])
        Target_speed_rpm(2,system_input[1])
        Target_speed_rpm(3,system_input[2])
        

        
        Controller1_history.append(system_input[0])
        Controller2_history.append(system_input[1])
        Controller3_history.append(system_input[2])
        Controller_history = np.array([Controller1_history, Controller2_history, Controller3_history])  

        

        velocity_feedback_1.append(Velocity_actual_rpm(1))
        velocity_feedback_2.append(Velocity_actual_rpm(2))
        velocity_feedback_3.append(Velocity_actual_rpm(3))
        velocity_feedback = np.array([velocity_feedback_1,velocity_feedback_2,velocity_feedback_3])        

        torque_feedback_1.append(Torque_actual(1))
        torque_feedback_2.append(Torque_actual(2))
        torque_feedback_3.append(Torque_actual(3))
        torque_feedback = np.array([torque_feedback_1,torque_feedback_2,torque_feedback_3])  
        
####

        if dtime==n*period:
            print("encircled {} times!".format(n))
            Motion_z_endeffector(0)
            
            # Target_torque(1,0)
            # Target_torque(2,0)
            # Target_torque(3,0)
        
            break
    Motion_z_endeffector(0)
    # Target_torque(1, 0)
    # Target_torque(2, 0)
    # Target_torque(3, 0)
    # Disable_all_drivers()
    
    return position_feedback, velocity_feedback, torque_feedback, desired_path, Controller_history, torque_cmd_his

# =================================================================================================
# -- main function --------------------------------------------------------------------------------
# =================================================================================================

def main():
    pass

# =================================================================================================
# -------------------------------------------------------------------------------------------------
# =================================================================================================


if __name__=="__main__":
    main()
