

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

        print("reading ID==2")

        Actuator2_serial.flushInput()      

        print("flushed input")

        Actuator2_serial.write(Up_master_send)

        print("serial write finsihed")

        Up_slave_response = list(Actuator2_serial.read(10))

        print("serial read finished")

        while len(Up_slave_response) !=10 :

            print("entered while loop")
            Up_slave_response = list(Actuator2_serial.read(10))
            print("Unable to read position data from Driver={}".format(ID))
            num_of_tries = num_of_tries + 1
            if num_of_tries == max_tries:
                print("Max Tries Unable to read position data from Driver={}".format(ID))
                break       
            
    elif ID == 3 :

        print("reading ID==3")

        Actuator3_serial.flushInput()   

        print("flushed input")
       
        Actuator3_serial.write(Up_master_send)

        print("serial write finsihed")

        Up_slave_response = list(Actuator3_serial.read(10))

        print("serial read finished")

        while len(Up_slave_response) !=10 :

            print("entered while loop")
            Up_slave_response = list(Actuator3_serial.read(10))
            print("Unable to read position data from Driver={}".format(ID))
            num_of_tries = num_of_tries + 1
            if num_of_tries == max_tries:
                print("Max Tries Unable to read position data from Driver={}".format(ID))
                break
            
    print("the function is about to end")
        
    if ( len(Up_slave_response)!=10 ):
        return -333333
    position_deg = Position_convert(Up_slave_response)
    # while abs(position_deg) > (5000/Gear_ratio) :
        # return -333333

    print("the function ended")

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

    
def Goto_xyz(final_xyz, duration):
    
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
        
    # # print("Distance is: ", distance)

    dtime = 0 

    while dtime<duration:

        # print("enter loop")

        # safety 
        if keyboard.is_pressed('q'):
            Emergency_stop()
            Disable_all_drivers()
            print("Loop terminated by user.")
            break


        # print("safety is passed")

        # checking the passed time 
        last_time = datetime.datetime.now()
        dtime = (last_time - start_time).total_seconds()

        # print("time is calced")

        if dtime>=duration:
            Motion_z_endeffector(0)
            Motion_z_endeffector(0)
            break


        # print("time finish is checked")


        # trajectory 
        tau = dtime/duration
        s = trajectory_4567(duration, 0, start_time)

        # print("some calculations are done")

        for i in range(3): 
            last_position[i] = s*distance[i] + current_position[i]

        # print("some more calculations are done")

        in1 = Position_absolute_read(1)
        in2 = Position_absolute_read(2)
        in3 = Position_absolute_read(3)

        # print("Position are read")

        feedback = [in1, in2, in3]

        system_input = implement_PID(last_position, feedback)

        # print("PID is doing its job")
        
        # print("system input:", system_input)

        Target_speed_rpm(1, system_input[0])
        Target_speed_rpm(2, system_input[1])
        Target_speed_rpm(3, system_input[2])


        # print("here is FK:", Forward(Position_absolute_read(1), Position_absolute_read(2), Position_absolute_read(3)))
        # print("here is the speed", system_input)


def trajectory_4567(time_map,time_shift,start_time): #return value within 0 , 1
    last_time=datetime.datetime.now()
    dtime=(last_time-start_time).total_seconds()
    tau=(dtime-time_shift)/time_map
    s=-20*(tau**7)+70*(tau**6)-84*(tau**5)+35*(tau**4)
    return s

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

    


        # position_reads_homing.append( Position_absolute_read(1))
        # velocity_reads_1.append(Velocity_actual_rpm(1))
        # torque_read.append(Torque_actual(1))
        



    # Motion_z_endeffector(0)

    # kinematic_forward( position_feedback , velocity_feedback , 0)
    
    # Motion_z_endeffector(0)
    # offset_new = Offset(1)
    # plt.figure(1)
    # plt.plot(position_reads_homing)
    # plt.figure(2)
    # plt.plot(velocity_reads_1)
    # plt.figure(3)
    # plt.plot(torque_read)


def spiral_motion(center,radi,starting_height,ending_height,n,period):
    pitch=(ending_height-starting_height)/n
    e,x,y,z=Forward(Position_absolute_read(1),Position_absolute_read(1),Position_absolute_read(1))
    first_time=datetime.datetime.now()
    z_velocity=pitch/period
    dtime=0
    
    
    
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
    
    while not z==ending_height and dtime<n*period :
        current_time=datetime.datetime.now()
        dtime=(current_time-first_time).total_seconds()
        
        px=radi*np.cos(np.pi*2/period*dtime)+center[0] 
        py=radi*np.sin(np.pi*2/period*dtime)+center[1]
        pz=starting_height+dtime*z_velocity
        
        
        x_des.append(px)
        y_des.append(py)
        z_des.append(pz)
        
        desired_path = np.array([x_des, y_des, z_des])
        
        
        in1 = Position_absolute_read(1)        
        in2 = Position_absolute_read(2)        
        in3 = Position_absolute_read(3)


        position_feedback_1.append(in1)
        position_feedback_2.append(in2)
        position_feedback_3.append(in3)
        position_feedback = np.array([position_feedback_1,position_feedback_2,position_feedback_3])


        feedback=[in1,in2,in3]
        system_input=implement_PID([px,py,pz],feedback) 
        
        Target_speed_rpm(1,system_input[0])
        Target_speed_rpm(2,system_input[1])
        Target_speed_rpm(3,system_input[2])

        Controller1_history.append(system_input[0])
        Controller2_history.append(system_input[1])
        Controller3_history.append(system_input[2])
        Controller_history = np.array([Controller1_history, Controller2_history, Controller3_history])  
        
        torque_control = implement_PID_torque([px,py,pz],feedback) 

        
        torque_cmd_his_1.append(torque_control[0])
        torque_cmd_his_2.append(torque_control[1])
        torque_cmd_his_3.append(torque_control[2])
        torque_cmd_his = np.array([torque_cmd_his_1, torque_cmd_his_2, torque_cmd_his_3])
        


        velocity_feedback_1.append(Velocity_actual_rpm(1))
        velocity_feedback_2.append(Velocity_actual_rpm(2))
        velocity_feedback_3.append(Velocity_actual_rpm(3))
        velocity_feedback = np.array([velocity_feedback_1,velocity_feedback_2,velocity_feedback_3])        

        torque_feedback_1.append(Torque_actual(1))
        torque_feedback_2.append(Torque_actual(2))
        torque_feedback_3.append(Torque_actual(3))
        torque_feedback = np.array([torque_feedback_1,torque_feedback_2,torque_feedback_3])  
        


        e,x,y,z=Forward(in1,in2,in3)
        
        if z==ending_height or dtime==n*period:
            print("End Of Spiral!")
            Motion_z_endeffector(0)
            Motion_z_endeffector(0)
            break
    Motion_z_endeffector(0)
    Motion_z_endeffector(0)        
    return position_feedback, velocity_feedback, torque_feedback, desired_path, Controller_history, torque_cmd_his



# =================================================================================================
# -- FK -------------------------------------------------------------------------------------------
# =================================================================================================


def kinematic_forward( theta1_feedback , theta1_dot_feedback , theta1_ddot_feedback):
    
    psi = np.zeros((1,3))
    psi[0,0] = 0                ### angle of first limb base-fixed frame
    psi[0,1] = 2*(np.pi/3)      ### angle of second limb base-fixed frame
    psi[0,2] = -2*(np.pi/3)     ### angle of third limb base-fixed frame
    
    
    phi_forw = np.zeros((3,1))
    theta2_forw = np.zeros((3,1))
    phi_dot_forw = np.zeros((3,1))
    theta2_dot_forw = np.zeros((3,1))


        
    ## Rotation Matrices
    # rot_02g = np.zeros((3,3))
    # Rot_0_to_global_all = np.zeros((3,9)) ### rotation matrices for every limb
    Rot_1_to_0 = np.zeros((3,9))
    Rot_phi_to_1 = np.zeros((3,9))
    Rot_2_to_phi = np.zeros((3,9))
    Rot_2_to_0 = np.zeros((3,9))

    L1_vector_forw = np.zeros((3,1)) ### in the arm coordinate system
    L2_vector_forw = np.zeros((3,1))

    # S2 = np.zeros((3,3))
    # s_actuator = [0 , 0 , 0 , 0 , -1 , 0]  ### 6x1

    cross_L2L1 = np.zeros((3,3))
    cross_S2S1 = np.zeros((3,3))

#    phi_ddot = np.zeros((3,1))
#    theta2_ddot =  np.zeros((3,1))
    J_theta1 = np.zeros((3,3))
    p_ddot_d_l2 = np.zeros((3,1))
    J_P_ddot = np.zeros((3,3))
    x_o = np.zeros((3,1))
    y_o = np.zeros((3,1))
    z_o = np.zeros((3,1))
    O = np.zeros((3,3))
    B = np.zeros((3,1))
    A = np.zeros((3,3))
    
    J_theta1_forw = np.zeros((3,3))
    
    Rot_0_to_global = np.array([ [ np.cos(psi[0,0]) , np.sin(psi[0,0]) , 0 , np.cos(psi[0,1]) , np.sin(psi[0,1]) , 0 , np.cos(psi[0,2]) , np.sin(psi[0,2]) , 0 ]
                               ,[-np.sin(psi[0,0]) , np.cos(psi[0,0]) , 0 ,-np.sin(psi[0,1]) , np.cos(psi[0,1]) , 0 ,-np.sin(psi[0,2]) , np.cos(psi[0,2]) , 0 ] 
                               ,[ 0 , 0 , 1 , 0 , 0 , 1 , 0 , 0 , 1] ]).reshape(3,9)
    
    for i in range(3):
        
        x_o[i] = (R-r) + L1*np.cos(theta1_feedback[i])
        y_o[i] = 0
        z_o[i] = L1*np.sin(theta1_feedback[i])
        
        rot_02g = Rot_0_to_global[:, 3*i:3*i+3]   ## 3x3
        O[:,i] = np.matmul(rot_02g , np.array([[x_o[i,0]] , [y_o[i,0]] , [z_o[i,0]]])).reshape(1,3)  ##3x3
    #---------------------------------------------------------------------------
    O1O3 = (O[:,2] - O[:,0]).reshape(3,1) ## 3x1
    O1O2 = (O[:,1] - O[:,0]).reshape(3,1) ## 3x1
    
    d = np.cross (O1O2.reshape(1,3),O1O3.reshape(1,3)).reshape(3,1) ## 3x1
    d = d/np.linalg.norm(d) 
    
    cos_alpha = np.power(np.linalg.norm(O1O3),2) / (2*L2*np.linalg.norm(O1O3)) ;
    cos_beta = np.power(np.linalg.norm(O1O2),2) / (2*L2*np.linalg.norm(O1O2))
    
    O1O13 = (cos_alpha*L2) * (O1O3/np.linalg.norm(O1O3))
    O1O12 = (cos_beta*L2) * (O1O2/np.linalg.norm(O1O2))
    
    O13M13 = np.sqrt(L2**2 - np.power(np.linalg.norm(O1O13),2)) * np.cross(d.T , (O1O13/np.linalg.norm(O1O13)).T ).reshape(3,1)
    O12M12 = np.sqrt(L2**2 - np.power(np.linalg.norm(O1O12),2)) * np.cross(d.T , (O1O12/np.linalg.norm(O1O12)).T ).reshape(3,1)
   
    M13N13 = (-2*O13M13).reshape(3,1)
    M12N12 = (-2*O12M12).reshape(3,1)
    
    M13 = O[:,0].reshape(3,1) + O1O13.reshape(3,1) + O13M13.reshape(3,1)
    M12 = O[:,0].reshape(3,1) + O1O12.reshape(3,1) + O12M12.reshape(3,1)
    
    k13 = (( -M13[1,0] + M12[1,0] ) * M12N12[0,0] + ( M13[0,0] - M12[0,0] ) * M12N12[1,0] ) / ( M13N13[1,0] * M12N12[0,0] - M12N12[1,0] * M13N13[0,0] )
    P13 = (M13.reshape(3,1) + k13 * M13N13.reshape(3,1)) ## 3x1
    h = np.sqrt(L2**2 - np.linalg.norm(P13 - O[:,0].reshape(3,1))**2)
    P_feedback = P13 - h*d ## 3x1
    P_feedback [1][0] = - P_feedback [1][0]

    for i in range(3):
         rot_02g = Rot_0_to_global[:,3*i:3*i+3]
         Q = np.matmul(rot_02g, P_feedback) + np.array([[r-R] , [0] , [0]])
         L = np.linalg.norm(Q)

         phi_forw[i,0] = np.arccos(Q[1,0]/L2)
         theta2_forw[i,0] = np.arccos( (L**2 - L2**2 - L1**2)/(2*L2*L1*np.sin(phi_forw[i,0])))
         
         L1_vector_forw = L1 * np.array( [ [np.cos(theta1_feedback[i])] , [0] , [np.sin(theta1_feedback[i])] ] )
         L2_vector_forw = L2 * np.array([ np.sin(phi_forw[i,0])*np.cos(theta1_feedback[i]+theta2_forw[i]) , np.cos(phi_forw[i]) , np.sin(theta1_feedback[i]+theta2_forw[i])] )
         
         s1_forw = np.array([0 , -1 , 0]).reshape(3,1)  ### Screw 1 : 3x1
         s2_forw = np.array([ -np.sin(theta1_feedback[i]+theta2_forw[i]) , [0] , np.cos(theta1_feedback[i]+theta2_forw[i])] )
        
         cross_l2l1_forw = np.cross(L2_vector_forw.reshape(1,3),L1_vector_forw.reshape(1,3))
         cross_s2s1_forw = np.cross(s2_forw.reshape(1,3),s1_forw.reshape(1,3))
        

         ### upper link jacobian for 1 arm = J_1i_theta
         J_1_forw = np.matmul(rot_02g.T , -L2_vector_forw/(np.dot(cross_l2l1_forw.reshape(3), s1_forw.reshape(3))))  ## 3x1
         ### upper link jacobian for 3 arms = J_theta1
         J_theta1_forw[i,:] = J_1_forw.reshape(3)  ##3x3
    
    P_dot_feedback = np.matmul(np.linalg.inv(J_theta1_forw) , theta1_dot_feedback)
        
        
#----------------------------------------------------------------------------------------------------------------------------
  #       ### this is the upper link jacobian made by identification
  #       # B[i,0] = -np.dot(cross_l2l1,s1)
  #       # A[i,:] = np.matmul(l2.T,rot_02g)       
  # J_angular2linear = np.matmul(np.linalg.inv(A) , np.array([[B[0,0],0,0],[0,B[1,0],0],[0,0,B[2,0]]]))
#----------------------------------------------------------------------------------------------------------------------------
  
    for i in range(3):
        
         rot_02g = Rot_0_to_global[:,3*i:3*i+3] 
         L1_vector_forw = L1 * np.array( [ [np.cos(theta1_feedback[i])] , [0] , [np.sin(theta1_feedback[i])] ] )     
         L2_vector_forw = L2 * np.array([ np.sin(phi_forw[i,0])*np.cos(theta1_feedback[i]+theta2_forw[i]) , np.cos(phi_forw[i]) , np.sin(theta1_feedback[i]+theta2_forw[i])] )
         
         s1_forw = np.array([0 , -1 , 0]).reshape(3,1)  ### Screw 1 : 3x1
         s2_forw = np.array([ -np.sin(theta1_feedback[i]+theta2_forw[i]) , [0] , np.cos(theta1_feedback[i]+theta2_forw[i])] )
         
         cross_l2l1_forw = np.cross(L2_vector_forw.reshape(1,3),L1_vector_forw.reshape(1,3))
         cross_s2s1_forw = np.cross(s2_forw.reshape(1,3),s1_forw.reshape(1,3))
  #       l1 = l1_vector[:,i]
  #       l2 = l2_vector[:,i]
  #       s2 = S2[:,i]
  #       cross_l2l1 = cross_L2L1[:,i]
  #       cross_s2s1 = cross_S2S1[:,i]
         p_dot =np.matmul(rot_02g, P_dot_feedback)
         # phi_dot_forw(m,k) = -(s1_forw.' * p_dot)/(dot(cross_s2s1_forw,L2_vector_forw));
                               
         phi_dot_forw[i,0] = -np.matmul(s1_forw.T,p_dot)/np.dot(cross_s2s1_forw.reshape(3),L2_vector_forw.reshape(3))
         theta2_dot_forw[i,0] = -((np.matmul(L1_vector_forw.T,p_dot) + np.dot(cross_l2l1_forw.reshape(3),s2_forw.reshape(3))/np.dot(cross_s2s1_forw.reshape(3),L2_vector_forw.reshape(3)) * np.matmul(s1_forw.T,p_dot)) / -np.dot(cross_l2l1_forw.reshape(3),s1_forw.reshape(3)))
         
        
         
         # p_ddot_d_l2[i,0] = - np.dot(cross_l2l1_forw.reshape(3),s1_forw.reshape(3))*theta1_ddot_feedback[i] - (np.dot((L1_vector_forw*theta1_dot_feedback[i]**2).reshape(3),L2_vector_forw.reshape(3)) + 2*theta2_dot_forw[i,0]*phi_dot_forw[i,0]*np.dot(s1_forw.reshape(3),s2_forw.reshape(3))*np.cos(phi_forw[i])*L2**2 + (np.sin(phi_forw[i])**2 * theta2_dot_forw[i]**2 + phi_dot_forw[i]**2)*L2**2 )
        
         # J_P_ddot[i,:] = np.matmul(L2_vector_forw.T , rot_02g).reshape(3)
     
    # P_ddot_feedback = np.matmul(np.linalg.inv(J_P_ddot) , p_ddot_d_l2)
    
    
    return P_feedback, P_dot_feedback




#%%
    





# theta1_feedback = np.array([-76 , -77 , -78]).T
# theta1_dot_feedback = np.array([0 , 0 , 0]).T
# theta1_ddot_feedback = np.array([0 , -0 , 0]).T
# P_ee, P_dot_ee = kinematic_forward( theta1_feedback , theta1_dot_feedback , theta1_ddot_feedback)
# print(kinematic_forward( theta1_feedback , theta1_dot_feedback , theta1_ddot_feedback))
# test_forward = kinematic_forward( theta1_feedback , theta1_dot_feedback , theta1_ddot_feedback)


# # accel = np.array([np.zeros(157) , np.zeros(157) ,np.zeros(157) ])
# # P_feedback = np.array([np.zeros(157) , np.zeros(157) ,np.zeros(157) ])
# # P_dot_feedback = np.array([np.zeros(157) , np.zeros(157) ,np.zeros(157) ])
# a = []
# # a.append(kinematic_forward(np.array([position_feedback[:,i]]).T, np.array(velocity_feedback[:,i]).T, np.array(accel[:,i]).T))

# # for i in range(1,len(position_feedback)):
# #     # theta = 
# #     P_feedback = np.array([np.zeros(157) , np.zeros(157) ,np.zeros(157) ])
    
# #     P_feedback[:,i], P_dot_feedback[:,i] = kinematic_forward(np.array([position_feedback[:,i]]).T, np.array(velocity_feedback[:,i]).T, np.array(accel[:,i]).T)
    


# =================================================================================================
# -- IK -------------------------------------------------------------------------------------------
# =================================================================================================




# t1 = time.time()

def kinematic_inverse (desired_trajectory):
        
        x_des = desired_trajectory[0][0]
        y_des = desired_trajectory[0][1]
        z_des = desired_trajectory[0][2]
        
        xd_des = desired_trajectory[1][0]
        yd_des = desired_trajectory[1][1]
        zd_des = desired_trajectory[1][2]
        
        xdd_des = desired_trajectory[2][0]
        ydd_des = desired_trajectory[2][1]
        zdd_des = desired_trajectory[2][2]
        
        # print(x_des, y_des, zdd_des, ydd_des)
        psi = np.zeros((1,3))
        psi[0,0] = 0                ### angle of first limb base-fixed frame
        psi[0,1] = 2*(np.pi/3)      ### angle of second limb base-fixed frame
        psi[0,2] = -2*(np.pi/3)     ### angle of third limb base-fixed frame

        phi_inverse = np.zeros((3,1))
        theta1_inverse = np.zeros((3,1))
        theta2_inverse = np.zeros((3,1))
        phi_dot_inverse = np.zeros((3,1))
        theta1_dot_inverse = np.zeros((3,1))
        theta2_dot_inverse = np.zeros((3,1))
        phi_ddot_inverse = np.zeros((3,1))
        theta1_ddot_inverse = np.zeros((3,1))
        theta2_ddot_inverse = np.zeros((3,1))

## Rotation Matrices
        rot_02g = np.zeros((3,3))
        Rot_0_to_global_all = np.zeros((3,9)) ### rotation matrices for every limb
        Rot_1_to_0_all = np.zeros((3,9))
        Rot_phi_to_1_all = np.zeros((3,9))
        Rot_2_to_phi_all = np.zeros((3,9))
        Rot_2_to_0_all = np.zeros((3,9))
        
        L1_vector = np.zeros((3,3)) ### in the arm coordinate system
        L2_vector = np.zeros((3,3))

        s1 = np.array([0 , -1 , 0]).reshape(3,1)  ### Screw 1 : 3x1
        S2 = np.zeros((3,3))
        s_actuator = [0 , 0 , 0 , 0 , -1 , 0]  ### 6x1
        
        P_ee = np.zeros((3,3))
        V_ee = np.zeros((3,3))
        A_ee = np.zeros((3,3))
        q = np.zeros((3,3))
        
        J_theta1 = np.zeros((3,3))
        J_theta2 = np.zeros((3,3))
        J_phi = np.zeros((3,3))
        
        cross_L2L1 = np.zeros((3,3))
        cross_S2S1 = np.zeros((3,3))
        
        Rot_0_to_global = np.array([ [ np.cos(psi[0,0]) , np.sin(psi[0,0]) , 0 , np.cos(psi[0,1]) , np.sin(psi[0,1]) , 0 , np.cos(psi[0,2]) , np.sin(psi[0,2]) , 0 ]
                                ,[-np.sin(psi[0,0]) , np.cos(psi[0,0]) , 0 ,-np.sin(psi[0,1]) , np.cos(psi[0,1]) , 0 ,-np.sin(psi[0,2]) , np.cos(psi[0,2]) , 0 ] 
                                ,[ 0 , 0 , 1 , 0 , 0 , 1 , 0 , 0 , 1] ]).reshape(3,9)
        # print(Rot_0_to_global)
        for i in range(3):
            
            rot_02g = Rot_0_to_global[:, 3*i:3*i+3]   ## 3x3
            # print(np.array([x_des , y_des , z_des]))
            # print(np.matmul(rot_02g, np.array([x_des , y_des , z_des])))
            # print(rot_02g)
            P_ee[:,i] = np.matmul(rot_02g , np.array([ x_des , y_des , z_des ])).reshape(3)
            # print(P_ee[:,i])
            V_ee[:,i] = np.matmul(rot_02g , np.array([ xd_des , yd_des , zd_des ])).reshape(3)
            A_ee[:,i] = np.matmul(rot_02g , np.array([ xdd_des , ydd_des , zdd_des ])).reshape(3)
                                  
            
            q[:,i] = np.array(np.array([P_ee[:,i]]).reshape(3,1) + np.array([[r-R] , [0] , [0]])).reshape(3) # 3x1

            qx = q[0,i] # 1x1
            qy = q[1,i] # 1x1
            qz = q[2,i] # 1x1
            
#             # Equation 5- Paralellogram Angle
            phi_inverse[i,0] = np.arccos(qy/L2) # 1x1
            # print(phi_inverse)
            a = (qx+L1)**2 + qz**2 - (L2*np.sin(phi_inverse[i,0]))**2  # 1x1  
            b = -4*L1*qz  # b is defined as b prime in delta equation
            c=(qx-L1)**2 + qz**2 - (L2*np.sin(phi_inverse[i,0]))**2 # 1x1
            
            if b**2 < 4*a*c:
                ti = 0
                print("out of work space")
            else:
                ti =(-b-(np.sqrt(b**2-4*(a*c))))/(2*a)  # 1x1
            
#             # Equation 7- The planar angle between upper and lower link
            theta2_inverse[i,0] = np.arccos((qx**2 + qy**2 + qz**2 - L2**2 - L1**2)/(2*L1*L2*np.sin(phi_inverse[i,0])))  # 1x1
            # print(theta2_inverse)
#             # Equation 14- Actuated Link Angle (theta1)
            theta1_inverse[i,0] = 2*np.arctan(ti) # 1x1  
#             print(theta1_inverse)
# #             # in the arm coordinate system
            L1_vector = L1 * np.array( [ [np.cos(theta1_inverse[i])] , [0] , [np.sin(theta1_inverse[i])] ] )
            L2_vector = L2 * np.array([ np.sin(phi_inverse[i,0])*np.cos(theta1_inverse[i]+theta2_inverse[i]) , np.cos(phi_inverse[i]) , np.sin(theta1_inverse[i]+theta2_inverse[i])] )
         
            s1 = np.array([0 , -1 , 0]).reshape(3,1)  ## Screw 1 : 3x1
            s2 = np.array([ -np.sin(theta1_inverse[i]+theta2_inverse[i]) , [0] , np.cos(theta1_inverse[i]+theta2_inverse[i])] )
            
            cross_l2l1 = np.cross(L2_vector.reshape(1,3), L1_vector.reshape(1,3))
            cross_s2s1 = np.cross(s2.reshape(1,3), s1.reshape(1,3))
            
            # J_theta1_i  = (-(1*L2_vector.T)/(np.dot(cross_l2l1,s1)))
            
            J_theta1[i,:] = (-(1*L2_vector.T)/(np.dot(cross_l2l1,s1)))  ## 1x3
#             J_theta2[i,:] = -1*(L1_vector.T + ((np.dot(cross_l2l1,s2))/(np.dot(cross_s2s1,L2_vector)))* s1.T)/(-np.dot(cross_l2l1,s1))  ## 1x3
#             J_phi[i,:] = -(1*s1.T)/(np.dot(cross_s2s1,L2_vector))  ## 1x3
            
            # J_x = np.linalg.inv(J_theta1)
#             phi_dot_inverse[i] = -np.matmul(s1.T, V_ee[:,i])/np.dot(cross_s2s1.reshape(3), L2_vector.reshape(3))
#             theta2_dot_inverse[i] = -((np.matmul(L1_vector.T, V_ee[:,i]) + np.dot(cross_l2l1.reshape(3), s2.reshape(3))/np.dot(cross_s2s1.reshape(3), L2_vector.reshape(3)) * np.matmul(s1.T, V_ee[:,i])) / -np.dot(cross_l2l1.reshape(3), s1.reshape(3)))
#             theta1_dot_inverse[i] = -np.matmul(L2_vector.T , V_ee[:,i])/np.dot(cross_l2l1,s1)  ## 1x1
   
#             # phi_ddot_inverse[i] = ( np.dot(s1 , A_ee[:,i] + L2_vector*(0*np.cos(phi_inverse[i])*(theta2_dot_inverse[i])**2 + phi_dot_inverse[i]**2) + (L1_vector * (theta1_dot_inverse[i]**2) + 0 ))) / (np.dot(L2_vector,-cross_s2s1))   ## 1x1
# #             # theta1_ddot_inverse[i] = -( np.dot((A_ee[:,i] + L1_vector*(theta1_dot_inverse[i]**2)), L2_vector) + (2*theta2_dot_inverse[i]*phi_dot_inverse[i]*np.dot(s1,s2)*np.cos(phi_inverse[i])*L2**2) + (((np.sin(phi_inverse[i]))**2*theta2_dot_inverse[i]**2 + phi_dot_inverse[i]**2)*L2**2 )) / (np.dot(cross_l2l1,s1))  ## 1x1
# #             # theta2_ddot_inverse[i] = ( np.dot( L1_vector ,  A_ee[:,i] + L2_vector*(phi_dot_inverse[i]**2) - (2*theta2_dot_inverse[i]*phi_dot_inverse[i] * np.cross(s1,np.cross(s2,L2_vector)))) + L2*np.sin(phi_inverse[i])*L1*np.cos(theta2_inverse[i])*(theta2_dot_inverse[i]**2) + (L1*theta1_dot_inverse[i])**2 + phi_ddot_inverse[i]*np.dot(s2,-cross_l2l1) ) / (np.dot(s1,cross_l2l1))  ## 1x1
            
        return theta1_inverse, J_theta1



# =================================================================================================
# -- tests ----------------------------------------------------------------------------------------
# =================================================================================================


def circle_test():

    #%%
    # Operation_mode(1,4)
    # Operation_mode(2,4)
    # Operation_mode(3,4)

    # Operation_mode(1,-3)
    # Operation_mode(2,-3)
    # Operation_mode(3,-3)


    position_feedback, velocity_feedback, torque_feedback, desired_path, Controller_history, torque_cmd_his = circle_motion([0,0],15,-65,2,15) #arguments: center,minor_axis,major_axis,height,n,period
    # Target_torque(1, 0)
    # Target_torque(2, 0)
    # Target_torque(3, 0)

    # position_feedback, velocity_feedback, torque_feedback, desired_path, Controller_history, torque_cmd_his = spiral_motion([0,0],20,-64,-64,3,3) #arguments: center,minor_axis,major_axis,height,n,period
     # arguments: center,radi,starting_height,ending_height,n,period
    Motion_z_endeffector(0)
    Motion_z_endeffector(0)
    # e,x,y,z = Forward(position_feedback[0],position_feedback[1],position_feedback[2])

    x_feedback = []
    y_feedback = []
    z_feedback = []
    size = len(position_feedback[0])

    for i in range(size):

        # x_ee_des = x_des[i,0]
        # y_ee_des = y_des[i,0]
        # z_ee_des = z_des[i,0]
        # kinema = kinematic_inverse(xi,yi,zi,0,0,0,0,0,0)
        kinema_new = Forward( position_feedback[0,i],position_feedback[1,i],position_feedback[2,i] )
        # print(kinema[:,0])
        x_feedback.append(kinema_new[1])
        y_feedback.append(kinema_new[2])
        z_feedback.append(kinema_new[3])

    x_feedback = np.asarray(x_feedback).reshape(size,1)
    y_feedback = np.asarray(y_feedback).reshape(size,1)
    z_feedback = np.asarray(z_feedback).reshape(size,1)

    # plt.plot(t1, T, label = 'Input Torque')
    #             plt.plot(t1, torque_measured, label = 'Measured Torque')
               
    #             plt.grid()
    #             plt.show()



    #%%

    plt.figure(1)
    plt.title("desired and actual path")
    thetaact1 = plt.subplot(3,1,1)
    plt.plot(x_feedback)  
    plt.plot(desired_path[0])

    thetaact2 = plt.subplot(3,1,2)
    plt.plot(y_feedback)
    plt.plot(desired_path[1])
      
    thetaact3 = plt.subplot(3,1,3)
    plt.plot(-z_feedback)  
    plt.plot(-desired_path[2])

    plt.figure(2)
    plt.title("desired and actual path")
    plt.plot(x_feedback,y_feedback)
    plt.plot(desired_path[0], desired_path[1])
    plt.legend()
    plt.xlabel('time')
    plt.ylabel('torque')


    plt.figure(3)
    plt.title("desired and actual path")
    thetaact1 = plt.subplot(3,1,1)
    plt.plot(torque_feedback[0], label = 'Torque Exerted')  
    plt.plot(torque_cmd_his[0])

    thetaact2 = plt.subplot(3,1,2)
    plt.plot(torque_feedback[1])  
    plt.plot(torque_cmd_his[1])
      
    thetaact3 = plt.subplot(3,1,3)
    plt.plot(torque_feedback[2])  
    plt.plot(torque_cmd_his[2])

    plt.title("torque feedback")


    plt.figure(4)
    thetaact1 = plt.subplot(3,1,1)
    plt.plot(velocity_feedback[0])  
    plt.plot(Controller_history[0])  

    thetaact2 = plt.subplot(3,1,2)
    plt.plot(velocity_feedback[1])
    plt.plot(Controller_history[1])  

    thetaact3 = plt.subplot(3,1,3)
    plt.plot(velocity_feedback[2])
    plt.plot(Controller_history[2])  
    plt.title("velocity feedback and commanded")


    plt.figure(5)
    thetaact1 = plt.subplot(3,1,1)
    plt.plot(torque_cmd_his[0])

    thetaact2 = plt.subplot(3,1,2)
    plt.plot(torque_cmd_his[1])
      
    thetaact3 = plt.subplot(3,1,3)
    plt.plot(torque_cmd_his[2])
    plt.title("commaded torque")


    #%%
    size = len(position_feedback[0])
    error_x = []
    error_y = []
    error_z = []
    for i in range(size): 
        error_x.append(x_feedback[i]-desired_path[0,i])
        error_y.append(y_feedback[i]-desired_path[1,i])
        error_z.append(z_feedback[i]-desired_path[2,i])

    error_x = np.asarray(error_x).reshape(size,1)
    error_y = np.asarray(error_y).reshape(size,1)
    error_z = np.asarray(error_z).reshape(size,1)

    plt.figure()
    errorx = plt.subplot(3,1,1)
    plt.plot(error_x)

    errory = plt.subplot(3,1,2)
    plt.plot(error_y)

    errory = plt.subplot(3,1,3)
    plt.plot(error_z)



    #%%
        
    # position_feedback, velocity_feedback, torque_feedback = Homing()

    plt.figure(1)  
    theta_act_1 = plt.subplot(311)
    plt.plot(position_feedback[0])
    theta_act_2 = plt.subplot(312)
    plt.plot(position_feedback[1])
    theta_act_3 = plt.subplot(313)
    plt.plot(position_feedback[2])

    plt.figure(2)
    theta__dot_act_1 = plt.subplot(311)
    plt.plot(velocity_feedback[0])
    theta__dot_act_2 = plt.subplot(312)
    plt.plot(velocity_feedback[1])
    theta__dot_act_3 = plt.subplot(313)
    plt.plot(velocity_feedback[2])

    plt.figure(3)
    torque_act_1 = plt.subplot(311)
    plt.plot(torque_feedback[0])
    torque_act_2 = plt.subplot(312)
    plt.plot(torque_feedback[1])
    torque_act_3 = plt.subplot(313)
    plt.plot(torque_feedback[2])


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
