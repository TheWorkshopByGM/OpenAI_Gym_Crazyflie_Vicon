import math
import sys
import os
import time
import logging
import struct
import socket
import select
import numpy as np
from threading import Thread
from threading import Event

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander
from cflib.utils import uri_helper

#Vicon
#Vicon real positions
VICON_POS = {'X':0,'Y':0,'Z':0,'Pitch':0,'Roll':0,'Yaw':0}

#Vicon take_off pos
VICON_INITIALIZED = 0
VICON_POS_INIT = {'X':0,'Y':0,'Z':0,'Pitch':0,'Roll':0,'Yaw':0}

#Vicon relative pos
VICON_POS_REL = {'X':0,'Y':0,'Z':0,'Pitch':0,'Roll':0,'Yaw':0}

class ViconUDPDataRelay(Thread):
    def __init__(self, RX_sock):
        #
        Thread.__init__(self)
        self.DEBUG = False
        # Define Message length
        self.MsgLen = 1024
        self.RX_sock = RX_sock
        self.MessageRX_flag = False
        self.SrvMsgReceived = True
        self.RxMessage = None
        # Entry dictionary
        self.object_dict = {}
        self.reset_object_dict()
        
    def reset_object_dict(self):
        self.object_dict['number_objects'] = 0

    def ReceiveMsgOverUDP(self):
        Header = 'Waiting!'
        # Select RX_sock
        sock = self.RX_sock
        # Verify if data has been received from VICON
        ready = select.select([sock], [], [], 0.001)
        # If data has been received, process it
        if ready[0]:
            self.MessageRX_flag = True
            data, addr = sock.recvfrom(self.MsgLen)
            # Extract message header
            Sequence_B00 = data[0]
            Sequence_B01 = data[1]
            Sequence_B02 = data[2]
            Sequence_B03 = data[3]
            Sequence = (Sequence_B03<<32)+(Sequence_B02<<16)+(Sequence_B01<<8)+Sequence_B00
            self.ProcessViconData(data[0:160])
            
    def ProcessViconData(self, data):
        # Create struct with message format:
        # Data types according to https://docs.python.org/3/library/struct.html
        # FrameNumber                       -> B000 - B003 (uint32,  I)
        # ItemsInBlock                      -> B004        (uint8,   B)
        #              -----------------------------------------------
        #                      ItemID       -> B005        (uint8,   B)
        #              Header  ItemDataSize -> B006 - B007 (uint16,  H)
        #                      ItemName     -> B008 - B031 (uint8,   B)
        #              -----------------------------------------------
        # Item_raw_00          TransX       -> B032 - B039 (double,  d)
        #                      TransY       -> B040 - B047 (double,  d)
        #                      TransZ       -> B048 - B055 (double,  d)
        #              Data    RotX         -> B056 - B063 (double,  d)
        #                      RotY         -> B064 - B071 (double,  d)
        #                      RotZ         -> B072 - B079 (double,  d)    
        #              -----------------------------------------------
        #                      ItemID       -> B080        (uint8,   B)
        #              Header  ItemDataSize -> B081 - B082 (uint16,  H)
        #                      ItemName     -> B083 - B106 (uint8,   B)
        #              -----------------------------------------------
        # Item_raw_01          TransX       -> B107 - B114 (double,  d)
        #                      TransY       -> B115 - B122 (double,  d)
        #                      TransZ       -> B123 - B130 (double,  d)
        #              Data    RotX         -> B131 - B139 (double,  d)
        #                      RotY         -> B140 - B146 (double,  d)
        #                      RotZ         -> B147 - B154 (double,  d)    
        #              -----------------------------------------------
        s = struct.Struct('I2BH24c6dBH24c6d')
        UnpackedData = s.unpack(data)
        FrameNumber  = UnpackedData[0]
        ItemsInBlock = UnpackedData[1]
        Item_raw_00_ItemID       = UnpackedData[2]
        Item_raw_00_ItemDataSize = UnpackedData[3]
        Item_raw_00_ItemName     = UnpackedData[4:28]
        Item_raw_00_TransX       = UnpackedData[28]
        Item_raw_00_TransY       = UnpackedData[29]
        Item_raw_00_TransZ       = UnpackedData[30]
        Item_raw_00_RotX         = UnpackedData[31]
        Item_raw_00_RotY         = UnpackedData[32]
        Item_raw_00_RotZ         = UnpackedData[33]
        Item_raw_00_ItemDataSize_string = []
        for this_byte in range(0,len(Item_raw_00_ItemName)):
            if Item_raw_00_ItemName[this_byte]>= b'!' and Item_raw_00_ItemName[this_byte]<= b'~':
                Item_raw_00_ItemDataSize_string.append(Item_raw_00_ItemName[this_byte].decode('utf-8'))
        Item_raw_00_ItemDataSize_string = ''.join(Item_raw_00_ItemDataSize_string)
        self.object_dict[Item_raw_00_ItemDataSize_string] = {'PosX':Item_raw_00_TransX,'PosY':Item_raw_00_TransY,
                                                             'PosZ':Item_raw_00_TransZ,'RotX':Item_raw_00_RotX,
                                                             'RotY':Item_raw_00_RotY,'RotZ':Item_raw_00_RotZ}
        self.object_dict['number_objects'] += 1
        
        #Store vicon readings in global variables
        #Vicon real positions
        global VICON_POS, VICON_POS_INIT, VICON_POS_REL, VICON_INITIALIZED

        # Storing values in VICON_POS (vicon position)
        VICON_POS['X'] = self.object_dict[Item_raw_00_ItemDataSize_string]['PosX']*1e-1
        VICON_POS['Y']  = self.object_dict[Item_raw_00_ItemDataSize_string]['PosY']*1e-1
        VICON_POS['Z']  = self.object_dict[Item_raw_00_ItemDataSize_string]['PosZ']*1e-1
        VICON_POS['Pitch']  = np.rad2deg(self.object_dict[Item_raw_00_ItemDataSize_string]['RotY'])
        VICON_POS['Roll']  = np.rad2deg(self.object_dict[Item_raw_00_ItemDataSize_string]['RotX'])
        VICON_POS['Yaw']  = np.rad2deg(self.object_dict[Item_raw_00_ItemDataSize_string]['RotZ'])
        
        #If not initialised, initialise VICON_POS_INIT (initial vicon position, take off point)
        if VICON_INITIALIZED == 0:
            VICON_INITIALIZED = 1
            VICON_POS_INIT['X'] = VICON_POS['X']
            VICON_POS_INIT['Y'] = VICON_POS['Y']
            VICON_POS_INIT['Z'] = VICON_POS['Z']
            VICON_POS_INIT['Pitch'] = VICON_POS['Pitch']
            VICON_POS_INIT['Roll'] = VICON_POS['Roll']
            VICON_POS_INIT['Yaw'] = VICON_POS['Yaw']

        #Find relative position of drone compared to home point
        VICON_POS_REL['X'] = VICON_POS['X'] - VICON_POS_INIT['X']
        VICON_POS_REL['Y'] = VICON_POS['Y'] - VICON_POS_INIT['Y']
        VICON_POS_REL['Z'] = VICON_POS['Z'] - VICON_POS_INIT['Z']
        VICON_POS_REL['Ptch'] = VICON_POS['Pitch'] - VICON_POS_INIT['Pitch']
        VICON_POS_REL['Roll'] = VICON_POS['Roll'] - VICON_POS_INIT['Roll']
        VICON_POS_REL['Yaw'] = VICON_POS['Yaw'] - VICON_POS_INIT['Yaw']

        if self.DEBUG:
            print('\tPosition [cm]: {0:+3.4f}, {1:+3.4f}, {2:+3.4f}'.format(VICON_POS_REL['X'],VICON_POS_REL['Y'],VICON_POS_REL['Z']))
            print('\tAttitude [deg]: {0:+3.4f}, {1:+3.4f}, {2:+3.4f}'.format(VICON_POS_REL['Roll'],VICON_POS_REL['Pitch'],VICON_POS_REL['Yaw']))
            print('----------------------------------')
        
    def close(self):
        pass

#Crazyflie
URI = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E7E7')

class Crazyflie:
    def __init__(self):
        self.m1 = 0
        self.m2 = 0
        self.m3 = 0
        self.m4 = 0
        self.x = 0
        self.y = 0
        self.z = 0
        self.pitch = 0
        self.roll = 0
        self.yaw = 0
        IP_Address01 = "0.0.0.0"
        Host01   = (IP_Address01, 51001)
        # Create RX socket
        self.RX_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.RX_sock.bind(Host01)
        self.MyViconDataRelay = ViconUDPDataRelay(self.RX_sock)
        self.MyViconDataRelay.DEBUG = True
        
        cflib.crtp.init_drivers()

        with SyncCrazyflie(URI, cf=Crazyflie(rw_cache='./cache')) as scf:
            scf.cf.param.set_value('motorPowerSet.enable',1)           
            scf.cf.param.set_value('motorPowerSet.m1', self.m1)
            scf.cf.param.set_value('motorPowerSet.m2', self.m2)
            scf.cf.param.set_value('motorPowerSet.m3', self.m3)
            scf.cf.param.set_value('motorPowerSet.m4', self.m4)

    #Function to update motor speed based on class motor variables (m1,m2,m3,m4)
    def motor_update(self):
        with SyncCrazyflie(URI, cf=Crazyflie(rw_cache='./cache')) as scf:
            scf.cf.param.set_value('motorPowerSet.enable',1)           
            scf.cf.param.set_value('motorPowerSet.m1', self.m1)
            scf.cf.param.set_value('motorPowerSet.m2', self.m2)
            scf.cf.param.set_value('motorPowerSet.m3', self.m3)
            scf.cf.param.set_value('motorPowerSet.m4', self.m4)
    
    #Function to update crazyflie position and attitude variables based on Vicon readings
    def vicon_read(self): 
        try:
            global VICON_POS_REL
            self.MyViconDataRelay.ReceiveMsgOverUDP()
            self.x = VICON_POS_REL['X']
            self.y = VICON_POS_REL['Y']
            self.z = VICON_POS_REL['Z']
            self.pitch = VICON_POS_REL['Pitch']
            self.roll = VICON_POS_REL['Roll']
            self.yaw = VICON_POS_REL['Yaw']
            
        except(KeyboardInterrupt,SystemExit):
            print("\nClosing program ...")
            self.RX_sock.close() # Close socket
            sys.exit() # Exit program
        
        except socket.error as msg:
            print("Socket error!")
            self.RX_sock.close() # Close socket
            print(msg)
            sys.exit()

       

class Crazyflie_Sim:
    def __init__(self):
        Crazyflie.init()

    def action(self, action):
        #Write function to select which action to take
        if action == 0:
            self.Crazyflie.m1 += 2
        if action == 1:
            self.Crazyflie.m1 -= 2
        if action == 2:
            self.Crazyflie.m2 += 2
        if action == 3:
            self.Crazyflie.m2 -= 2
        if action == 4:
            self.Crazyflie.m3 += 2
        if action == 5:
            self.Crazyflie.m3 -= 2
        if action == 6:
            self.Crazyflie.m4 += 2
        if action == 7:
            self.Crazyflie.m4 -= 2

        self.Crazyflie.motor_update()
        self.Crazyflie.vicon_read()

    def evaluate(self):
        #Write function to compute reward for AI
        reward = 0
        return reward

    def is_done(self):
        #Write function to determine if simulation is done (ie: failed or reached desired state)
        if 1 == 1:
            return True
        return False

    def observe(self):
        # MODIFY function to return discrete state
        self.Crazyflie.vicon_read()
        attitude = {'x':self.Crazyflie.x,
                    'y':self.Crazyflie.y,
                    'z':self.Crazyflie.z,
                    'pitch':self.Crazyflie.pitch,
                    'roll':self.Crazyflie.roll,
                    'yaw':self.Crazyflie.yaw}
        return attitude

    def view(self):
        # write function to visualise simultion
        pass
