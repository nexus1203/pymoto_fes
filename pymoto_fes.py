__author__ = "Nexus1203"

# Echo client program
import array
import logging
import socket
import time
import struct
import numpy as np
from collections import OrderedDict
from time import perf_counter as pf
from .udpPacket import UdpPacket, UdpPacket_Req, UdpPacket_Ans, axisTorque, axisPosition, UdpPacket_StrAns


#sws
def testBit(int_types, offset):
    mask = 1 << offset
    if (int_types & mask) == mask:
        return True
    else:
        return False


def splitString(s, chunksize=479):
    return [s[i:i + chunksize] for i in range(0, len(s), chunksize)]


class FastEthServer:

    def __init__(self, ip="192.168.99.100"):
        """
        constructor method for DxFastEthServer class
        :param ip:  ip number of dx controller-server
        :return:    None
        """

        #create socket
        self.s = socket.socket(
            socket.AF_INET,  #Internet socket types
            socket.SOCK_DGRAM)  #UDP socket !!!
        self.s.settimeout(2)  #timeout 1 sec

        #set connection data
        self.UDP_IP = ip  #default IP ("192.168.99.100")
        self.UDP_PORT = 10040  #port (fixed to 10040)

        self.status = OrderedDict()
        self.error = None

    def setHostIp(self, ip):
        self.UDP_IP = ip

    def sendCmd(self, reqSubHeader, reqData, procDiv=1, as_string=False):
        """
        Send Command (request packet) to Dx server ang get response (answer packet)
        :param reqSubHeader:    request sub header part of packet ( depend on each command )
        :param reqdata:         data part of the packet ( optional, depend of the command )
        :param procDiv :        Processing division (1-robot control, 2-file control)
        :return: ansPacket      answer packet
        """

        req_packet = UdpPacket_Req(reqSubHeader, reqData,
                                   procDiv)  #prepare packet
        req_packet.reqID = 0
        req_str = str(req_packet)  #string representation of the packet

        ans_str = self.socketSndRcv(req_str)
        # print("reqData, :",reqData)

        if ans_str == None:
            return None

        # a = array.array('B', req_str)
        # b = array.array('B', ans_str)
        if as_string:
            ansPacket = UdpPacket_StrAns(ans_str, procDiv)
        else:
            ansPacket = UdpPacket_Ans(
                ans_str, procDiv)  #create answer packet from answer string
        return ansPacket

    def socketSndRcv(self, req_str):

        try:
            #send request packet (string) to dx server, get response (string) from dx server
            #send request
            # print("send to", six.b(req_str))
            # print("send in encode", req_str.encode('latin-1'))

            self.s.sendto(
                req_str.encode('latin-1'),  #UDP packet
                ((self.UDP_IP), self.UDP_PORT)
            )  #A pair (host, port) is used for the AF_INET address family
            #get answer from the server
            while True:
                (ans_str, address) = self.s.recvfrom(512)
                ans_str = ans_str.decode('unicode-escape')  #
                # ans_str = ans_str.decode('latin-1') #
                # print("raw", ans_str)
                if address: break

        except socket.timeout as e:
            print('socket timeout: ' + str(e))
            logging.exception(str(e))
            return None

        except socket.gaierror as e:
            print('socket address error')
            logging.exception(str(e))
            return None

        except socket.error as e:
            print('socket related error')
            logging.exception(str(e))
            return None

        else:
            return ans_str

    #---------- Dx server functions --------------------
    #Status Information Reading Command
    def getStatusInfo(self):
        """
        """
        reqSubHeader = {
            'cmdNo': (0x72, 0x00),  #Command No. 0x72
            'inst': (1, 0),  #Fixed to 1
            'attr': 0,  #1: Data 1, 2: Data 2
            'service':
            0x01,  #Get_Attribute_Single: 0x0E, Get_Attribute_All: 0x01
            'padding': (0, 0)
        }

        reqData = []

        ansPacket = self.sendCmd(reqSubHeader, reqData)

        if (ansPacket == None or ansPacket.status != 0):
            return False

        #parse answer data
        byte1 = ansPacket.data[0]
        byte2 = ansPacket.data[4]

        self.status['Step'] = testBit(byte1, 0)
        self.status['Cycle'] = testBit(byte1, 1)
        self.status['Auto'] = testBit(byte1, 2)
        self.status['Running'] = testBit(byte1, 3)
        self.status['InGuard'] = testBit(byte1, 4)
        self.status['Teach'] = testBit(byte1, 5)
        self.status['Play'] = testBit(byte1, 6)
        self.status['Remote'] = testBit(byte1, 7)

        self.status['Hold_by_Pendant'] = testBit(byte2, 1)
        self.status['Hold_External'] = testBit(byte2, 2)
        self.status['Hold_by_cmd'] = testBit(byte2, 3)
        self.status['Alarm'] = testBit(byte2, 4)
        self.status['Error'] = testBit(byte2, 5)
        self.status['ServoOn'] = testBit(byte2, 6)

        return True

    #Hold/Servo control
    def holdServoOnOff(self, a1, a2):
        """
        Sub header part:
        Command No. 0x83
        Instance Specify one out of followings  Specify the types of OFF/ON command
            1: HOLD
            2: Servo ON
            3: HLOCK
        Attribute Fixed to “1”. Specify “1”.
        Service • Set_Attribute_Single: 0x10 Specify the accessing method to the data.
                    0x10 : Execute the specified request

        Data part:
        32bit integer Byte 0 Byte 1 Byte 2 Byte3 <Details>
                1 1:ON
                2:OFF

        """
        reqSubHeader = {
            'cmdNo': (0x83, 0x00),  #Command No.
            'inst': [a1, 0],  #instance 1: HOLD, 2: Servo ON, 3: HLOCK
            'attr': 1,  #Fixed to “1”
            'service':
            0x10,  #Get_Attribute_Single: 0x0E, Get_Attribute_All: 0x01
            'padding': (0, 0)
        }

        reqData = [a2, 0, 0, 0]

        ansPacket = self.sendCmd(reqSubHeader, reqData)

        return (ansPacket != None and ansPacket.status == 0)

    def putServoOn(self):
        return self.holdServoOnOff(2, 1)

    def putServoOff(self):
        return self.holdServoOnOff(2, 2)

    def putHoldOn(self):
        return self.holdServoOnOff(1, 1)

    def putHoldOff(self):
        return self.holdServoOnOff(1, 2)

    # select job to run
    def selectJob(self, jobName, line_num=0):
        """
        Select Job to run in the controller
        Command No. 0x87
        instance 1: select job
        Attribute Fixed to “1”. Specify “1”. 

        Args:
            jobName (_type_): Name of the job to run
            line_num (int, optional): Line from which job will start excecuting. Defaults to 0.

        Returns:
            bool: True if success
        """
        reqSubHeader = {
            'cmdNo': (0x87, 0x00),
            'inst': [1, 0],
            'attr': 0,
            'service': 0x02,
            'padding': (0, 0)
        }
        reqData = jobName.encode('utf-8')
        if len(reqData) > 32:
            raise ValueError("Job name too long")
        reqData += bytearray(32 - len(reqData))
        reqData += struct.pack('<I', line_num)
        ansPacket = self.sendCmd(reqSubHeader, reqData)
        return (ansPacket != None and ansPacket.status == 0)

    #Start-up (Job Start) Command
    def startJob(self):
        """
        Command No. 0x86
        Instance Fixed to “1”. Specify “1”.
        Attribute Fixed to “1”. Specify “1”.
        Service • Set_Attribute_Single: 0x10 Specify the accessing method to the data.
                0x10 : Execute the specified request

        Data part:
        32bit integer Byte 0 Byte 1 Byte 2 Byte3 <Details>
        """
        reqSubHeader = {
            'cmdNo': (0x86, 0x00),
            'inst': [1, 0],
            'attr': 1,
            'service': 0x10,
            'padding': (0, 0)
        }

        reqData = [1, 0, 0, 0]

        ansPacket = self.sendCmd(reqSubHeader, reqData)

        return (ansPacket != None and ansPacket.status == 0)

    def setCycleType(self, cycleType):
        """
        Command No. 0x84
        Instance type of cycle
        CYCLE_TYPE_STEP = 1
        CYCLE_TYPE_ONE_CYCLE = 2
        CYCLE_TYPE_CONTINUOUS = 3
        Attribute Fixed to “1”. Specify “1”.
        Service • Set_Attribute_Single: 0x10 Specify the accessing method to the data.
                0x10 : Execute the specified request
        
        Data part:
        32bit integer Byte 0 Byte 1 Byte 2 Byte3 <Details>
    
        Args:
            cycleType (int): type of cycle, CYCLE_TYPE_STEP = 1, CYCLE_TYPE_ONE_CYCLE = 2, CYCLE_TYPE_CONTINUOUS = 3
            
        Returns:
            bool: True if success
        """
        reqSubHeader = {
            'cmdNo': (0x84, 0x00),
            'inst': [2, 0],
            'attr': 1,
            'service': 0x10,
            'padding': (0, 0)
        }

        reqData = [cycleType, 0, 0, 0]

        ansPacket = self.sendCmd(reqSubHeader, reqData)

        return (ansPacket != None and ansPacket.status == 0)

    def checkErrorCode(self):
        """
        Command No. 0x70
        instance 1: latest error code
        Attribute Fixed to “1”. Specify “1”. for alarm code
        Service • Get_Attribute_Single: 0x0E Specify the accessing method to the data.
                0x0E : Get the specified data
        
        Returns:
            bool: True if success
        """
        reqSubHeader = {
            'cmdNo': (0x70, 0x00),
            'inst': [1, 0],
            'attr': 1,
            'service': 0x0E,
            'padding': (0, 0)
        }

        reqData = []

        ansPacket = self.sendCmd(reqSubHeader, reqData)

        if (ansPacket == None or ansPacket.status != 0):
            return False
        else:
            self.error = struct.unpack('<I', ansPacket.data[0:4])[0]
            return True

    # Alarm Reset
    def resetAlarm(self):
        """
        Command No. 0x82
        Instance "1” for alarm, "2" for Error. Specify “1”. for the alarm reset
        Attribute Fixed to “1”. Specify “1”.
        Service • Set_Attribute_Single: 0x10 Specify the accessing method to the data.
                0x10 : Execute the specified request

        Data part:
        32bit integer Byte 0 Byte 1 Byte 2 Byte3 <Details>
        """
        reqSubHeader = {
            'cmdNo': (0x82, 0x00),
            'inst': [1, 0],
            'attr': 1,
            'service': 0x10,
            'padding': (0, 0)
        }

        reqData = [1, 0, 0, 0]

        ansPacket = self.sendCmd(reqSubHeader, reqData)

        return (ansPacket != None and ansPacket.status == 0)

    # Error Reset
    def resetError(self):
        """
        Command No. 0x82
        Instance "1” for alarm, "2" for Error. Specify “1”. for the error reset
        Attribute Fixed to “1”. Specify “1”.
        Service • Set_Attribute_Single: 0x10 Specify the accessing method to the data.
                0x10 : Execute the specified request

        Data part:
        32bit integer Byte 0 Byte 1 Byte 2 Byte3 <Details>
        """
        reqSubHeader = {
            'cmdNo': (0x82, 0x00),
            'inst': [2, 0],
            'attr': 1,
            'service': 0x10,
            'padding': (0, 0)
        }

        reqData = [1, 0, 0, 0]

        ansPacket = self.sendCmd(reqSubHeader, reqData)

        return (ansPacket != None and ansPacket.status == 0)

    #Read/Write vars (B, I, D)
    def writeVar(self, types, index, value):
        """
        types = "B", "I", "D", "R", "IO"
        index = variable index
        value = value to write to the variable
        """
        commNo = {
            "B": 0x7A,  #Bvar
            "I": 0x7B,  #Ivar
            "D": 0x7C,  #Dvar
            "R": 0x7D,  #Rvar
            "IO": 0x78  #IOvar
        }

        reqSubHeader = {
            'cmdNo': (commNo[types], 0x00),
            'inst': [index, 0],
            'attr': 1,
            'service': 0x10,  #writing
            'padding': (0, 0)
        }

        if (types == "B" or types == "IO"):
            reqData = [value]
        elif (types == 'I'):
            tc = two_comp(value, 16)  #two's complement
            bytes = divmod(
                tc, 0x100
            )  #vrne [celi_del, ostanek]   --->   [bytes / 2^8, bytes % 2^8]
            reqData = [bytes[1], bytes[0]]
        elif (types == "D"):
            tc = two_comp(value, 32)  #two's complement
            bytes = divmod(
                tc, 0x10000
            )  #vrne [celi_del, ostanek]   --->   [bytes / 2^16, bytes % 2^16]
            bytesLow = divmod(bytes[1], 0x100)
            bytesHigh = divmod(bytes[0], 0x100)
            reqData = [bytesLow[1], bytesLow[0], bytesHigh[1], bytesHigh[0]]

        ansPacket = self.sendCmd(reqSubHeader, reqData)

        return (ansPacket != None and ansPacket.status == 0)

    def readVar(self, types, index):
        """
        types = "B", "I", "D", "R", "IO"
        index = variable index. for IO input index = 1 to 512, for IO output index = 1001 to 1512
        """

        commNo = {
            "B": 0x7A,  #Bvar
            "I": 0x7B,  #Ivar
            "D": 0x7C,  #Dvar
            "R": 0x7D,  #Rvar
            "IN": 0x78  #IOvar
        }

        if types == "IN":
            bk_index = index
            index = (index / 8) + 1
            index = int(index)

        reqSubHeader = {
            'cmdNo': (commNo[types], 0x00),  #cmd Nr
            'inst': [index, 0],  #index of var
            'attr': 1,
            'service': 0x0E,  #reading variable
            'padding': (0, 0)
        }

        reqData = []

        ansPacket = self.sendCmd(reqSubHeader, reqData)

        if (ansPacket == None or ansPacket.status != 0):
            return False

        if (types == "B"):  #B var
            #B var - unsigned data
            return (ansPacket.data[0])
        elif (types == "IN"):  #IO var for input
            res = (ansPacket.data[0])
            # convert to binary
            res = bin(res)[2:].zfill(8)
            to_list = list(res)
            # to list of int
            to_list = [int(i) for i in to_list]
            # reverse list
            to_list.reverse()
            selected = to_list[(bk_index % 8) - 1]
            return selected

        elif (types == "I"):  #I var
            #convert received data (2 bytes) to signed integer
            return toSint(ansPacket.data[1] * (1 << 8) + ansPacket.data[0], 16)
        elif (types == "D"):  #D var
            #convert received data (4 bytes) to signed integer
            wordLow = ansPacket.data[1] * (1 << 8) + ansPacket.data[0]
            wordHigh = ansPacket.data[3] * (1 << 8) + ansPacket.data[2]
            return toSint(wordHigh * (1 << 16) + wordLow, 32)

    def readVarCmd(self, cmd, types, index):

        #Command No.
        # commNo = [0x7A,   #Bvar
        #             0x7B,   #Ivar
        #             0x7C,   #Dvar
        #             0x7D]   #Rvar
        """
        Instance (Specify the variable number.) 10
        Attribute Fixed to “1”. Specify “1”.
        Service • Get_Attribute_Single: 0x0E
                • Get_Attribute_All: 0x01
                • Set_Attribute_Single: 0x10
                • Set_Attribute_All: 0x02

        Data part:
        32bit integer Byte 0 Byte 1 Byte 2 Byte3 <Details>
        """
        reqSubHeader = {
            'cmdNo': (cmd, 0x00),  #cmd Nr e.g. 0x77
            'inst': [index, 0],  #index of var
            'attr': 1,
            'service': 0x0E,  #reading variable
            'padding': (0, 0)
        }

        reqData = []

        ansPacket = self.sendCmd(reqSubHeader, reqData)

        if (ansPacket == None or ansPacket.status != 0):
            return False

        if (types == 0):  #B var
            #B var - unsigned data
            return (ansPacket.data[0])
        elif (types == 1):  #I var
            #convert received data (2 bytes) to signed integer
            return toSint(ansPacket.data[1] * (1 << 8) + ansPacket.data[0], 16)
        elif (types == 2):  #D var
            #convert received data (4 bytes) to signed integer
            wordLow = ansPacket.data[1] * (1 << 8) + ansPacket.data[0]
            wordHigh = ansPacket.data[3] * (1 << 8) + ansPacket.data[2]
            return toSint(wordHigh * (1 << 16) + wordLow, 32)

    def readVarData(self, cmd, index):
        # this function auto detects the types from the answer packet.
        #Command No.
        """
        Instance (Specify the variable number.) 10
        Attribute Fixed to “1”. Specify “1”.
        Service • Get_Attribute_Single: 0x0E
                • Get_Attribute_All: 0x01
                • Set_Attribute_Single: 0x10
                • Set_Attribute_All: 0x02

        Data part:
        32bit integer Byte 0 Byte 1 Byte 2 Byte3 <Details>
        """
        reqSubHeader = {
            'cmdNo': (cmd, 0x00),  #cmd Nr e.g. 0x77
            'inst': [index, 0],  #index of var
            'attr': 1,
            'service': 0x0E,  #reading variable
            'padding': (0, 0)
        }

        reqData = []

        ansPacket = self.sendCmd(reqSubHeader, reqData)

        if (ansPacket == None or ansPacket.status != 0):
            return False

        types = ansPacket.add_status[0]
        if (types == 0):  #B var
            #B var - unsigned data
            return (ansPacket.data[0])
        elif (types == 1):  #I var
            #convert received data (2 bytes) to signed integer
            return toSint(ansPacket.data[1] * (1 << 8) + ansPacket.data[0], 16)
        elif (types == 2):  #D var
            #convert received data (4 bytes) to signed integer
            wordLow = ansPacket.data[1] * (1 << 8) + ansPacket.data[0]
            wordHigh = ansPacket.data[3] * (1 << 8) + ansPacket.data[2]
            return toSint(wordHigh * (1 << 16) + wordLow, 32)

    def readDataRAW(self, cmd, index, attr, service=0x0E, types=2):
        # this function auto detects the types from the answer packet.
        #Command No.
        """
        Instance (Specify the variable number.) 10
        Attribute Fixed to “1”. Specify “1”.
        Service • Get_Attribute_Single: 0x0E
                • Get_Attribute_All: 0x01
                • Set_Attribute_Single: 0x10
                • Set_Attribute_All: 0x02

        Data part:
        32bit integer Byte 0 Byte 1 Byte 2 Byte3 <Details>
        """
        # start = pf()
        reqSubHeader = {
            'cmdNo': (cmd, 0x00),  #cmd Nr e.g. 0x77
            'inst': [index, 0],  #index of var
            'attr': attr,
            'service': service,  #reading variable
            'padding': (0, 0)
        }

        reqData = []

        ansPacket = self.sendCmd(reqSubHeader, reqData)
        # print(ansPacket.data)
        # print('response_time',pf()-start)
        if (ansPacket == None or ansPacket.status != 0):
            return False
        # print("data = " ,ansPacket.data, ansPacket.add_status)
        # types = ansPacket.add_status[0]
        if (types == 0):  #B var
            #B var - unsigned data
            return (ansPacket.data[0])
        elif (types == 1):  #I var
            #convert received data (2 bytes) to signed integer
            return toSint(ansPacket.data[1] * (1 << 8) + ansPacket.data[0], 16)
        elif (types == 2):  #D var
            #convert received data (4 bytes) to signed integer
            wordLow = ansPacket.data[1] * (1 << 8) + ansPacket.data[0]
            wordHigh = ansPacket.data[3] * (1 << 8) + ansPacket.data[2]
            return toSint(wordHigh * (1 << 16) + wordLow, 32)
        elif types is None:
            return ansPacket.data

    def getAxisPosition(self, axis=1, pulse=True):
        """get the axis position data.

        Args:
            axis (int, optional): axis number 1 to 8. Defaults to 1.
            pulse (bool, optional): return the pulse value, if false returns the cartesian value. Defaults to True.

        Returns:
            int: signed integer value
        """
        cmd = 0x75  # refer to page 45 of High speed ethernet manual

        attr = 5 + axis  # attribute for axis 1- 8 axis
        if not pulse:
            index = 101  # --> shift by 100 for getting cartesian value
            val = self.readDataRAW(cmd, index, attr, service=0x0E, types=2)
            if axis < 4:
                val = val / 1000
            else:
                val = val / 10000
            return val
        else:
            index = 1  # --> to read pulse data
            return self.readDataRAW(cmd, index, attr, service=0x0E, types=2)

    def batchAxisPosition(self, pulse=True):
        # 44 bytes for axis
        # bulk read the robot position data
        if pulse:
            # read pulse data of axis
            data = self.readDataRAW(0x75, 1, 0, 0x01, types=None)
            # print(data)
        else:
            # read cartesian coordinate (X, Y, Z, Rx, Ry, Rz)
            data = self.readDataRAW(0x75, 101, 0, 0x01, types=None)
        return axisPosition(data)

    def getAxisTorque(self, axis=1):
        """get the axis position data.

        Args:
            axis (int, optional): axis number 1 to 8. Defaults to 1.
            pulse (bool, optional): return the pulse value, if false returns the cartesian value. Defaults to True.

        Returns:
            int: signed integer value
        """
        cmd = 0x77  # refer to page 45 of High speed ethernet manual
        index = 1  # control group 1
        attr = axis  # attribute for axis 1- 8 axis
        return self.readDataRAW(cmd, index, attr, service=0x0E, types=None)

    def batchAxisTorque(self, ):
        # 24 bytes for torques 4 bytes x 6 axis == 6* 2 WORD
        # bulk read the robot torque data
        data = self.readDataRAW(0x77, 1, 0, 0x01, types=None)
        # print(data)
        return axisTorque(data)

    def getManagementTime(self, ):
        # get total motion time --> start time and elapsed time
        # data = self.readDataRAW(0x88, 210, 0, service=0x01,types=None)
        reqSubHeader = {
            'cmdNo': (0x88, 0x00),
            'inst': [210, 0],
            'attr': 0,
            'service': 0x01,
            'padding': (0, 0)
        }

        ansPacket = self.sendCmd(reqSubHeader, [], procDiv=1, as_string=True)
        data = ansPacket.data
        # print(data)
        mng_time = OrderedDict()
        mng_time["Op start time"] = "".join([str(x) for x in data[0:16]])
        mng_time["Elapsed time"] = "".join([str(x) for x in data[16:23]])
        return mng_time


#HELPER functions
#two's complement of the signed integer number
def two_comp(val, nbits):
    return (val + (1 << nbits)) % (1 << nbits)


#convert number to signed integer
def toSint(val, nbits):
    if (val >= (1 << nbits - 1)):
        val = val - (1 << nbits)
    return val
