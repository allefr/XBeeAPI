"""
@author: Brandon Zoss, Francesco Vallegra
MIT - SUTD  2015-2016
"""
from serial import Serial
from time import sleep
from datetime import datetime
import os

# import XBee_msg classes and method for finding a SBee serial device
from XB_Finder import serial_ports
from XBee_msg import *

# create dictionary for API operations
APIop = {0x88: ['AT Command Response', XB_locAT_IN, 1],     # name, reference to function, print (y/n)
         0x8B: ['Transmit Status', XB_RFstatus_IN, 1],
         0x8D: ['Route Information Packet', XB_RouteInfo_IN, 1],
         0x90: ['Receive Packet (AO=0)', XB_RF_IN, 0],
         0x91: ['Explicit Rx Indicator (AO=1)', XB_RFexpl_IN, 1],
         0x97: ['Remote Command Response', XB_remAT_IN, 1]}


# ===============================================================================
# ===============================================================================
#   XBeeAPI class
# ===============================================================================
# ===============================================================================
class XBee_module:

    def __init__(self, port=None, baud=9600, ID=0x7FFF, AP=0x02, AO=0x00, NO=0x04):
        """
        XBee initialization
        """

        if not port:
            port = serial_ports()

        # list where to save received transmits (as objects of class XBee_msg or children)
        self.RxMsg = list()

        # input buffer including all bytes from serial
        self.RxBuff = bytearray()

        # XBee configuration
        self.XBconf = {'ID': ID,    # Network ID (between 0x0000 and 0x7FFF)
                       'AP': AP,    # API mode
                       'AO': AO,    # API Output format
                       'NO': NO}    # include additional info when network and neighbor discovery

        # enquire main parameters from local XBee
        self.params = {'ID': [],    # Network ID (between 0x0000 and 0x7FFF)
                       'CE': [],    # Node type
                       'BH': [],    # Broadcast radius
                       'NT': [],    # Maximum seconds for waiting a NetworkDiscover reply (10th of sec)
                       'SH': [],    # Local Address High (high 32bit RF module's address)
                       'SL': [],    # Local Address Low (low 32bit RF module's address)
                       'DH': [],    # Destination Address High
                       'DL': [],    # Destination Address Low
                       'AP': [],    # API mode [0:no API; 1:API no escape seq; 2:API with escape seq]
                       'AO': []}    # API output format [0:standard frames; 1:explicit addressing frames]

        # diagnostic parameters
        self.diagn = {'GD': [],     # number of good frames
                      'EA': [],     # number of timeouts
                      'TR': [],     # number of transmission errors
                      'DB': []}     # RSSI (signal strength) of last received packet [-dBm]

        # set serial port and baud
        self.serial_port = Serial(port=port, baudrate=baud)

        # make sure no information is in the serial buffer
        self._flush()
        sleep(0.2)
        self._flush()

        # start logging RAW data from/to XBee
        path = "./Output/"
        if not os.path.exists(path):
            os.makedirs(path)
        self.rawLogFileIDstr = path + "XBeeRAW_log" + str(datetime.datetime.now()) + ".txt"
        self.testLogFileIDstr = path + "XBeeTEST_log" + str(datetime.datetime.now()) + ".txt"
        self.setFile()

        logStr = '\n\nXBee Communicating via ' + port
        print(logStr)
        self.logRAWtofile(logStr)
        logStr = 'Programming XBee for API mode {0}...'.format(self.XBconf['AP'])
        print(logStr)
        self.logRAWtofile(logStr)

        # set parameters' values from self.XBconf
        for param in self.XBconf:
            print(self.setLocalRegister(param, self.XBconf[param]))
            sleep(0.05)
            self.readSerial()

        # get parameters' values from XBee memory
        for param in self.params:
            print(self.getLocalRegister(param))
            sleep(0.05)
            self.readSerial()

        logStr = 'XBee initialization complete!\n'
        print(logStr)
        self.logRAWtofile(logStr)


# ===============================================================================
#   define methods for Frame-Specific Data Construction
# ===============================================================================
    def setLocalRegister(self, command, value, frame_ID=0x52):
        return self._setgetLocalRegister(command, value, frame_ID=frame_ID)

    def getLocalRegister(self, command, frame_ID=0x52):
        return self._setgetLocalRegister(command, frame_ID=frame_ID)

    def _setgetLocalRegister(self, command, value=None, frame_ID=0x52):
        """
        Query or set module parameters on the local device.

        :param command: local AT command as 2 ASCII string
        :param value: if provided, value to assign.
                If not provided, meaning a request
        :param frame_ID: default value 0x52. Change it if you know what you are doing..
        :return: XBee_msg object containing the created message.
                Can be printed using print(setLocalRegister(..))
        """
        # create new XB_locAT_OUT object and write to serial
        XBmsg = XB_locAT_OUT(self.params, command, regVal=value, frame_ID=frame_ID)
        if XBmsg.isValid():
            self._write(XBmsg.genFrame())

            # log RAW msg
            self.logRAWtofile(XBmsg)

        return XBmsg

    def setRemoteRegister(self, destH, destL, command, value, frame_ID=0x01):
        return self._setgetRemoteRegister(destH, destL, command, value, frame_ID=frame_ID)

    def getRemoteRegister(self, destH, destL, command, frame_ID=0x01):
        return self._setgetRemoteRegister(destH, destL, command, frame_ID=frame_ID)

    def _setgetRemoteRegister(self, destH, destL, command, value=None, frame_ID=0x01):
        """
        Query or set module parameters on a remote device.

        :param command: remote AT command as 2 ASCII string
        :param value: if provided, value to assign.
                If not provided, meaning a request
        :param frame_ID: default value 0x52. Change it if you know what you are doing..
        :return: XBee_msg object containing the created message.
                Can be printed using print(setRemoteRegister(..))
        """
        self.params['DH'] = destH
        self.params['DL'] = destL

        # create new XB_remAT_OUT object and write to serial
        XBmsg = XB_remAT_OUT(self.params, command, regVal=value, frame_ID=frame_ID)
        if XBmsg.isValid():
            self._write(XBmsg.genFrame())

            # log RAW msg
            self.logRAWtofile(XBmsg)

        return XBmsg

    def sendDataToRemote(self, destH, destL, data, frame_ID=0x01, option=0x00, reserved='fffe'):
        """
        Send data as an RF packet to the specified destination.

        :param destH: high address of the destination XBee
        :param destL: low address of the destination XBee
        :param data: content of the transmit as bytearray
        :param frame_ID: default value 0x01. Change it if you know what you are doing..
        :param option: default value 0x00. can be changed to 0x08 for trace routing
        :param reserved: should be 'FFFE' unless for trace routing = 'FFFF'
        :return: XBee_msg object containing the created message.
                Can be printed using print(sendDataToRemote(..))
        """
        # check data consistency
        if type(data) == str:
            # change it to bytearray
            data = bytearray(data.encode())

        self.params['DH'] = destH
        self.params['DL'] = destL

        # create new XB_RF_OUT object and write to serial
        XBmsg = XB_RF_OUT(self.params, data, frame_ID=frame_ID, option=option, reserved=reserved)
        if XBmsg.isValid():
            self._write(XBmsg.genFrame())

            # log RAW msg
            self.logRAWtofile(XBmsg)

        return XBmsg

    def broadcastData(self, data, frame_ID=0x01, option=0x00, reserved='fffe'):
        """
        Send data as an RF packet to the specified destination.

        :param destH: high address of the destination XBee
        :param destL: low address of the destination XBee
        :param data: content of the transmit as bytearray
        :param frame_ID: default value 0x01. Change it if you know what you are doing..
        :param option: default value 0x00. can be changed to 0x08 for trace routing
        :param reserved: should be 'FFFE' unless for trace routing = 'FFFF'
        :return: XBee_msg object containing the created message.
                Can be printed using print(sendDataToRemote(..))
        """
        destH = '00000000'
        destL = '0000ffff'

        # same as sendDataToRemote() with defined destH and destL
        return self.sendDataToRemote(destH, destL, data, frame_ID=frame_ID, option=option, reserved=reserved)


# ===============================================================================
#   Serial communication with XBee
# ===============================================================================
    def _flush(self):
        """
        Dump all data waiting in the incoming
        serial port to be sure there is no data
        upon looking for specified responses
        """
        self.serial_port.flush()
        self.serial_port.flushInput()
        self.serial_port.flushOutput()

        while self.serial_port.inWaiting():
            self.serial_port.read()

    def _write(self, msg):
        """
        send data to serial communication to XBee
        :param msg: DigiMesh frame as bytearray
        :return: None
        """
        self.serial_port.write(msg)

    def readSerial(self):
        """
        Receives data from serial and
        checks buffer for potential messages.
        Stacks all messages into the queue
        if available, for later execution.
        """
        # clear received correct message as object list
        self.RxMsg = list()

        # Read incoming buffer and pack it into the RxStream
        while self.serial_port.inWaiting():
            incoming = self.serial_port.read()
            self.RxBuff.extend(incoming)
            # sleep(0.001)

        # Split the Rx stream by XBee Start byte (0x7E). OBS: byte 0x7E is stripped away!
        msgs = self.RxBuff.split(bytes(b'\x7E'))

        # Add the good messages to the Rx Frames buffer
        self._stack_frame(msgs)

        # Hold on to remaining bytes that may have not validated
        if len(self.RxMsg) > 0 and not self.RxMsg[-1].isValid():
            # discard last obj and put msg back in buffer
            self.RxMsg.pop(-1)
            self.RxBuff = msgs[-1][1:]
        elif not XBee_msg.validate(XBee_msg.unescape(msgs[-1]))[0]:
            self.RxBuff = msgs[-1][1:]
        else:
            # clear the buffer
            self.RxBuff = bytearray()

        return self.RxMsg


# ===============================================================================
#   Operations on received frames
# ===============================================================================
    def _stack_frame(self, msgs):
        """
        Validate incoming messages.  If message
        is valid, stack into inbox of frames to
        be executed
        """
        for msg in msgs:
            # put back previously removed start delimiter
            msg.insert(0, 0x7E)

            # use static methods from XBee_msg class to validate the msg
            msg = XBee_msg.unescape(msg)
            if not XBee_msg.validate(msg)[0]:
                continue

            # create XB_msg object depending on the frame_type
            try:
                # use frame_type info (msg[3]) to get the relative function from APIop dictionary
                recXB = APIop[msg[3]][1](self.params, msg)
                # print(recXB.getHexCmd())
                # check validity
                if recXB.isValid():
                    # log msg
                    self.logRAWtofile(recXB)
                    # if print flag on, then print on screen
                    if APIop[msg[3]][2]:
                        print(recXB)
                # else:
                #     print('failed frame validation on: {0}'.format(''.join('{:02x}'.format(byte) for byte in msg)))
                #     try:
                #         print(msg.decode())
                #     except: pass

                # append object to list of received messages -> note also if verification failed
                self.RxMsg.append(recXB)
            except KeyError:
                print('frame type 0x' + '{:02X}'.format(msg[3]) + ' not recognized/yet not coded')
                # print(''.join('{:02x}'.format(byte) for byte in msg))
                # try:
                #     print(msg.decode())
                # except: pass


# ===============================================================================
#   Custom methods for swarming
# ===============================================================================
#     def swarmTopologicalBroadcast(self):
#         # TODO: here create a function which, before broadcasting, first looks and
#         # finds the neighbors, put a limit on them (topology), and then makes single
#         # communication to them, expecting the ACK.
#         # this is because when using the XBee broadcast, every device sends the info
#         # MT+1 times to every other device within its RF range. Furthermore, since
#         # each device in our project is a router (not endpoint), then each device
#         # will then re-broadcast MT+1 times the same info.
#         # Considering our RF range is now pretty big (~400m) to total communications
#         # initiated by 1 broadcast is huge (MT+1)*n, where n=num devices
#
#         # problem here is however how to know when, if received something, if need to
#         # broadcast or not.. (if it was alr received before)
#
#         return

    def findNeighbors(self, destH, destL):
        """
        XBee provides a feature to discover and report all RF modules found within
        immediate RF range

        :param destH: high address of link receiving device
        :param destL: low address of link receiving device
        :return:
        """
        if destH.upper() == 'LOCAL' or destL.upper() == 'LOCAL':
            print(self.getLocalRegister('FN'))

            return
        elif destH.upper() == 'GLOBAL' or destL.upper() == 'GLOBAL':
            print('Attempting to broadcast findNeighbors! not allowed :(')
            return

        # check consistency of the input address
        destH = self._checkAddrConsistency(destH)
        destL = self._checkAddrConsistency(destL)
        if not destH or not destL:
            return

        # if address equal to local XBee, then use local register methods
        if destH.lower() == self.params['SH'] and destL.lower() == self.params['SL']:
            print(self.getLocalRegister('FN'))

            return
        # if address is for broadcast, then stop! not allowed
        elif destH.lower() == '00000000' and destL.lower() == '0000ffff':
            print('Attempting to broadcast findNeighbors! not allowed :(')
            return

        print(self.getRemoteRegister(destH, destL, 'FN'))


# ===============================================================================
#   Test Link
# ===============================================================================
    def linkQualityTest(self, senderH, senderL, destH, destL, byteToTest=0x20, iterationsToTest=200):
        """
        XBee provided features, which allows to test the link between the local
        device and a remote one. Note is not possible to broadcast this information

        :param senderH: high address of link starting device
        :param senderL: low address of link starting device
        :param destH: high address of link receiving device
        :param destL: low address of link receiving device
        :param byteToTest: number of bytes to test in each iteration
        :param iterationsToTest: number of iterations for repeating the test.
        :return: None
        """
        # check data consistency
        senderH = self._checkAddrConsistency(senderH)
        if not senderH:
            return
        senderL = self._checkAddrConsistency(senderL)
        if not senderL:
            return
        destH = self._checkAddrConsistency(destH)
        if not destH:
            return
        destL = self._checkAddrConsistency(destL)
        if not destL:
            return

        # set high and low destination address to local address: this is from where to start the linkTest
        self.params['DH'] = senderH
        self.params['DL'] = senderL

        # define test data as bytearray as: destination address, payload size (2bytes), iterations (max 4000)
        testData = bytearray.fromhex(destH) + bytearray.fromhex(destL) \
                   + bytearray.fromhex('{:04x}'.format(byteToTest)) \
                   + bytearray.fromhex('{:04x}'.format(iterationsToTest))
        XBmsg = XB_RFexpl_OUT(self.params, testData, 0xE6, 0xE6, '0014')
        if XBmsg.isValid():
            self._write(XBmsg.genFrame())
            print(XBmsg)
            # print(XBmsg.getHexCmd())

    @staticmethod
    def _checkAddrConsistency(addr):
        """
        check the given address is of type str and length 8.
        If addr is provided as bytearray it is converted to str.

        :param addr: address to check
        :return: the formatted address or None if not correct
        """
        if type(addr) is bytearray and len(addr) == 4:
            return ''.join('{:02x}'.format(byte for byte in addr))
        elif type(addr) is str and len(addr) == 8:
            return addr
        else:
            print("Address must be provided as string, length 8!")
            return None

    def networkDiscover(self):
        """
        XBee provides a feature to discover and report all RF modules found using
        the same 'ID' (Network ID)

        :return:
        """
        print(self.getLocalRegister('ND'))

    def traceRoute(self, destH, destL):
        """
        Attempt a route tracing when sending data to the defined XBee.
        Not acceptable to trace local XBee or broadcast

        :param destH: high address of link receiving device
        :param destL: low address of link receiving device
        :return:
        """
        if destH.upper() == 'LOCAL' or destH.upper() == 'LOCAL':
            print('Attempting to trace routing the local XBee! not allowed :(')
            return
        elif destH.upper() == 'GLOBAL' or destH.upper() == 'GLOBAL':
            print('Attempting to broadcast trace routing! not allowed :(')
            return

        # check consistency of the input address
        destH = self._checkAddrConsistency(destH)
        destL = self._checkAddrConsistency(destL)
        if not destH or not destL:
            return

        # if address is local or for broadcast, then stop! not allowed
        if destH.lower() == self.params['SH'] and destL.lower() == self.params['SL']:
            print('Attempting to trace routing the local XBee! not allowed :(')
            return
        elif destH.lower() == '00000000' and destL.lower() == '0000ffff':
            print('Attempting to broadcast trace routing! not allowed :(')
            return

        msg = self.sendDataToRemote(destH, destL, bytearray([1, 2, 3]), option=0x08, reserved='ffff')
        # print(msg.getHexCmd())
        print(msg)

# ===============================================================================
#   Log RAW data coming/outgoing from/to XBee
# ===============================================================================
    def setFile(self):
        """
        Create file for storing incoming strings.
        File is continuously appended to itself -> may want to change this to create new file every session
        """

        # file = open(self.rawLogFileIDstr, 'w+')

        print("Storing RAW Xbee log to: " + self.rawLogFileIDstr)
        print("Storing test Xbee log to: " + self.testLogFileIDstr)
        fileID = open(self.testLogFileIDstr, 'a')
        fileID.write("\n\n\n" + str(datetime.datetime.now()) + " new TEST attempt\n")
        fileID.flush()

    def logRAWtofile(self, line):
        """
        Write RAW data streams to a file
        for post-processing and logging
        """
        with open(self.rawLogFileIDstr, 'a') as fileID:
            fileID.write(str(line) + '\n')
            fileID.flush()

    def logTESTtofile(self, line):
        """
        Write TEST data streams to a file
        for post-processing and logging
        """
        with open(self.testLogFileIDstr, 'a') as fileID:
            fileID.write(str(line))
            fileID.flush()


# ===============================================================================
#   Establish Print Method
# ===============================================================================
    def __str__(self):

        return "XBee " + str(self.params['SL']) + " Communicating via " + self.serial_port
