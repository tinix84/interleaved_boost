''' 

'''

import serial
import time
import os, sys
import re
import struct
import logging


# create logger instance
logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)
# create console handler and set level to debug
ch = logging.StreamHandler()
ch.setLevel(logging.DEBUG)
# create formatter
formatter = logging.Formatter('%(levelname)s - %(message)s')
# add formatter to ch
ch.setFormatter(formatter)
# add ch to logger
logger.addHandler(ch)


class HWRCommunication:

    def __init__(self, comport, baudrate, commands_fn):
        # create commands dict from C definitions 
        self.commands_dict = self.commands_to_dict(commands_fn)
        #print(self.commands_dict)
        # set start byte
        self.start_byte = b'\x55'
        # open serial port
        try:
            self.ser = serial.Serial(comport, 
                                     baudrate=baudrate, 
                                     timeout=0.5)
        except:
            print('Aborted')
               
    
    def commands_to_dict(self, commands_fn):
        dict_out = dict()
        # Read file 
        file = open(commands_fn, 'r')
        enum_in = file.read()
        file.close()
        # regulat expression
        data_re = re.compile(r' {4}(?P<name>\w+) = (?P<data>\d+)')
        data_re_iter = data_re.finditer(enum_in)
        for data in data_re_iter:
            if data:
                #print('{} = {}'.format(data.group('name'), data.group('data')))    
                dict_out[data.group('name')] = int(data.group('data'))
        return dict_out

    def flaotToBytes(self, float_val):
        return bytes(struct.pack('f', float_val))

    def bytesToFloat(self, bytes_val):
        return struct.unpack('f', bytes_val)
        
    def intToBytes(self, float_val):
        return bytes(struct.pack('I', int(float_val)))

    def bytesToInt(self, bytes_val):
        return struct.unpack('I', bytes_val)
        
    def packFrame(self, command_str, data_bytes = b'\x00\x00\x00\x00'):
        command_byte = self.commands_dict[command_str].to_bytes(1, 'big')
        length_byte = len(data_bytes).to_bytes(1, 'big')
        subframe = self.start_byte + command_byte + length_byte + data_bytes
        
        # perform checksum calculation
        checksum = 0
        for b in subframe:
            checksum = checksum ^ int(b) 
        frame_bytes = subframe + checksum.to_bytes(1, 'big')
        return frame_bytes
        
    def unpackFrame(self, frame_bytes):
        command = int(frame_bytes[1])
        length = int(frame_bytes[2])
        data_bytes = frame_bytes[3:-1]
        # perform checksum calculation
        checksum = 0
        for b in frame_bytes[:-1]:
            checksum = checksum ^ b 
        data = None
        if checksum == frame_bytes[-1]:
            if length == 4:
                data = self.bytesToInt(data_bytes)[0]
            else:
                print('receive length error on cmd {}'.format(command))
            
        else:
            print('receive checksum error')
        return data
        
    # def unpackFrameMulti(self, frame_bytes):
        # command = int(frame_bytes[1])
        # length = int(frame_bytes[2])
        # data_bytes = frame_bytes[3:-1]
        # # perform checksum calculation
        # checksum = 0
        # for b in frame_bytes[:-1]:
            # checksum = checksum ^ b 
        
        # if checksum == frame_bytes[-1]:
            # if command == self.commands_dict['multi']:
                # data = list()
                # while(len(data_bytes) > 4):
                    # data.append(self.bytesToFloat(data_bytes[1:5])[0])
                    # data_bytes = data_bytes[5:]            
            # else:
                # if length == 4:
                    # data = self.bytesToFloat(data_bytes)[0]
                # else:
                    # data = None
                    # print('unpack error')
        # return data


    def transmitReceiveFrame(self, frame_bytes):
        if len(frame_bytes) < 128:
            # while(self.ser.out_waiting > 0):
                # pass
            self.ser.write(frame_bytes)
            time.sleep(.5)
            msg = self.ser.read(len(frame_bytes))
        else:
            msg = b''
            print('Frame too long')
        logger.debug('TXmsg: 0x{}, RXmsg: 0x{}'.format(frame_bytes.hex(), msg.hex()))
        return bytes(msg)
                
    def send(self, command, data=None):
        if any(command in s for s in list(self.commands_dict.keys())):
            if data is not None:
                frame_bytes = self.packFrame(command, self.intToBytes(float(data)))
            else:
                frame_bytes = self.packFrame(command)
            print(frame_bytes.hex())
            response_bytes  = self.transmitReceiveFrame(frame_bytes)
            print(response_bytes.hex())
            
            ret = self.unpackFrame(response_bytes)   
            print('response of cammand {}'.format(ret))
                
        else:
            print('Command error!')
            ret = None
        return ret
  
    # def sendMulti(self, command_list, data_list):
        
        # comdata = bytes()
        
        # for command, data in zip(command_list, data_list):
            # if any(command in s for s in list(self.commands_dict.keys())):
                    # comdata = comdata + self.commands_dict[command].to_bytes(1, 'big') + self.flaotToBytes(float(data))
            # else:
                # print('Command error!')   
        # #print(comdata.hex())        
        # frame_bytes = self.packFrame('multi', comdata)
        # #frame_bytes = self.packFrame('echo', comdata)
        # #print(frame_bytes.hex())
        # response_bytes  = self.transmitReceiveFrame(frame_bytes)
        # #print(response_bytes.hex())
        # ret = self.unpackFrameMulti(response_bytes)        #ret=0
        # return ret


# def test(hwr):
    
    # for command in hwr.commands_dict:
        # if command != 'multi':
            # if 'get' in str(command):
                # # getter command
                # resp = hwr.send(command)
                # print('comm:{}, resp: {}'.format(str(command), resp))
            # if ('Enable' in str(command)) or ('Disable' in str(command)):
                # resp = hwr.send(command)
                # print('comm:{},resp: {}'.format(str(command), resp))
            # if 'DutyCycle' in str(command):
                # resp = hwr.send(command, 0.2)
                # print('comm:{}, resp: {}'.format(str(command), resp))


# def testMulti(hwr):
    # #del hwr.commands_dict['multi']
    # cmds = list()
    # dta = list()
    # for command in hwr.commands_dict:
        # if command != 'multi':
            # # if 'get' in str(command):
                # # cmds.append(command)
                # # dta.append(0.0)
            # if ('Enable' in str(command)) or ('Disable' in str(command)):
                # cmds.append(command)
                # dta.append(0.0)
            # # if 'DutyCycle' in str(command):
                # # cmds.append(command)
                # # dta.append(0.2)
    # # print(len(cmds)*5)
    # # print(cmds)
    # resp = hwr.sendMulti(cmds, dta)
    # print(resp)
    # for cmd , dta in zip(cmds, resp):
        # print('comm:{}, resp: {}'.format(cmd, dta))
    
    
# def getAllMulti(hwr):
    # cmds = list()
    # dta = list()
    # for command in hwr.commands_dict:
        # if 'get' in str(command):
            # cmds.append(command)
            # dta.append(0.0)
    # resp = hwr.sendMulti(cmds, dta)
    # if resp:
        # strng = ''
        # for cmd , dta in zip(cmds, resp):
            # #print('{0} = {1}{2:2.3f}'.format(cmd,(20-len(cmd))*' ',  dta))
            # strng = strng + '{0} = {1}{2:2.3f}\n'.format(cmd,(20-len(cmd))*' ',  dta)
        # os.system('cls')
        # print(strng)
        
def getAll(hwr):
    strng = ''
    for command in hwr.commands_dict:
        if 'get' in str(command):
            #print('perform command {}'.format(command))
            resp = hwr.send(command)
            if resp is not None:
                strng = strng + '{0} = {1}{2:2.3f}\n'.format(command,(30-len(command))*' ',  resp)
        #time.sleep(0.01)
    os.system('cls')
    print(strng)
    
    
def run(hwr):
    while (1):
        resp = hwr.send('setFrequency', '200')
        print('{} = {}'.format('setFrequency', resp))
        time.sleep(1)
    
    
def main():

    hwr = HWRCommunication(comport='COM5', 
                           baudrate=9600, 
                           commands_fn="./commands.h")
         

    if len(sys.argv) > 1:
        if len(sys.argv) == 2:
            if '-cmds' == sys.argv[1]:
                for cmd in hwr.commands_dict.keys():
                    print(cmd)
            elif '-run' == sys.argv[1]:
                run(hwr)
            else:
                resp = hwr.send(sys.argv[1])
                print('{} = {}'.format(str(sys.argv[1]), resp))
        if len(sys.argv) == 3:
            resp = hwr.send(sys.argv[1], sys.argv[2])
            print('{} = {}'.format(str(sys.argv[1]), resp))

    else:
        #testMulti(hwr)
        #getAll(hwr)
        while (True):
            getAll(hwr)
            time.sleep(1)

    
if __name__ == "__main__":
    main()
    
    
    
