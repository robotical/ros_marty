#!/usr/bin/env python
# Robotical Ltd. 2016 - Apache License
import signal
import sys
import time
import serial
import StringIO
from rosserial_msgs.msg import TopicInfo
from std_msgs.msg import String, Int8
from marty_msgs.msg import ServoMsg, ServoMsgArray
ROS_ACK = "\xff\xfe\x00\x00\xff\x00\x00\xff"
# ROS_STOP = "\xff\xfe\x00\x00\xff\x0b\x00\xf4"    # TX stop request
ROS_RESP = "\xff\xfe"   # Sync Flag / Protocol version

# ROS msgs md5 checksums
MD5_String = "992ce8a1687cec8c8bd883ec73ca41d1"
MD5_Int8 = "27ffa0c9c4b8fb8492252bcad9e5c57b"
MD5_Int32 = "da5909fbe378aeaf85e547e830cc1bb7"
MD5_ServoMsg = "1bdf7d827c361b33ed3b4360c70ce4c0"
MD5_ServoMsgArray = "320c8def1674d51423fb942c56a6619b"

freq = 1.0

# Catch Ctrl-c
def signal_handler(signal, frame):
        print "EXITING!"
        sys.exit(0)
signal.signal(signal.SIGINT, signal_handler)

# Byte/Hex Manipulation Methods
#-------------------------------------------------------------------------------

def ByteToHex( byteStr ):
    return ''.join( [ "%02X " % ord( j ) for j in byteStr ] ).strip()

#-------------------------------------------------------------------------------

def ByteToInt( byteStr ):
    test = (''.join( [ "%02X " % ord( j ) for j in byteStr ] ).strip()).split()
    for i in xrange(len(test)):
        test[i] = int(test[i], 16)
    return sum(test)

#-------------------------------------------------------------------------------

def HexToByte( hexStr ):
    byte_array = []
    hexStr = ''.join( hexStr.split(" ") )
    for i in range(0, len(hexStr), 2):
        byte_array.append( chr( int (hexStr[i:i+2], 16 ) ) )
    return ''.join( byte_array )

#-------------------------------------------------------------------------------

# SerialMsg Length Checksum
def LCheckSum(data):
    checksum = len(data)
    return " {:02X}".format(255 - (checksum % 256))

# TopicID Checksum
def TCheckSum(topic_id, data):
    checksum = topic_id + ByteToInt(data)
    return " {:02X}".format(255 - (checksum % 256))

# Length as 2 HexBytes
def HexMsgLen(data):
    LEN_HIGH = " {:02X}".format(len(data) / 256)
    LEN_LOW = " {:02X}".format(len(data) % 256)
    return LEN_LOW, LEN_HIGH

# TopicNum as 2 HexBytes
def HexTopID(topic_num):
    TID_HIGH = " {:02X}".format(topic_num / 256)
    TID_LOW = " {:02X}".format(topic_num % 256)
    print "LOW/HIGH: ", TID_LOW, TID_HIGH
    return TID_LOW, TID_HIGH


#-------------------------------------------------------------------------------
# MAIN FUNCTION
#-------------------------------------------------------------------------------

# Init Serial
ser = serial.Serial(
    port='/dev/ttyUSB0',
    baudrate = 115200,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    bytesize=serial.EIGHTBITS,
    timeout=0
)

# Setup Publisher Info
pub_topic_info = TopicInfo()
pub_topic_info.topic_id = 101
pub_topic_info.topic_name = "/marty/chatter"
pub_topic_info.message_type = "std_msgs/String"
pub_topic_info.md5sum = MD5_String
pub_topic_info.buffer_size = 256
# Initialise Pub Message
pub_msg = String()
pub_msg.data = "Testing!"

# Setup Subscriber Info
sub_topic_info = TopicInfo()
sub_topic_info.topic_id = 103
sub_topic_info.topic_name = "/marty/servo_array"
# sub_topic_info.message_type = "std_msgs/Int8"
# sub_topic_info.md5sum = MD5_Int8
sub_topic_info.message_type = "marty_msgs/ServoMsgArray"
sub_topic_info.md5sum = MD5_ServoMsgArray
# sub_topic_info.message_type = "marty_msgs/ServoMsgArray"
# sub_topic_info.md5sum = MD5_ServoMsgArray
sub_topic_info.buffer_size = 32

# Initialise Serial Publisher Buffer
pub_buffer = StringIO.StringIO()
pub_topic_info.serialize(pub_buffer)
pub_topic_data = pub_buffer.getvalue()

# Initialise Serial Message Buffer
pub_msg_buffer = StringIO.StringIO()
pub_msg.serialize(pub_msg_buffer)
msg_data = pub_msg_buffer.getvalue()

# Initialise Serial Subscriber Buffer
sub_buffer = StringIO.StringIO()
sub_topic_info.serialize(sub_buffer)
sub_topic_data = sub_buffer.getvalue()

# Flags
ready = False
pub_init = False
sub_init = False

test_pub_id = 0
test_sub_id = 1

# Wait for ROS Serial ACK, then send info
while 1:
    # Wait for ROSSerial Acknowledgement
    x=ser.readline()
    print "Received: ", ByteToHex(x)
    # if (ready == False):
    #     print "Expecting: ", ByteToHex(ROS_ACK)
    #     if (x==ROS_ACK):
    #         print "GOT ROS ACK!"
    #         ready = True
    #     else:
    #         print "No ROS ACK :("
    # else:
    # Send Publisher Setup
    if (sub_init == False):
        len_low, len_high = HexMsgLen(sub_topic_data)
        # tid_low, tid_high = HexTopID(1)
        tid_low, tid_high = HexTopID(TopicInfo.ID_SUBSCRIBER)
        l_checksum = LCheckSum(sub_topic_data)
        t_checksum = TCheckSum(TopicInfo.ID_SUBSCRIBER, sub_topic_data)
        s_header = ByteToHex(ROS_RESP) + len_low + len_high + l_checksum + tid_low + tid_high
        print "SerialHeader: ", s_header
        print "TopicInfo: ", sub_buffer.getvalue()
        ser.write(HexToByte(s_header) + sub_buffer.getvalue() + HexToByte(t_checksum))
        sub_init = True
        time.sleep(1)
    # Send Message
    else:
        print "Waiting for Sub Msg!"
        # sub_msg_buffer = StringIO.StringIO()
        # print "Length: ", len(x)
        # if (len(x) <= 9):
        #     # print "Received: ", ByteToHex(x)
        #     sub_msg_buffer.write(x)
        #     # print "RecBuff: ", ByteToHex(sub_msg_buffer.getvalue())
        #     # # Initialise Sub Message
        #     sub_msg = Int8()
        #     sub_msg.deserialize(sub_msg_buffer.getvalue())
        #     print "GOT: ", sub_msg.data

        # Send Publisher Setup
        # if (pub_init == False):
        #     len_low, len_high = HexMsgLen(pub_topic_data)
        #     tid_low, tid_high = HexTopID(TopicInfo.ID_PUBLISHER)
        #     # tid_low, tid_high = HexTopID(pub_topic_info.topic_id)
        #     l_checksum = LCheckSum(pub_topic_data)
        #     t_checksum = TCheckSum(TopicInfo.ID_PUBLISHER, pub_topic_data)
        #     s_header = ByteToHex(ROS_RESP) + len_low + len_high + l_checksum + tid_low + tid_high
        #     print "SerialHeader: ", s_header
        #     print "TopicInfo: ", pub_buffer.getvalue()
        #     ser.write(HexToByte(s_header) + pub_buffer.getvalue() + HexToByte(t_checksum))
        #     pub_init = True
        #     time.sleep(1)
        # # Send Message
        # else:
        #     len_low, len_high = HexMsgLen(msg_data)
        #     tid_low, tid_high = HexTopID(pub_topic_info.topic_id)
        #     l_checksum = LCheckSum(msg_data)
        #     t_checksum = TCheckSum(pub_topic_info.topic_id, msg_data)
        #     s_header = ByteToHex(ROS_RESP) + len_low + len_high + l_checksum + tid_low + tid_high
        #     ser.write(HexToByte(s_header) + pub_msg_buffer.getvalue() + HexToByte(t_checksum))
    time.sleep(1.0/freq)
