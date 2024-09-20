import sys
from unionproto_pb2 import UnionMessage, MsgType1, MsgType2, MsgType3
import serial
import struct

# custom Exception
class TransportError(Exception):
    pass

class ProtocolError(Exception):
    pass

def send_message(serial_port, message):
    # Get the message length
    message_length = len(message)

    # Create a length prefix (2 bytes in network byte order)
    length_prefix = struct.pack('>H', message_length)  # '>H' means big-endian unsigned short

    # Send length prefix followed by the message
    serial_port.write(length_prefix)  # Send the length prefix
    serial_port.write(message)         # Send the actual message

def receive_response(serial_port):
    # Receive length prefix (2 bytes)
    length_prefix = serial_port.read(2)
    message_length = struct.unpack('>H', length_prefix)[0]

    # Receive response type (1 byte)
    response_type = serial_port.read(1)

    return message_length, response_type[0]  # Return message length and response type

def process_serial_message(port, baudrate, timeout, msgtype):
    with serial.Serial(port, baudrate=baudrate, timeout=timeout) as ser:
        message = UnionMessage()

        if msgtype == 1:
            message.msg1.value = 69
        elif msgtype == 2:
            message.msg2.value = True
        elif msgtype == 3:
            message.msg3.value1 = 3
            message.msg3.value2 = 1415
        else:
            raise ValueError("Unknown message type")

        encoded_message = message.SerializeToString()
        send_message(ser, encoded_message)
        
        # Wait for response
        length, response = receive_response(ser)
        if response == 0x00:
            print("Success response received")
        elif response == 0x01:
            raise TransportError("Transport error response received!")
        elif response == 0x02:
            raise ProtocolError("Protocol error response received!")
        
        return 0  # Success

if __name__ == "__main__":
    msgtype = int(sys.argv[1])
    process_serial_message('/dev/tty.usbmodem13203', 115200, 1, msgtype)