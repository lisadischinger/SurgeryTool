
#import atexit import math
from binascii import hexlify
import logging
import struct
from PyMata.pymata import PyMata

# Constants that define the Circuit Playground Firmata command values.
LMD_COMMAND = 0x01                              # Byte that identifies all Circuit Playground commands.
LMD_IMU1_1_READ = 0x10                          # Return the current x, y, z accel, gyro, and mag values.
LMD_IMU1_2_READ = 0x11
LMD_IMU_READ_REPLY = 0x12                       # Result of an IMU read. Includes 3 floating point values
                                                # (4 bytes each) with omega, theat, zeta angles

logger = logging.getLogger(__name__)
first_imu_read = True
t_edit = 1                                                  # normalizing factor to correct for the clock silliness


class LMD_firmata_addons(PyMata):

    def __init__(self, port_id='COM3', bluetooth=True, verbose=True):           #/dev/ttyACM0
        PyMata.__init__(self, port_id, bluetooth, verbose)
        # Setup handler for response data.
        # Note that the data length (1) appears to be unused for these sysex responses.
        self._command_handler.command_dispatch.update({LMD_COMMAND: [self._response_handler, 1]})
        # Setup configured callbacks to null.
        self._imu_callback = None
        self._accel_callback = None

    def _parse_firmata_byte(self, data):
        """Parse a byte value from two 7-bit byte firmata response bytes."""
        if len(data) != 2:
            raise ValueError('Expected 2 bytes of firmata repsonse for a byte value!')
        return (data[0] & 0x7F) | ((data[1] & 0x01) << 7)

    def _parse_firmata_float(self, data):
        """Parse a 2 byte floating point value from a 7-bit byte firmata response byte array.  Each pair of firmata
        7-bit response bytes represents a single byte of float data so there should be 8 firmata response bytes total.
        """
        if len(data) != 4:                      #8:
            raise ValueError('Expected 8 bytes of firmata response for floating point value!')
        # Convert 2 7-bit bytes in little endian format to 1 8-bit byte for each of the four floating point bytes.
        raw_bytes = bytearray(2)                #(4)
        for i in range(2):                      #(4):
            raw_bytes[i] = self._parse_firmata_byte(data[i * 2:i * 2 + 2])
        return struct.unpack('<f', raw_bytes)[0]        # Use struct unpack to convert to floating point value.

    def _parse_firmata_long(self, data):
        """Parse a 4 byte signed long integer value from a 7-bit byte firmata response byte array.  Each pair of
        firmata 7-bit response bytes represents a single byte of long data so there should be 8 firmata response
        bytes total. """
        if len(data) != 8:
            raise ValueError('Expected 2 bytes of firmata response for long value!')
        # Convert 2 7-bit bytes in little endian format to 1 8-bit byte for each of the four floating point bytes.
        raw_bytes = bytearray(4)
        for i in range(4):
            raw_bytes[i] = self._parse_firmata_byte(data[i * 2:i * 2 + 2])
        return struct.unpack('<l', raw_bytes)[0]        # Use struct unpack to convert to floating point value.

    def _response_handler(self, data):
        """Callback invoked when a circuit playground sysex command is received."""
        global first_imu_read, t_edit

        logger.debug('LMD response: 0x{0}'.format(hexlify(bytearray(data))))
        if len(data) < 1:
            logger.warning('Received response with no data!')
            return
        # Check what type of response has been received.
        command = data[0] & 0x7F
        if command == LMD_IMU_READ_REPLY:
            # Parse IMU response.
            if len(data) < 42:
                logger.warning('Received IMU response with not enough data!')
                print(' received the IMU information')
                return
            t = self._parse_firmata_float(data[2:6])
            a_x = self._parse_firmata_float(data[6:10])
            a_y = self._parse_firmata_float(data[10:14])
            a_z = self._parse_firmata_float(data[14:18])
            g_x = self._parse_firmata_float(data[18:22])
            g_y = self._parse_firmata_float(data[22:26])
            g_z = self._parse_firmata_float(data[26:30])
            m_x = self._parse_firmata_float(data[30:34])
            m_y = self._parse_firmata_float(data[34:38])
            m_z = self._parse_firmata_float(data[38:42])
            if self._imu_callback is not None:
                self._imu_callback(t, a_x, a_y, a_z, g_x, g_y, g_z, m_x, m_y, m_z)

        # elif command == LMD_IMU_READ_REPLY:         # Parse accelerometer response.
        #     if len(data) < 34:
        #         print('Received IMU response with not enough data!!')
        #         print('received the IMUresponse')
        #         return
        #
        #     x = self._parse_firmata_float(data[2:10])
        #     y = self._parse_firmata_float(data[10:18])
        #     z = self._parse_firmata_float(data[18:26])
        #     t = (self._parse_firmata_long(data[26:34]))/1000          # gather time info and translate from millis to s
        #     #if first_accel_read:
        #         #t_edit = t-1                                          # we know this first value is supposed to be about 1, i know this is bad but I
        #         #first_accel_read = False                                #have no idea how to not have an initial time of 1 million
        #     #t = t - t_edit
        #     if self._accel_callback is not None:
        #         self._accel_callback(x, y, z, t)
        else:
            logger.warning('Received unexpected response!')

    def read_imu(self, callback):
        """Request an IMU reading.  The result will be returned by
        calling the provided callback function and passing it 3 parameters for
        each the accelerometer, gyroscope, and magnetometer
            """
        self._imu_callback = callback
        self._command_handler.send_sysex(LMD_COMMAND, [LMD_IMU_READ])

