
import atexit
from binascii import hexlify
import logging
import math
import struct

from PyMata.pymata import PyMata

# Constants that define the Circuit Playground Firmata command values.

LMD_COMMAND                 = 0x01                          # Byte that identifies all Circuit Playground commands.
LMD_IMU_READ                = 0x10                          # Return the current x, y, z accelerometer values.
LMD_IMU_READ_REPLY          = 0x11                          # Result of an IMU read.  Includes 3 floating point values
                                                            # (4 bytes each) with omega, theat, zeta angles

LMD_ACCEL_READ              = 0X20                          #RETURN THE CURRENT LINEAR ACCELERATION VALUES, a_x, a_y, a_z
LMD_ACCEL_READ_REPLY        = 0X21                          #Result of an linear accel read, includes 3 floating points

logger = logging.getLogger(__name__)


class LMD_firmata_addons(PyMata):

    def __init__(self, port_id='COM3', bluetooth=True, verbose=True):           #/dev/ttyACM0
        # PyMata is an old style class so you can't use super.
        PyMata.__init__(self, port_id, bluetooth, verbose)
        # Setup handler for response data.
        # Note that the data length (1) appears to be unused for these sysex
        # responses.
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
        """Parse a 4 byte floating point value from a 7-bit byte firmata response
        byte array.  Each pair of firmata 7-bit response bytes represents a single
        byte of float data so there should be 8 firmata response bytes total.
        """

        if len(data) != 8:
            raise ValueError('Expected 8 bytes of firmata response for floating point value!')
        # Convert 2 7-bit bytes in little endian format to 1 8-bit byte for each
        # of the four floating point bytes.
        raw_bytes = bytearray(4)
        for i in range(4):
            raw_bytes[i] = self._parse_firmata_byte(data[i * 2:i * 2 + 2])
        # Use struct unpack to convert to floating point value.
        return struct.unpack('<f', raw_bytes)[0]

    def _parse_firmata_long(self, data):
        """Parse a 4 byte signed long integer value from a 7-bit byte firmata response
        byte array.  Each pair of firmata 7-bit response bytes represents a single
        byte of long data so there should be 8 firmata response bytes total.
        """

        if len(data) != 8:
            raise ValueError('Expected 8 bytes of firmata response for long value!')
        # Convert 2 7-bit bytes in little endian format to 1 8-bit byte for each
        # of the four floating point bytes.
        raw_bytes = bytearray(4)
        for i in range(4):
            raw_bytes[i] = self._parse_firmata_byte(data[i * 2:i * 2 + 2])
        # Use struct unpack to convert to floating point value.
        return struct.unpack('<l', raw_bytes)[0]

    def _response_handler(self, data):
        """Callback invoked when a circuit playground sysex command is received.
        """
        logger.debug('LMD response: 0x{0}'.format(hexlify(bytearray(data))))
        if len(data) < 1:
            logger.warning('Received response with no data!')
            return
        # Check what type of response has been received.
        command = data[0] & 0x7F
        if command == LMD_IMU_READ_REPLY:
            # Parse IMU response.
            if len(data) < 26:
                logger.warning('Received IMU response with not enough data!')
                print(' received the Orinetation information')
                return
            omega = self._parse_firmata_float(data[2:10])
            theta = self._parse_firmata_float(data[10:18])
            zeta = self._parse_firmata_float(data[18:26])
            if self._imu_callback is not None:
                self._imu_callback(omega, theta, zeta)

        elif command == LMD_ACCEL_READ_REPLY:
            # Parse accelerometer response.
            if len(data) < 34:
                print('Received Acceleration response with not enough data!!')
                print(' received the Acceleration response')
                return

            x = self._parse_firmata_float(data[2:10])
            y = self._parse_firmata_float(data[10:18])
            z = self._parse_firmata_float(data[18:26])
            t = (self._parse_firmata_long(data[26:34])) / 1000         # gather time info and translate into seconds
            if self._accel_callback is not None:
                self._accel_callback(x, y, z, t)
        else:
            logger.warning('Received unexpected response!')

    def read_imu(self, callback):
        """Request an accelerometer reading.  The result will be returned by
        calling the provided callback function and passing it 3 parameters:
            - Omega angle
            - theta angle
            - Zeta angle
            """
        self._imu_callback = callback
        self._command_handler.send_sysex(LMD_COMMAND, [LMD_IMU_READ])

    def read_accel(self, callback):
        """Request an accelerometer reading.  The result will be returned by
        calling the provided callback function and passing it 3 parameters:
            - X  angular acceleration
            - Y angular acceleration
            - Z angular acceleration
            """
        self._accel_callback = callback
        self._command_handler.send_sysex(LMD_COMMAND, [LMD_ACCEL_READ])
