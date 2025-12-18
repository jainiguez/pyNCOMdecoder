from collections import OrderedDict
from bitstring import BitArray
import math
import struct
import time

ACC_FACTOR = 1e-4
ANG_FACTOR = 1e-5
VEL_FACTOR = 1e-4
IMU_FACTOR = 1e-6


class NCOM(object):
    def __init__(self, ncom_byte):
        self.d = OrderedDict()
        self._data = ()
        self._names = ('Sync', 'Time',
                      'AccX', 'AccY', 'AccZ',
                      'AngX', 'AngY', 'AngZ',
                      'NavStat', 'Checksum_1', 'Lat', 'Long', 'Alti',
                      'Vel_North', 'Vel_East', 'Vel_Down',
                      'Heading', 'Pitch', 'Roll',
                      'Checksum_2', 'Status_channel',
                      'Status')
        self._navstat = {
            0: '0: All quantities in the packet are invalid',
            1: '1: Raw IMU measurements',
            2: '2: Initialising',
            3: '3: Locking',
            4: '4: Locked',
            5: '5: Reserved for "unlocked" navigation output',
            6: '6: Expired firmware',
            7: '7: Blocked firmware',
            10: '10: Status only',
            11: '11: Internal Use. Do not use any values from this message.',
            20: '20: Trigger packet while "initialising"',
            21: '21: Trigger packet while "locking"',
            22: '22: Trigger packet while "locked"',
        }
        self._get_value(ncom_byte)

    def _get_value(self, ncom_byte):
        b = BitArray(bytes=ncom_byte)
        self._data = b.unpack(('uintle:8, uintle:16,'   # Sync, Time
                               'intle:24, intle:24, intle:24,'  # AccX, AccY, AccZ
                               'intle:24, intle:24, intle:24,'  # AngX, AngY, AngZ
                               'uintle:8, uintle:8, ' # NavStat, Checksum_1
                               'floatle:64, floatle:64, uintle:32,'  # NavStat, Lat, Long, Alti
                               'intle:24, intle:24, intle:24,'  # Vel North, Vel East, Vel Down
                               'intle:24, uintle:24, uintle:24, '  # Heading, Pitch, Roll
                               'uintle:8, uintle:8, '  # Checksum_2, Status_channel
                               'bytes:8'
                               ))
        self.d = OrderedDict(zip(self._names, self._data))
        assert self.d['Sync'] == 231, print("Sync byte not matched:0x{:x}".format(self.d['Sync'])) # Always 0xE7
        self.d['AccX'] *= ACC_FACTOR
        self.d['AccY'] *= ACC_FACTOR
        self.d['AccZ'] *= ACC_FACTOR
        self.d['AngX'] *= ANG_FACTOR
        self.d['AngY'] *= ANG_FACTOR
        self.d['AngZ'] *= ANG_FACTOR
        self.d['Vel_North'] *= VEL_FACTOR
        self.d['Vel_East'] *= VEL_FACTOR
        self.d['Vel_Down'] *= VEL_FACTOR
        self.d['Heading'] *= IMU_FACTOR * 180 / math.pi
        self.d['Pitch'] *= IMU_FACTOR * 180 / math.pi
        self.d['Roll'] *= IMU_FACTOR * 180 / math.pi
        self.d['Lat'] *= 180 / math.pi
        self.d['Long'] *= 180 / math.pi
        self.d['NavStat'] = self._navstat.get(self.d['NavStat'], 'Reserved')