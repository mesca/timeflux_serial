import serial
import time
import numpy as np
import pandas as pd
from timeflux.core.exceptions import WorkerInterrupt
from timeflux.helpers import clock
from timeflux.core.node import Node


class SerialDevice(Node):

    """A generic serial device driver.

    Attributes:
        o (Port): Stream from the serial device, provides DataFrame.

    Args:
        port (string): The serial port.
            e.g. ``COM3`` on Windows;  ``/dev/tty.tty.SLAB_USBtoUART`` on MacOS;
            ``/dev/ttyUSB0`` on GNU/Linux.
        rate (int): The device rate in Hz.
            Possible values: ``1``, ``10``, ``100``, ``1000``. Default: ``1000``.

    Example:
        .. literalinclude:: /../test/graphs/test.yaml
           :language: yaml

    Notes:

    .. attention::

        Make sure to set your graph rate to an high-enough value, otherwise the device
        internal buffer may saturate, and data may be lost. A 30Hz graph rate is
        recommended for a 1000Hz device rate.

    """

    def __init__(self, port, rate=1000):

        # Check port
        if not port.startswith('/dev/') and not port.startswith('COM'):
            raise ValueError(f'Invalid serial port: {port}')

        # Check rate
        if not isinstance(rate, int):
            raise ValueError(f'Invalid rate: {rate}')
        self._rate = rate

        # Sample size in bytes
        self.sample_size = 9

        # Connect to device
        self.device = self._connect(port)
        # try:
        #     self.device = self.connect(port)
        # except Exception as e:
        #     raise WorkerInterrupt(e)

        # Set rate
        self._send(f'rate,{rate}')

        # Start Acquisition
        self._send('start')

        # Initialize counters for timestamp indices and continuity checks
        self._sample_counter = 65535
        self._timestamp = clock.now()


    def update(self):

        data, indices = self._read()
        self.o.set(data, indices, ('SEQ', 'UTIME', 'A0'), {'rate': self._rate})


    def _connect(self, port):
        device = serial.Serial(port, 115200)
        msg = device.readline(); # Wait for 'ready\n'
        try:
            msg.decode()
        except UnicodeDecodeError:
            self.logger.error('Unstable state. Please re-plug the device and start again.')
            # https://stackoverflow.com/questions/21073086/wait-on-arduino-auto-reset-using-pyserial
            # https://forum.arduino.cc/index.php?topic=38981.0
            raise WorkerInterrupt()
        return device


    def _send(self, cmd):
        cmd = (cmd + '\n').encode()
        self.device.write(cmd)


    def _read(self):

        """Read all available data"""

        # Check buffer size and limits
        buffer_size = self.device.in_waiting
        if buffer_size == 1020:
            # The OS serial buffer can hold up to 1020 bytes
            self.logger.warn('Internal buffer saturated. Increase graph rate or decrease device rate.')

        # Compute the maximum number of samples we can get
        sample_count = int(buffer_size / self.sample_size)

        # Read raw samples from device
        now = clock.now()
        raw = self.device.read(sample_count * self.sample_size)

        # Initialize the output matrix
        data = np.full((sample_count, 3), np.nan, np.uint32)

        # Parse the raw data
        for sample_number in range(sample_count):

            # Extract sample
            start = sample_number * self.sample_size
            stop = start + self.sample_size
            sample = raw[start:stop]

            # Check (dummy) CRC
            crc = sample[8]
            if crc != 0:
                self.logger.warn('Corrupted sample.')
                continue

            # Parse sample
            data[sample_number, 0] = int.from_bytes(sample[0:2], byteorder='little', signed=False)
            data[sample_number, 1] = int.from_bytes(sample[2:6], byteorder='little', signed=False)
            data[sample_number, 2] = int.from_bytes(sample[6:8], byteorder='little', signed=False)

            # Did we miss any sample?
            # Check for discontinuity in the internal sample counter (2 bytes).
            sample_counter = data[sample_number, 0]
            if sample_counter == self._sample_counter + 1:
                pass
            elif sample_counter == 0 and self._sample_counter == 65535:
                pass
            else:
                self.logger.warn('Missed sample.')
            self._sample_counter = sample_counter

        # Compute indices
        indices = clock.time_range(self._timestamp, now, len(data))
        self._timestamp = now

        return data, indices


    def terminate(self):
        self._send('stop')
        self.device.close()
