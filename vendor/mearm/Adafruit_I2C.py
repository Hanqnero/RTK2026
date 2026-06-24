#!/usr/bin/env python3

import smbus


class Adafruit_I2C:
    @staticmethod
    def getPiRevision():
        """Gets the version number of the Raspberry Pi board."""
        try:
            with open("/proc/cpuinfo", "r", encoding="utf-8") as f:
                for line in f:
                    if line.startswith("Revision"):
                        return 1 if int(line.rstrip()[-4:], 16) < 3 else 2
        except OSError:
            return 0
        except ValueError:
            return 0
        return 0

    @staticmethod
    def getPiI2CBusNumber():
        """Gets the I2C bus number /dev/i2c#."""
        return 1 if Adafruit_I2C.getPiRevision() > 1 else 0

    def __init__(self, address, busnum=-1, debug=False):
        self.address = address
        self.bus = smbus.SMBus(busnum if busnum >= 0 else Adafruit_I2C.getPiI2CBusNumber())
        self.debug = debug

    def reverseByteOrder(self, data):
        """Reverses the byte order of an int value."""
        byte_count = len(hex(data)[2:][::2])
        val = 0
        for _ in range(byte_count):
            val = (val << 8) | (data & 0xFF)
            data >>= 8
        return val

    def errMsg(self):
        print("Error accessing 0x%02X: Check your I2C address" % self.address)
        return -1

    def write8(self, reg, value):
        """Writes an 8-bit value to the specified register/address."""
        try:
            self.bus.write_byte_data(self.address, reg, value)
            if self.debug:
                print("I2C: Wrote 0x%02X to register 0x%02X" % (value, reg))
        except OSError:
            return self.errMsg()
        return None

    def write16(self, reg, value):
        """Writes a 16-bit value to the specified register/address pair."""
        try:
            self.bus.write_word_data(self.address, reg, value)
            if self.debug:
                print(
                    "I2C: Wrote 0x%02X to register pair 0x%02X,0x%02X"
                    % (value, reg, reg + 1)
                )
        except OSError:
            return self.errMsg()
        return None

    def writeList(self, reg, values):
        """Writes an array of bytes using I2C format."""
        try:
            if self.debug:
                print("I2C: Writing list to register 0x%02X:" % reg)
                print(values)
            self.bus.write_i2c_block_data(self.address, reg, values)
        except OSError:
            return self.errMsg()
        return None

    def readList(self, reg, length):
        """Read a list of bytes from the I2C device."""
        try:
            results = self.bus.read_i2c_block_data(self.address, reg, length)
            if self.debug:
                print(
                    "I2C: Device 0x%02X returned the following from reg 0x%02X"
                    % (self.address, reg)
                )
                print(results)
            return results
        except OSError:
            return self.errMsg()

    def readU8(self, reg):
        """Read an unsigned byte from the I2C device."""
        try:
            result = self.bus.read_byte_data(self.address, reg)
            if self.debug:
                print(
                    "I2C: Device 0x%02X returned 0x%02X from reg 0x%02X"
                    % (self.address, result & 0xFF, reg)
                )
            return result
        except OSError:
            return self.errMsg()

    def readS8(self, reg):
        """Reads a signed byte from the I2C device."""
        try:
            result = self.bus.read_byte_data(self.address, reg)
            if result > 127:
                result -= 256
            if self.debug:
                print(
                    "I2C: Device 0x%02X returned 0x%02X from reg 0x%02X"
                    % (self.address, result & 0xFF, reg)
                )
            return result
        except OSError:
            return self.errMsg()

    def readU16(self, reg):
        """Reads an unsigned 16-bit value from the I2C device."""
        try:
            result = self.bus.read_word_data(self.address, reg)
            if self.debug:
                print(
                    "I2C: Device 0x%02X returned 0x%04X from reg 0x%02X"
                    % (self.address, result & 0xFFFF, reg)
                )
            return result
        except OSError:
            return self.errMsg()

    def readS16(self, reg):
        """Reads a signed 16-bit value from the I2C device."""
        try:
            result = self.bus.read_word_data(self.address, reg)
            if result > 32767:
                result -= 65536
            if self.debug:
                print(
                    "I2C: Device 0x%02X returned 0x%04X from reg 0x%02X"
                    % (self.address, result & 0xFFFF, reg)
                )
            return result
        except OSError:
            return self.errMsg()


if __name__ == "__main__":
    try:
        bus = Adafruit_I2C(address=0)
        print("Default I2C bus is accessible")
    except OSError:
        print("Error accessing default I2C bus")
