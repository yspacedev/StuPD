import digitalio
import board
import time
import busio
from adafruit_bus_device.i2c_device import I2CDevice
class INA237:
    def __init__(self, i2c, addr):
        self.i2c = i2c
        self.addr = addr
        self.device = I2CDevice(i2c, addr) #adafruit i2cdevice object thing
        self.CurrADCrange=0b0
        self.currentLSB=5/(2**15)
        #constants:
        self.range_small=0b1 #40.96mV
        self.range_large=0b0 #163.84mV

        #error codes
        self.code_avgdone=13 #by bit
        self.code_overflow=9
        self.code_overtemp=7
        self.code_shuntovervoltage=6
        self.code_shuntundervoltage=5
        self.code_busovervoltage=4
        self.code_busundervoltage=3
        self.code_overpower=2
        self.code_convdone=1
        self.code_memerr=0
    def writeReg(self, reg, val):
        MSB = val >> 8
        LSB = ((val << 8) & 0xFF00) >> 8
        self.device.write(bytes([reg, MSB, LSB]))

    def readReg(self, reg):
        result=bytearray(2)
        self.device.write_then_readinto(bytes([reg]), result)
        result = self.from_bytes(result, 'big')
        return result

    def readSubReg(self, reg, endpos, startpos):
        val = self.readReg(reg)

        val = val >> startpos
        val = val & ((1 << (endpos-startpos+1)) - 1)

        return val

    def writeSubReg(self, reg, val, endpos, startpos): #write from startpos to endpos, inclusive. Zero-indexed
        oldval = self.readReg(reg)#int((self.readReg(reg)[0]<<8)+(self.readReg(reg)[1]))
        #print(bin((self.readReg(reg)[0])))
        #print(bin((self.readReg(reg)[1])))

        mask = ((1 << (endpos-startpos+1)) - 1) << startpos
        val = val << startpos
        val = val & mask
        oldval = oldval & ~mask
        oldval = oldval | val
        self.writeReg(reg, oldval)

    def unlock(self):
        self.i2c.unlock()

    def relock(self):
        while not self.i2c.try_lock():
            pass

    def from_bytes(self, byte, byteorder='big', signed=False):
        if byteorder == 'little':
            little_ordered = list(byte)
        elif byteorder == 'big':
            little_ordered = list(reversed(byte))
        else:
            raise ValueError("byteorder must be either 'little' or 'big'")

        n = sum(b << i*8 for i, b in enumerate(little_ordered))
        if signed and little_ordered and (little_ordered[-1] & 0x80):
            n -= 1 << 8*len(little_ordered)
        return n

    def twosCompConv(self, val, bits):
        result=0
        negative=False
        if (val>>(bits-1)==0b1):
            negative=True
            result=((val-(2**bits)))
        else:
            result=val
        return result

    def reset(self): #reset INA237
        self.writeSubReg(0x00, 0b1, 15, 15)
    def setConvDelay(self, delay):
        self.writeSubReg(0x00, (delay&0xFF), 13, 6)
    def setADCrange(self, _range):
        self.writeSubReg(0x00, (_range&0b1), 4, 4)
        self.CurrADCrange=_range
    def setMode(self, mode):
        self.writeSubReg(0x01, (mode&0xF), 15, 12)
    def setConvTimes(self, VBus, VShnt, VTemp):
        self.writeSubReg(0x01, (VBus&0b111), 11, 9)
        self.writeSubReg(0x01, (VShnt&0b111), 8, 6)
        self.writeSubReg(0x01, (VTemp&0b111), 5, 3)
    def setAverageCount(self, avg):
        self.writeSubReg(0x01, (avg&0b111), 2, 0)
    def setShuntCalibration(self, Rshunt, maxCurrent):
        multiplier=1
        if self.CurrADCrange==0b1:
            multiplier=4
        self.currentLSB=maxCurrent/(2**15)
        ShuntCal=int((819.2*(10**6))*(self.currentLSB)*Rshunt*multiplier)
        self.writeSubReg(0x02, (ShuntCal&0xFFFF), 14, 0)
    def readShunt(self):
        convConst=0.00000125
        if self.CurrADCrange==0b0:
            convConst=0.000005
        if self.CurrADCrange==0b1:
            convConst=0.00000125
        return self.twosCompConv(self.readReg(0x04), 16)*convConst
    def readBus(self):
        return self.twosCompConv(self.readReg(0x05), 16)*0.003125
    def readDieTemp(self):
        return self.twosCompConv(self.readSubReg(0x06, 15, 4), 12)*0.125
    def readCurrent(self):
        return self.twosCompConv(self.readReg(0x07), 16)*self.currentLSB
    def readError(self):
        errors=[]
        for i in range(15):
            bit=self.readSubReg(0x0B, i, i)
            if bit==0b0 and i==0:
                errors.append(i) #because bit 0 is usually set to 1 unless there is an error
            if bit==0b1 and i!=0:
                errors.append(i)
        return errors

    def clearErrors(self):
        self.writeSubReg(0x0B, 0b0000000, 7, 1)
    def AlertLatch(self, enable):
        if enable:
            self.writeSubReg(0x0B, 0b1, 15, 15)
        else:
            self.writeSubReg(0x0B, 0b0, 15, 15)
    def ConversionAlert(self, enable):
        if enable:
            self.writeSubReg(0x0B, 0b1, 14, 14)
        else:
            self.writeSubReg(0x0B, 0b0, 14, 14)
    def ready(self):
        if self.done in self.readError():
            return True
        else:
            return False
    def setShuntOvervoltage(self, limit):
        if self.CurrADCrange==0b0:
            self.writeReg(0x0C, limit/0.000005)
        elif self.CurrADCrange==0b1:
            self.writeReg(0x0C, limit/0.00000125)
    #still have to add some more limits