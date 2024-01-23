import digitalio
import board
import time
import busio
import displayio
import terminalio
from adafruit_display_text import label
import adafruit_displayio_ssd1306
print("hi")
displayio.release_displays()
i2c = busio.I2C(board.GP7, board.GP6)
print("2")
#while not i2c.try_lock():
#    pass
class INA237:
    def __init__(self, i2c, addr):
        self.i2c = i2c
        self.addr = addr
        self.CurrADCrange=0b0
        #constants:
        self.range_small=0b1 #40.96mV
        self.range_large=0b0 #163.84mV

        #error codes
        self.done=13 #by bit
    def writeReg(self, reg, val):
        MSB = val >> 8
        LSB = ((val << 8) & 0xFF00) >> 8
        self.i2c.writeto(self.addr, bytes([reg, MSB, LSB]))

    def readReg(self, reg):
        result=bytearray(2)
        self.i2c.writeto(self.addr, bytes([reg]))
        self.i2c.readfrom_into(self.addr, result)
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
        self.writeSubReg(0x01, (delay&0b111), 2, 0)
    def setShuntCalibration(self, Rshunt, maxCurrent):
        multiplier=1
        if self.CurrADCrange==0b1:
            multiplier=4
        ShuntCal=(819.2*(10**6))*(maxCurrent/(2**15))*Rshunt*multiplier
        self.writeSubReg(0x02, (delay&0xFFFF), 14, 0)
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
        return self.twosCompConv(self.readReg(0x07), 16)*0.00125
    def readError(self):
        errors=[]
        for i in range(15):
            bit=self.readSubReg(0x0B, i, i)
            if bit==0b0 and i==0:
                errors.append(i)
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
    #still have to add some more stuff

print("3")
display_bus = displayio.I2CDisplay(i2c, device_address=0x3C)
display = adafruit_displayio_ssd1306.SSD1306(display_bus, width=128, height=64)
splash=displayio.Group()
display.show(splash)
temp=25.125
Text_area = label.Label(terminalio.FONT, text=(str(temp)+"C"), color=0xFFFF00, x=0, y=7)
splash.append(Text_area)
Text_area.text="text"
print(4)
CS=INA237(i2c, 0x40)
#CS.relock()
#print(CS.readBus())
#print(CS.readDieTemp())
#CS.unlock()

while True:
    #displayio.release_displays()
    CS.relock()
    temp=str(CS.readBus())
    print(temp)
    print(CS.readBus())
    CS.unlock()
    Text_area.text=temp
    time.sleep(0.5)

