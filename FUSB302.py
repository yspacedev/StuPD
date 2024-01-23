import time
from adafruit_bus_device.i2c_device import I2CDevice
import busio
#YSPACE Labs StuPD library
#thanks to Arya/CRimier on the Hackaday Discord for writing an article and helping with this issue
#https://hackaday.com/2023/02/14/all-about-usb-c-talking-low-level-pd/ https://hackaday.com/2023/02/22/all-about-usb-c-replying-low-level-pd/
#hopefully this is reasonably easy to port. I2C-specific functions will need changing and syntax will change ofc
#otherwise it's just a good resource to learn from since imo the code is not very obfuscated and has enough comments to explain the process
class FUSB302: #minimal class for FUSB302 USB PD PHY
    def __init__(self, i2c, addr):
        self.i2c = i2c
        self.addr = addr
        self.device = I2CDevice(i2c, addr) #adafruit i2cdevice object thing
        #dynamic
        self.PDOS=[] #list of PDOs
        self.startTime=0 #debug for how fast stuff happens
        self.msgID=0 #hold message ID (needs to be incremented every successful message send)
        #constants:
        self.pdo_types = ['fixed', 'batt', 'var', 'pps']
        self.pps_types = ['spr', 'epr', 'res', 'res']
    def writeReg(self, reg, val):
        self.device.write(bytes([reg, val]))

    def readReg(self, reg):
        result=bytearray(1)
        self.device.write_then_readinto(bytes([reg]), result)
        #self.i2c.readfrom_into(self.addr, result)
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
    def reset(self): #reset chip
        self.writeReg(0x0C, 0x01)
    def resetPD(self): #reset PD logic
        self.writeReg(0x0C, 0x02)
    def powerOn(self): #turn on power to chip
        self.writeReg(0x0B, 0x0F)
    def startUp(self): #initialize chip
        self.reset()
        self.powerOn()
        self.writeReg(0x06, 0x00) #unmask interrupts
        self.writeReg(0x09, 0x07) #enable 3 packet retries
    def findPolarity(self): #this part works
        self.writeReg(0x02, 0x07) #ADC connect to CC1
        CC1=self.readSubReg(0x40, 1, 0)
        self.writeReg(0x02, 0x0b) #ADC connect to CC2
        CC2=self.readSubReg(0x40, 1, 0)
        if CC1==CC2:
            return 0 #no PSU or unknown case
        polarity=(1, 2)[CC2>CC1] #compact way of saying that if CC2>CC1, polarity is 2
        return polarity
    def recvMessage(self, length=80):
        read=bytearray(length)
        #self.i2c.writeto(self.addr, bytes([0x43]))
        #self.i2c.readfrom_into(self.addr, read)
        self.device.write_then_readinto(bytes([0x43]), read) #block read
        return read
    def sendHardReset(self):
        self.writeReg(0x09, 0x5F) #hard reset will cause source to turn off and will reset FUSB302
    def parse_pdo(self, pdo):
        pdo_t = self.pdo_types[pdo[3] >> 6]
        if pdo_t == 'fixed':
            current_h = pdo[1] & 0b11
            current_b = ( current_h << 8 ) | pdo[0]
            current = current_b * 10
            voltage_h = pdo[2] & 0b1111
            voltage_b = ( voltage_h << 6 ) | (pdo[1] >> 2)
            voltage = voltage_b * 50
            peak_current = (pdo[2] >> 4) & 0b11
            return (pdo_t, voltage, current, peak_current, pdo[3])
        elif pdo_t in ['batt', 'var']:
            # TODO am not motivated to parse these and they're rare anyways
            return (pdo_t, pdo)
        elif pdo_t == 'pps':
            t = (pdo[3] >> 4) & 0b11
            limited = (pdo[3] >> 5) & 0b1
            max_voltage_h = pdo[3] & 0b1
            max_voltage_b = (max_voltage_h << 7) | pdo[2] >> 1
            max_voltage = max_voltage_b * 100
            min_voltage = pdo[1] * 100
            max_current_b = pdo[0] & 0b1111111
            max_current = max_current_b * 50
            return ('pps', self.pps_types[t], max_voltage, min_voltage, max_current, limited)
    def request_pdo(self, num, current, max_current, msg_id=0):
        sop_seq = [0x12, 0x12, 0x12, 0x13, 0x80] #These act as tokens to tell the FUSB302 what data to send. The packet itself has a certain encoding. See the FUSB302 datasheet for mor info on the tokens
        eop_seq = [0xff, 0x14, 0xfe, 0xa1]
        obj_count = 1
        pdo_len = 2 + (4*obj_count)
        pdo = [0 for i in range(pdo_len)]

        pdo[0] |= 0b10 << 6 # PD 3.0
        pdo[0] |= 0b00010 # request

        pdo[1] |= obj_count << 4
        pdo[1] |= (msg_id&0b111) << 1 #INCREMENT EVERY TIME MESSAGE SUCCEEDS otherwise source will not change voltage

        # packing max current into fields
        max_current_b = max_current // 10
        max_current_l = max_current_b & 0xff
        max_current_h = max_current_b >> 8
        pdo[2] = max_current_l
        pdo[3] |= max_current_h

        # packing current into fields
        current_b = current // 10
        current_l = current_b & 0x3f
        current_h = current_b >> 6
        pdo[3] |= current_l << 2
        pdo[4] |= current_h

        pdo[5] |= (num+1) << 4 # object position
        pdo[5] |= 0b1 # no suspend
        sop_seq[4] |= pdo_len
        #self.writeReg(0x06, 0x40) #flush TX buffer
        #NOTE: this part can cause an IO error if it is executed too quickly after another I2C action. Add some delay so the previous action completes.
        self.device.write(bytes([0x43]+sop_seq)) #yeet stuff into FIFO
        self.device.write(bytes([0x43]+pdo))
        self.device.write(bytes([0x43]+eop_seq))
        #not exactly sure why 3 seperate block writes (possibly to minimize latency by avoiding array concatenation), but that's the way the example was written and it works
        #print(time.monotonic()-self.startTime) #debug for printing time
        #self.writeReg(0x06, 0x01) #start TX?
        #self.i2c.writeto(self.addr, bytes([0x43]))
        #self.i2c.writeto(self.addr, bytes(pdo))
        #self.i2c.writeto(self.addr, bytes([0x43]))
        #self.i2c.writeto(self.addr, bytes(eop_seq)) 1000000

    def readPDOS(self):
        pdo_list = []
        #msg = self.recvMessage()
        #print(msg)
        header = self.recvMessage(1)[0]
        assert(header == 0xe0)
        b1, b0 = self.recvMessage(2) #bits that encode message length
        pdo_count = (b0 >> 4) & 0b111
        read_len = pdo_count*4
        pdos = self.recvMessage(read_len) #read part of message that encodes PDOs
        _ = self.recvMessage(4) #clear CRC
        for pdo_i in range(pdo_count):
            pdo_bytes = pdos[(pdo_i*4):][:4] #extract individual pdo
            parsed_pdo = self.parse_pdo(pdo_bytes) #parse
            pdo_list.append(parsed_pdo)
        return pdo_list
    def getAcceptMessage(self):
        #read buffer until 0xe0 reached
        maxReadLen = 80 #maximum number of bits to read to search for accept message. Source will send GoodCRC then accept message but accept may come after 
        for i in range(maxReadLen):
            if self.recvMessage(1)[0]==0xe0: #packet header
                b0, b1 = self.recvMessage(2)
                _ = self.recvMessage(4) #clear CRC since we don't need it
                msgID=b0&0b1111 #only bits 0-4
                if msgID == 0x03: #accept message ID
                    return True
        return False
    def getProfiles(self, CC):
        #prepare things and clean buffers
        self.writeReg(0x06, 0x40) #flush TX
        self.writeReg(0x07, 0x04) #flush RX
        self.resetPD() #reset PD logic
        bits1=(0x25,0x26)[CC-1] #first one is TX on CC1. I might be doing this wrong. LSB or MSB?
        self.writeReg(0x03, bits1) #SWITCHES1 - set as sink, rev 2.0, TX driver on CC pin
        bits2=(0x07,0x0B)[CC-1]
        #start TX, decode message, send message to stay at 5v, return power levels
        self.writeReg(0x02, bits2) #connect MEAS to CC pin
        self.startTime=time.monotonic()
        retries=0
        while (not self.readSubReg(0x41, 5, 5)==0b0): #wait until something is in recv buffer
            if time.monotonic()-self.startTime>=1 and retries<1: #if it hasn't received data after 1 second, send hard reset
                #print("sending hard reset")
                self.sendHardReset() #hard reset will disconnect power and also reset the FUSB302. Maybe there's a better solution (soft reset?)
                self.startUp() #so we need to start everything again (in the case that there's external power keeping everything on after the hard reset)
                self.writeReg(0x03, bits1)
                self.writeReg(0x02, bits2)
                self.msgID=0
                retries+=1 #don't spam hard reset
            if time.monotonic()-self.startTime>=3:
                return False
        self.startTime=time.monotonic()
        self.PDOS=self.readPDOS()
        #print(self.PDOS)
        return not (self.PDOS == None)

    def requestVoltage(self, voltage, maxcurrent):
        for i, pdo in enumerate(self.PDOS):
            if pdo[0] == 'fixed':
                pdo_voltage=pdo[1]
                pdo_current=pdo[2]
                if pdo_voltage==(voltage*1000) and pdo_current>=maxcurrent:
                    self.startTime=time.monotonic() #for debug timer to track delay in code in case it times out
                    self.request_pdo(i, maxcurrent, maxcurrent, msg_id=self.msgID)
                    self.startTime=time.monotonic()
                    while (not self.readSubReg(0x41, 5, 5)==0b0):
                        if time.monotonic()-self.startTime>=1:
                            return False #return false if timeout
                    self.msgID+=1 #increment message ID after sucessful message send
                    if self.msgID>7: #clamp
                        self.msgID=0
                    #print(self.recvMessage(80)) #print recvd message for debug
                    #TODO: if header of message somewhere has 0xa3 (0xX3) as the second byte, that indicates accept. For example, 0xE0, 0xA3, 0x03
                    if self.getAcceptMessage():
                        return True
                    self.writeReg(0x07, 0x04) #flush RX
                    return False #message received but no accept.
            elif pdo[0]=='pps':
                pass #TODO
        return False
    def autoRequest(self, R, maxPower): #algorithm for finding maximum power for given resistive load and fixed power limit
        selectedInd=-1
        greatestPower=0 #highest power found so far
        for i, pdo in enumerate(self.PDOS):
            if pdo[0] == 'fixed':
                pdo_voltage=pdo[1]/1000
                pdo_current=pdo[2]/1000
                powerSup=pdo_voltage*pdo_current
                powerLoad=(pdo_voltage**2)/R
                if powerSup>=powerLoad and powerLoad>greatestPower and powerLoad<=maxPower:
                    greatestPower=powerLoad
                    selectedInd=i
        if selectedInd>-1:
            maxcurrent=self.PDOS[selectedInd][2]
            self.startTime=time.monotonic() #for debug timer
            self.request_pdo(selectedInd, maxcurrent, maxcurrent, msg_id=self.msgID)
            self.startTime=time.monotonic()
            while (not self.readSubReg(0x41, 5, 5)==0b0):
                if time.monotonic()-self.startTime>=1:
                    return False #return false if timeout
            self.msgID+=1 #increment message ID after sucessful message send
            if self.msgID>7: #clamp
                self.msgID=0
            #print(self.recvMessage(80)) #print recvd message for debug
            if self.getAcceptMessage():
                return True
            self.writeReg(0x07, 0x04) #flush RX
            return False #message received, but no accept
        else:
            return False