import digitalio
import board
import time
import busio
import FUSB302

i2c = busio.I2C(board.GP7, board.GP6) #change to I2C on your board

PD=FUSB302.FUSB302(i2c, 0x23)

def PDstuff():
    PD.relock()
    PD.reset()
    PD.startUp()
    CCpin=PD.findPolarity()
    if CCpin == 0:
        return False #no PD
    if not PD.getProfiles(CCpin):
        return False #source didn't respond so no PD
    return PD.requestVoltage(5, 1000) #request default 5v

print(PDstuff())
time.sleep(5)
PD.writeReg(0x07, 0x04) #flush RX since PSU sends a PS_RDY shortly after change voltage is requested
print("changing voltage")
print(PD.autoRequest(9, 9)) #9W to 9 ohms should request 9V
#print(PD.requestVoltage(9, 1000))
PD.unlock()
