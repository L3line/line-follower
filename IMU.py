from Adafruit_GPIO import I2C

class MM7150(I2C.Device):
    def __init__(self):
        busnum = I2C.get_default_bus()
        super(MM7150, self).__init__(0x40, busnum)
        self.DescLen = None
    def _getDesclen(self):
        return 0


imu = MM7150()
print(imu.readS8(0))    
