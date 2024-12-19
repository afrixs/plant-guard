import time

class SpiDev:
    def __init__(self, bus, device):
        pass

    def writebytes(self, data):
        pass

    def readbytes(self, reg):
        time.sleep(0.01)
        return [0]*reg
        pass