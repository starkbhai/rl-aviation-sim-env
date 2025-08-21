from xpc import XPlaneConnect
#192.168.78.222
#10.19.182.67
class XPlaneClient:
    def __init__(self, ip="10.19.182.67", port=49009):
        self.client = XPlaneConnect(ip, port)

    def sendCTRL(self, controls):
        self.client.sendCTRL(controls)

    def getPOSI(self):
        return self.client.getPOSI()

    def getDREF(self, ref):
        return self.client.getDREF(ref)

    def getDREFs(self, refs):
        return self.client.getDREFs(refs)

    def sendDATA(self, data):
        self.client.sendDATA(data)

    def sendDREF(self, dataref, value):
        self.client.sendDREF(dataref, value)

    def setDREF(self, dataref, value):
        # alias for sendDREF, so existing code using setDREF won't break
        self.sendDREF(dataref, value)

    def sendCOMM(self, command):  # ✅ NEW: For sending X-Plane commands
        self.client.sendCOMM(command)

    def sendPOSI(self, pos):  # ✅ ADD THIS METHOD
        self.client.sendPOSI(pos)
        
    def sendVEL(self, vel):  # ✅ NEW
        self.client.sendVEL(vel)

    def close(self):
        self.client.close()
