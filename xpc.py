import socket
import struct

class XPlaneConnect(object):
    """XPlaneConnect (XPC) facilitates communication to and from the XPCPlugin."""
    socket = None

    def __init__(self, xpHost='localhost', xpPort=49009, port=0, timeout=5000):
        """Sets up a new connection to an X-Plane Connect plugin running in X-Plane."""
        try:
            xpIP = socket.gethostbyname(xpHost)
        except:
            raise ValueError("Unable to resolve xpHost.")

        if xpPort < 0 or xpPort > 65535:
            raise ValueError("The specified X-Plane port is not a valid port number.")
        if port < 0 or port > 65535:
            raise ValueError("The specified port is not a valid port number.")
        if timeout < 0:
            raise ValueError("timeout must be non-negative.")

        self.xpDst = (xpIP, xpPort)
        clientAddr = ("0.0.0.0", port)
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
        self.socket.bind(clientAddr)
        timeout /= 1000.0
        self.socket.settimeout(timeout)

    def __del__(self):
        self.close()

    def __enter__(self):
        return self

    def __exit__(self, type, value, traceback):
        self.close()

    def close(self):
        """Closes the specified connection and releases resources associated with it."""
        if self.socket is not None:
            try:
                self.socket.close()
            except Exception as e:
                print(f"[ERROR] Failed to close socket: {e}")
            self.socket = None

    def sendUDP(self, buffer):
        """Sends a message over the underlying UDP socket."""
        if len(buffer) == 0:
            raise ValueError("sendUDP: buffer is empty.")
        try:
            self.socket.sendto(buffer, 0, self.xpDst)
        except Exception as e:
            print(f"[ERROR] sendUDP failed: {e}")
            raise

    def readUDP(self):
        """Reads a message from the underlying UDP socket."""
        try:
            return self.socket.recv(16384)
        except socket.timeout:
            print("[WARNING] readUDP timed out")
            raise
        except Exception as e:
            print(f"[ERROR] readUDP failed: {e}")
            raise

    def setCONN(self, port):
        """Sets the port on which the client sends and receives data."""
        if port < 0 or port > 65535:
            raise ValueError("The specified port is not a valid port number.")

        buffer = struct.pack(b"<4sxH", b"CONN", port)
        self.sendUDP(buffer)
        clientAddr = ("0.0.0.0", port)
        timeout = self.socket.gettimeout()
        self.socket.close()
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
        self.socket.bind(clientAddr)
        self.socket.settimeout(timeout)
        try:
            buffer = self.socket.recv(1024)
        except Exception as e:
            print(f"[ERROR] setCONN failed: {e}")
            raise

    def pauseSim(self, pause):
        """Pauses or un-pauses the physics simulation engine in X-Plane."""
        pause = int(pause)
        if pause < 0 or pause > 2:
            raise ValueError("Invalid argument for pause command.")
        buffer = struct.pack(b"<4sxB", b"SIMU", pause)
        self.sendUDP(buffer)

    def readDATA(self):
        """Reads X-Plane data."""
        try:
            buffer = self.readUDP()
            if len(buffer) < 6:
                print("[WARNING] readDATA: Empty or incomplete response")
                return None
            rows = (len(buffer) - 5) // 36
            data = []
            for i in range(rows):
                try:
                    data.append(struct.unpack_from(b"9f", buffer, 5 + 36*i))
                except Exception as e:
                    print(f"[ERROR] readDATA failed at row {i}: {e}")
                    data.append([])
            return data
        except Exception as e:
            print(f"[ERROR] readDATA failed: {e}")
            return None

    def sendDATA(self, data):
        """Sends X-Plane data over the underlying UDP socket."""
        if len(data) > 134:
            raise ValueError("Too many rows in data.")
        buffer = struct.pack(b"<4sx", b"DATA")
        for row in data:
            if len(row) != 9:
                raise ValueError("Row does not contain exactly 9 values: " + str(row))
            buffer += struct.pack(b"<I8f", *row)
        self.sendUDP(buffer)

    def getPOSI(self, ac=0):
        """Gets position information for the specified aircraft."""
        buffer = struct.pack(b"<4sxB", b"GETP", ac)
        self.sendUDP(buffer)
        try:
            resultBuf = self.readUDP()
            print(f"[DEBUG] getPOSI received buffer size: {len(resultBuf)} bytes")
            if len(resultBuf) == 34:
                result = struct.unpack(b"<4sxBfffffff", resultBuf)
            elif len(resultBuf) == 46:
                result = struct.unpack(b"<4sxBdddffff", resultBuf)
            else:
                print(f"[WARNING] getPOSI: Unexpected response length: {len(resultBuf)} bytes")
                return None
            if result[0] != b"POSI":
                print(f"[WARNING] getPOSI: Unexpected header: {result[0]}")
                return None
            return result[2:]
        except socket.timeout:
            print("[WARNING] getPOSI timed out")
            return None
        except Exception as e:
            print(f"[ERROR] getPOSI failed: {e}")
            return None

    def sendPOSI(self, values, ac=0):
        """Sets position information on the specified aircraft."""
        if len(values) < 1 or len(values) > 7:
            raise ValueError("Must have between 0 and 7 items in values.")
        if ac < 0 or ac > 20:
            raise ValueError("Aircraft number must be between 0 and 20.")
        buffer = struct.pack(b"<4sxB", b"POSI", ac)
        for i in range(7):
            val = -998
            if i < len(values):
                val = values[i]
            if i < 3:
                buffer += struct.pack(b"<d", val)
            else:
                buffer += struct.pack(b"<f", val)
        self.sendUDP(buffer)

    def getCTRL(self, ac=0):
        """Gets the control surface information for the specified aircraft."""
        buffer = struct.pack(b"<4sxB", b"GETC", ac)
        self.sendUDP(buffer)
        try:
            resultBuf = self.readUDP()
            if len(resultBuf) != 31:
                print(f"[WARNING] getCTRL: Unexpected response length: {len(resultBuf)} bytes")
                return None
            result = struct.unpack(b"<4sxffffbfBf", resultBuf)
            if result[0] != b"CTRL":
                print(f"[WARNING] getCTRL: Unexpected header: {result[0]}")
                return None
            return result[1:7] + result[8:]
        except Exception as e:
            print(f"[ERROR] getCTRL failed: {e}")
            return None

    def sendCTRL(self, values, ac=0):
        """Sets control surface information on the specified aircraft."""
        if len(values) < 1 or len(values) > 7:
            raise ValueError("Must have between 0 and 7 items in values.")
        if ac < 0 or ac > 20:
            raise ValueError("Aircraft number must be between 0 and 20.")
        buffer = struct.pack(b"<4sx", b"CTRL")
        for i in range(6):
            val = -998
            if i < len(values):
                val = values[i]
            if i == 4:
                val = -1 if (abs(val + 998) < 1e-4) else val
                buffer += struct.pack(b"b", int(val))
            else:
                buffer += struct.pack(b"<f", val)
        buffer += struct.pack(b"B", ac)
        if len(values) == 7:
            buffer += struct.pack(b"<f", values[6])
        self.sendUDP(buffer)

    def sendDREF(self, dref, values):
        """Sets the specified dataref to the specified value."""
        self.sendDREFs([dref], [values])

    def sendDREFs(self, drefs, values):
        """Sets the specified datarefs to the specified values."""
        if len(drefs) != len(values):
            raise ValueError("drefs and values must have the same number of elements.")
        buffer = struct.pack(b"<4sx", b"DREF")
        for i in range(len(drefs)):
            dref = drefs[i]
            value = values[i]
            if len(dref) == 0 or len(dref) > 255:
                raise ValueError("dref must be a non-empty string less than 256 characters.")
            if value is None:
                raise ValueError("value must be a scalar or sequence of floats.")
            if hasattr(value, "__len__"):
                if len(value) > 255:
                    raise ValueError("value must have less than 256 items.")
                fmt = "<B{0:d}sB{1:d}f".format(len(dref), len(value))
                buffer += struct.pack(fmt.encode(), len(dref), dref.encode(), len(value), *value)
            else:
                fmt = "<B{0:d}sBf".format(len(dref))
                buffer += struct.pack(fmt.encode(), len(dref), dref.encode(), 1, value)
        self.sendUDP(buffer)

    def getDREF(self, dref):
        """Gets the value of an X-Plane dataref."""
        return self.getDREFs([dref])[0]

    def getDREFs(self, drefs):
        """Gets the value of one or more X-Plane datarefs."""
        buffer = struct.pack(b"<4sxB", b"GETD", len(drefs))
        for dref in drefs:
            fmt = "<B{0:d}s".format(len(dref))
            buffer += struct.pack(fmt.encode(), len(dref), dref.encode())
        self.sendUDP(buffer)
        try:
            buffer = self.readUDP()
            # print(f"[DEBUG] getDREFs received buffer size: {len(buffer)} bytes")
            if len(buffer) < 6:
                print("[WARNING] getDREFs: Empty or incomplete response")
                return [[] for _ in drefs]
            resultCount = struct.unpack_from(b"B", buffer, 5)[0]
            offset = 6
            result = []
            for i in range(resultCount):
                rowLen = struct.unpack_from(b"B", buffer, offset)[0]
                offset += 1
                if offset + rowLen * 4 > len(buffer):
                    print(f"[WARNING] getDREFs: Insufficient buffer size for row {i}")
                    result.append([])
                    continue
                fmt = "<{0:d}f".format(rowLen)
                row = struct.unpack_from(fmt.encode(), buffer, offset)
                result.append(row)
                offset += rowLen * 4
            return result
        except socket.timeout:
            print("[WARNING] getDREFs timed out")
            return [[] for _ in drefs]
        except Exception as e:
            print(f"[ERROR] getDREFs failed: {e}")
            return [[] for _ in drefs]

    def sendTEXT(self, msg, x=-1, y=-1):
        """Sets a message that X-Plane will display on the screen."""
        if y < -1:
            raise ValueError("y must be greater than or equal to -1.")
        if msg is None:
            msg = ""
        msgLen = len(msg)
        buffer = struct.pack(b"<4sxiiB" + (str(msgLen) + "s").encode(), b"TEXT", x, y, msgLen, msg.encode())
        self.sendUDP(buffer)

    def sendVIEW(self, view):
        """Sets the camera view in X-Plane."""
        if view < ViewType.Forwards or view > ViewType.FullscreenNoHud:
            raise ValueError("Unknown view command.")
        buffer = struct.pack(b"<4sxi", b"VIEW", view)
        self.sendUDP(buffer)

    def sendWYPT(self, op, points):
        """Adds, removes, or clears waypoints."""
        if op < 1 or op > 3:
            raise ValueError("Invalid operation specified.")
        if len(points) % 3 != 0:
            raise ValueError("Invalid points. Points should be divisible by 3.")
        if len(points) / 3 > 255:
            raise ValueError("Too many points. You can only send 255 points at a time.")
        if op == 3:
            buffer = struct.pack(b"<4sxBB", b"WYPT", 3, 0)
        else:
            buffer = struct.pack(("<4sxBB" + str(len(points)) + "f").encode(), b"WYPT", op, len(points), *points)
        self.sendUDP(buffer)

class ViewType(object):
    Forwards = 73
    Down = 74
    Left = 75
    Right = 76
    Back = 77
    Tower = 78
    Runway = 79
    Chase = 80
    Follow = 81
    FollowWithPanel = 82
    Spot = 83
    FullscreenWithHud = 84
    FullscreenNoHud = 85
