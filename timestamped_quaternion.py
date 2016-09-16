from quaternion import Quaternion


class TimestampedQuaternion(Quaternion):
    def __init__(self, src=None, timestamp=None):
        if src is not None:
            Quaternion.__init__(self, src.w, src.x, src.y, src.z)
        else:
            Quaternion.__init__(self)
        self.timestamp = timestamp
        self.valid = True
        self.interpolated = False

    def isValid(self):
        return self.valid

    def getTimestamp(self):
        return self.timestamp

    def set(self, w, x, y, z, timestamp):
        self.set(w, x, y, z)
        self.timestamp = timestamp
        self.valid = True

    def setQ(self, src, timestamp):
        Quaternion.setQ(src)
        self.timestamp = timestamp
        self.valid = True

    def getInterpolated(self):
        return self.interpolated

    def setInterpolated(self, interpolated):
        self.interpolated = interpolated
