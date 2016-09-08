import math


class Quaternion:
    def __init__(self, w=0.0, x=0.0, y=0.0, z=0.0):
        self.w = w
        self.x = x
        self.y = y
        self.z = z

    def set(self, w, x, y, z):
        self.w = w
        self.x = x
        self.y = y
        self.z = z

    def setQ(self, q):
        self.w = q.w
        self.x = q.x
        self.y = q.y
        self.z = q.z

    class FloatVectorStruct:
        def __init__(self, x=0.0, y=0.0, z=0.0):
            self.x = x
            self.y = y
            self.z = z

    @staticmethod
    def getGravity(v, q):
        v.x = 2 * (q.x*q.z - q.w*q.y)
        v.y = 2 * (q.w*q.x + q.y*q.z)
        v.z = q.w*q.w - q.x*q.x - q.y*q.y + q.z*q.z

    @staticmethod
    def getYawPitchRollStatic(q, gravity, ypr):
        # yaw: (about Z axis)
        ypr.x = math.atan2(2 * q.x * q.y - 2 * q.w * q.z, 2 * q.w * q.w + 2 * q.x * q.x - 1)

        # pitch: (node up/down, about X axis)
        ypr.y = math.atan(gravity.y / math.sqrt(gravity.x * gravity.x + gravity.z * gravity.z))

        # roll: (tilt left/right, about Y axis)
        ypr.z = math.atan(gravity.x / math.sqrt(gravity.y * gravity.y + gravity.z * gravity.z))

    def getYawPitchRoll(self, ypr):
        gravity = Quaternion.FloatVectorStruct()
        Quaternion.getGravity(ypr, self)
        Quaternion.getYawPitchRollStatic(self, gravity, ypr)

    def getYaw(self):
        ypr = Quaternion.FloatVectorStruct()
        self.getYawPitchRoll(ypr)
        return ypr.x

    def getPitch(self):
        ypr = Quaternion.FloatVectorStruct()
        self.getYawPitchRoll(ypr)
        return ypr.y

    def getRoll(self):
        ypr = Quaternion.FloatVectorStruct()
        self.getYawPitchRoll(ypr)
        return ypr.z

    '''
        Estimates an intermediate Quaternion given Quaternions representing each end of the path,
        and an interpolation ratio from 0.0 t0 1.0.

        Uses Quaternion SLERP (Spherical Linear Interpolation), an algorithm
        originally introduced by Ken Shoemake in the context of quaternion interpolation for the
        purpose of animating 3D rotation. This estimation is based upon the assumption of
        constant-speed motion along a unit-radius great circle arc.

        For more info:

        http://www.euclideanspace.com/maths/algebra/realNormedAlgebra/quaternions/slerp/index.htm
    '''
    @staticmethod
    def slerp(qa, qb, t):
        # quaternion to return
        qm = Quaternion()

        # Calculate angle between them.
        cosHalfTheta = qa.w * qb.w + qa.x * qb.x + qa.y * qb.y + qa.z * qb.z

        # if qa=qb or qa=-qb then theta = 0 and we can return qa
        if (abs(cosHalfTheta) >= 1.0):
            qm.w = qa.w
            qm.x = qa.x
            qm.y = qa.y
            qm.z = qa.z
            return qm

        # Calculate temporary values.
        halfTheta = math.acos(cosHalfTheta)
        sinHalfTheta = math.sqrt(1.0 - cosHalfTheta * cosHalfTheta)

        # if theta = 180 degrees then result is not fully defined
        # we could rotate around any axis normal to qa or qb
        if (abs(sinHalfTheta) < 0.001):
            qm.w = (qa.w * 0.5 + qb.w * 0.5)
            qm.x = (qa.x * 0.5 + qb.x * 0.5)
            qm.y = (qa.y * 0.5 + qb.y * 0.5)
            qm.z = (qa.z * 0.5 + qb.z * 0.5)
            return qm

        ratioA = (math.sin((1 - t) * halfTheta) / sinHalfTheta)
        ratioB = (math.sin(t * halfTheta) / sinHalfTheta)

        # calculate Quaternion.
        qm.w = (qa.w * ratioA + qb.w * ratioB)
        qm.x = (qa.x * ratioA + qb.x * ratioB)
        qm.y = (qa.y * ratioA + qb.y * ratioB)
        qm.z = (qa.z * ratioA + qb.z * ratioB)

        return qm
