import math


class Unit:
    def __init__(self, value, unit):
        self.base_value = value * unit

    def to(self, unit):
        return self.base_value / unit

    def __add__(self, other):
        return Unit(self.base_value + other.base_value, 1)

    def __sub__(self, other):
        return Unit(self.base_value - other.base_value, 1)

    def __mul__(self, other):
        return Unit(self.base_value * other.base_value, 1)

    def __truediv__(self, other):
        return Unit(self.base_value / other.base_value, 1)

    def __iadd__(self, other):
        return Unit(self.base_value + other.base_value, 1)

    def __isub__(self, other):
        return Unit(self.base_value - other.base_value, 1)

    def __imul__(self, other):
        return Unit(self.base_value * other.base_value, 1)

    def __itruediv__(self, other):
        return Unit(self.base_value / other.base_value)

    def __neg__(self):
        return Unit(-1 * self.base_value, 1)

    def __pos__(self):
        return Unit(self.base_value, 1)

    def __abs__(self):
        return Unit(abs(self.base_value), 1)

    def __lt__(self, other):
        return self.base_value < other.base_value

    def __le__(self, other):
        return self.base_value <= other.base_value

    def __eq__(self, other):
        return self.base_value == other.base_value

    def __ne__(self, other):
        return self.base_value != other.base_value

    def __gt__(self, other):
        return self.base_value > other.base_value

    def __ge__(self, other):
        return self.base_value >= other.base_value


class Constant(Unit):
    def __init__(self, value):
        Unit.__init__(self, value, 1.0)


class Length(Unit):
    def __init__(self, value, unit):
        Unit.__init__(self, value, unit)
    m = 1.0
    mm = m * .001
    cm = m * .01
    km = m * 1000.0

    inch = m * 39.3701
    ft = inch * 12.0

Distance = Length


class Angle(Unit):
    def __init__(self, value, unit):
        Unit.__init__(self, value, unit)
    degree = 1.0
    radian = degree * 180.0 / math.pi
    rev = degree * 360.0
    quaternion = radian * math.pi
    Q = quaternion


class Time(Unit):
    def __init__(self, value, unit):
        Unit.__init__(self, value, unit)
    s = 1.0
    ms = s * .001
    minute = s * 60.0
    hr = minute * 60.0


class Velocity(Unit):
    def __init__(self, value, unit):
        Unit.__init__(self, value, unit)
    m_s = Length.m / Time.s
    m_minute = Length.m / Time.minute

    mm_s = Length.mm / Time.s
    mm_minute = Length.mm / Time.minute

    cm_s = Length.cm / Time.s
    cm_minute = Length.cm / Time.minute

    inch_s = Length.inch / Time.s
    inch_minute = Length.inch / Time.minute

    ft_s = Length.ft / Time.s
    ft_minute = Length.ft / Time.minute

Speed = Velocity


class AngularVelocity(Unit):
    def __init__(self, value, unit):
        Unit.__init__(self, value, unit)
    rpm = Angle.rev / Time.s
    rps = Angle.rev / Time.s
    rad_s = Angle.radian / Time.s
    deg_s = Angle.degree / Time.s


class Acceleration(Unit):
    def __init__(self, value, unit):
        Unit.__init__(self, value, unit)
    m_s2 = Length.m / Time.s ** 2
    m_minute2 = Length.m / Time.minute ** 2

    mm_s2 = Length.mm / Time.s ** 2
    mm_minute = Length.mm / Time.minute ** 2

    cm_s2 = Length.cm / Time.s ** 2
    cm_minute2 = Length.cm / Time.minute ** 2

    inch_s2 = Length.inch / Time.s ** 2
    inch_minute2 = Length.inch / Time.minute ** 2

    ft_s2 = Length.ft / Time.s ** 2
    ft_minute2 = Length.ft / Time.minute ** 2

    G = m_s2 * 9.81
    g = G


class Force(Unit):
    def __init__(self, value, unit):
        Unit.__init__(self, value, unit)
    N = 1
    oz = N * 3.59694309
    lbs = oz / 16


class Pressue(Unit):
    def __init__(self, value, unit):
        Unit.__init__(self, value, unit)
    Pa = Force.N / (Distance.m ** 2)
    kPa = Pa * 1000
    atm = Pa * 9.86923e-6
    mb = Pa * 0.01
    Hg = Pa * 3386.38866667


class Torque(Unit):
    def __init__(self, value, unit):
        Unit.__init__(self, value, unit)
    Nm = Force.N * Distance.m
    ozinch = Force.oz * Distance.inch


class Current(Unit):
    def __init__(self, value, unit):
        Unit.__init__(self, value, unit)
    A = 1


class Voltage(Unit):
    def __init__(self, value, unit):
        Unit.__init__(self, value, unit)
    v = 1


class Temperature(Unit):
    def __init__(self, value, unit):
        if unit == self.C:
            self.base_value = value
        elif unit == self.K:
            self.base_value = value - 273.15
        elif unit == self.F:
            self.base_value = (value - 32) * (5 / 9)

    def to(self, unit):
        if unit == self.C:
            return self.base_value
        elif unit == self.K:
            return self.base_value + 273.15
        elif unit == self.F:
            return (self.base_value * (9 / 5)) + 32

    C = 1
    K = 2
    F = 3
