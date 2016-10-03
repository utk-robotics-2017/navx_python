from units import *


class ContinuousAngleTracker:
    def __init__(self):
        self.last_angle = Constant(0)
        self.last_rate = Constant(0)
        self.zero_crossing_count = 0
        self.first_sample = True

    def next_angle(self, newAngle):
        '''
            If the first received sample is negative,
            ensure that the zero crossing count is
            decremented.
        '''

        if(self.first_sample):
            self.first_sample = False
            if (newAngle < Constant(0.0)):
                self.zero_crossing_count -= 1

        '''
            Calculate delta angle, adjusting appropriately
            if the current sample crossed the -180/180
            point.
        '''

        bottom_crossing = False
        delta_angle = newAngle - self.last_angle

        # Adjust for wraparound at -180/+180 point
        if (delta_angle >= Angle(180.0, Angle.degree)):
            delta_angle = Angle(360.0, Angle.degree) - delta_angle
            bottom_crossing = True
        elif(delta_angle <= Angle(-180.0, Angle.degree)):
            delta_angle = Angle(360.0, Angle.degree) + delta_angle
            bottom_crossing = True

        self.last_rate = delta_angle

        '''
            If a zero crossing occurred, increment/decrement
            the zero crossing count appropriately.
        '''
        if(not bottom_crossing):
            if(delta_angle < Constant(0.0)):
                if ((newAngle < Constant(0.0)) and (self.last_angle >= Constant(0.0))):
                    self.zero_crossing_count -= 1
            elif(delta_angle >= Constant(0.0)):
                if ((newAngle >= Constant(0.0)) and (self.last_angle < Constant(0.0))):
                    self.zero_crossing_count += 1

        self.last_angle = newAngle

    def get_angle(self):
        accumulated_angle = Angle(self.zero_crossing_count * 360.0, Angle.degree)
        curr_angle = self.last_angle
        if (curr_angle < Constant(0.0)):
            curr_angle += Angle(360.0, Angle.degree)

        accumulated_angle += curr_angle
        return accumulated_angle

    def get_rate(self):
        return self.last_rate
