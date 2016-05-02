class InertialDataIntegrator:
    def __init__(self):
        self.last_velocity = [0.0, 0.0]
        self.displacement = [0.0, 0.0]

    def updateDisplacement(self, accel_x_g, accel_y_g, update_rate_hz, is_moving):
        if (is_moving):
            accel_g = [0.0, 0.0]
            accel_m_s2 = [0.0, 0.0]
            curr_velocity_m_s = [0.0, 0.0]
            sample_time = (1.0 / update_rate_hz)
            accel_g[0] = accel_x_g
            accel_g[1] = accel_y_g
            for i in range(2):
                accel_m_s2[i] = accel_g[i] * 9.80665
                curr_velocity_m_s[i] = self.last_velocity[i] + (accel_m_s2[i] * sample_time)
                self.displacement[i] += self.last_velocity[i] + (0.5 * accel_m_s2[i] * sample_time * sample_time)
                self.last_velocity[i] = curr_velocity_m_s[i]
        else:
            self.last_velocity[0] = 0.0
            self.last_velocity[1] = 0.0

    def resetDisplacement(self):
        for i in range(2):
            self.last_velocity[i] = 0.0
            self.displacement[i] = 0.0

    def getVelocityX(self):
        return self.last_velocity[0]

    def getVelocityY(self):
        return self.last_velocity[1]

    def getVelocityZ(self):
        return 0

    def getDisplacementX(self):
        return self.displacement[0]

    def getDisplacementY(self):
        return self.displacement[1]

    def getDisplacementZ(self):
        return 0
