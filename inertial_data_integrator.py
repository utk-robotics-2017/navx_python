class InertialDataIntegrator:
    def __init__(self):
        self.reset_displacement()

    def update_displacement(self, accel_x_g, accel_y_g, update_rate_hz, is_moving):
        if (is_moving):
            accel_g = [accel_x_g, accel_y_g]
            sample_time = (1.0 / update_rate_hz)

            for i in range(2):
                m_s2 = accel_g[i] * 9.80665
                self.displacement[i] += self.last_velocity[i] + (0.5 * m_s2 * sample_time * sample_time)
                self.last_velocity[i] = self.last_velocity[i] + (m_s2 * sample_time)
        else:
            self.last_velocity[0] = 0.0
            self.last_velocity[1] = 0.0

    def reset_displacement(self):
        self.displacement = [0, 0]
        self.last_velocity = [0, 0]

    def get_velocity_x(self):
        return self.last_velocity[0]

    def get_velocity_y(self):
        return self.last_velocity[1]

    def get_velocity_z(self):
        return 0

    def get_displacement_x(self):
        return self.displacement[0]

    def get_displacement_y(self):
        return self.displacement[1]

    def get_displacement_z(self):
        return 0
