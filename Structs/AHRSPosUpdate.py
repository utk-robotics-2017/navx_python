class AHRSPosUpdate:
    def __init__(self):
        self.yaw = 0.0
        self.pitch = 0.0
        self.roll = 0.0
        self.compass_heading = 0.0
        self.altitude = 0.0
        self.fused_heading = 0.0
        self.linear_accel_x = 0.0
        self.linear_accel_y = 0.0
        self.linear_accel_z = 0.0
        self.vel_x = 0.0
        self.vel_y = 0.0
        self.vel_z = 0.0
        self.disp_x = 0.0
        self.disp_y = 0.0
        self.disp_z = 0.0
        self.mpu_temp = 0.0
        self.quat_w = 0
        self.quat_x = 0
        self.quat_y = 0
        self.quat_z = 0
        self.barometric_pressure = 0.0
        self.baro_temp = 0.0
        self.op_status = 0
        self.sensor_status = 0
        self.cal_status = 0
        self.selftest_status = 0