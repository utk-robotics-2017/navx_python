class BoardState:
    def __init__(self):
        self.op_status = 0
        self.sensor_status = 0
        self.cal_status = 0
        self.selftest_status = 0
        self.capability_flags = 0
        self.update_rate_hz = 0
        self.accel_fsr_g = 0
        self.gyro_fsr_dps = 0
