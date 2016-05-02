class MagCalData:
    def __init__(self):
        self.action = 0
        self.mag_bias = [0, 0, 0]
        self.mag_xform = [[0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]]
        self.earth_mag_field_norm = 0.0
