import time

import Adafruit_GPIO.I2C as I2C

from Structs.AHRSPosUpdate import AHRSPosUpdate
from Structs.AHRSUpdate import AHRSUpdate
from Structs.GyroUpdate import GyroUpdate
from Structs.BoardID import BoardID
from Structs.BoardState import BoardState

from IMURegisters import IMURegisters
from AHRSProtocol import AHRSProtocol


class I2C_IO:
    def __init__(self, ahrs, update_rate_hz = 4, busnum=None, i2c_interface=None, **kwargs):
        self.i2c = I2C.get_i2c_device(0x32, busnum, i2c_interface, **kwargs)

        self.update_rate_hz = update_rate_hz
        self.stop = False

        self.ahrs = ahrs
        self.raw_data_update = GyroUpdate()
        self.ahrs_update = AHRSUpdate()
        self.ahrspos_update = AHRSPosUpdate()
        self.board_id = BoardID()
        self.board_state = BoardState()

        from AHRS import BoardCapabilities
        self.board_capabilities = BoardCapabilities()

        self.last_update_time = 0.0
        self.byte_count = 0
        self.update_count = 0
        self.last_sensor_timestamp = 0

    def stop(self):
        self.stop = True

    def run(self):
        #self.i2c.init()

        # Initial Device Configuration
        self.setUpdateRateHz(self.update_rate_hz)

        self.getConfiguration()

        '''
            Calculate delay to match configured update rate
            Note:  some additional time is removed from the
            1/update_rate value to ensure samples are not
            dropped, esp. at higher update rates.
        '''
        update_rate = 1.0 / (float)(self.update_rate_hz & 0xFF)
        if(update_rate > DELAY_OVERHEAD_SECONDS):
            update_rate -= DELAY_OVERHEAD_SECONDS

        # IO Loop
        while not stop:
            if not self.board_state.update_rate_hz == self.update_rate_hz:
                self.setUpdateRateHz(self.update_rate_hz)
            self.getCurrentData()
            time.sleep(update_rate)

    def getConfiguration(self):
        success = False
        retry_count = 0
        while retry_count < 3 and not success:
            config = self.read(IMURegisters.NAVX_REG_WHOAMI, IMURegisters.NAVX_REG_SENSOR_STATUS_H + 1)
            print config
            success = len(config) > 0

            if success:
                self.board_id.hw_rev                 = config[IMURegisters.NAVX_REG_HW_REV];
                self.board_id.fw_ver_major           = config[IMURegisters.NAVX_REG_FW_VER_MAJOR];
                self.board_id.fw_ver_minor           = config[IMURegisters.NAVX_REG_FW_VER_MINOR];
                self.board_id.type                   = config[IMURegisters.NAVX_REG_WHOAMI];
                self.ahrs.setBoardID(self.board_id);

                self.board_state.cal_status          = config[IMURegisters.NAVX_REG_CAL_STATUS];
                self.board_state.op_status           = config[IMURegisters.NAVX_REG_OP_STATUS];
                self.board_state.selftest_status     = config[IMURegisters.NAVX_REG_SELFTEST_STATUS];
                self.board_state.sensor_status       = AHRSProtocol.decodeBinaryUint16(config, IMURegisters.NAVX_REG_SENSOR_STATUS_L);
                self.board_state.gyro_fsr_dps        = AHRSProtocol.decodeBinaryUint16(config, IMURegisters.NAVX_REG_GYRO_FSR_DPS_L);
                self.board_state.accel_fsr_g         = config[IMURegisters.NAVX_REG_ACCEL_FSR_G];
                self.board_state.update_rate_hz      = config[IMURegisters.NAVX_REG_UPDATE_RATE_HZ];
                self.board_state.capability_flags    = AHRSProtocol.decodeBinaryUint16(config, IMURegisters.NAVX_REG_CAPABILITY_FLAGS_L);
                self.ahrs.setBoardState(self.board_state);
            else:
                time.sleep(0.05)
            retry_count += 1;
        return success

    def getCurrentData(self):
        first_address = IMURegisters.NAVX_REG_UPDATE_RATE_HZ
        displacement_registers = self.board_capabilities.isDisplacementSupported()
        # If firmware supports displacement data, acquire it - otherwise implement
        # similar (but potentially less accurate) calculations on this processor.
        if displacement_registers:
            success, current_data = read(first_address, IMURegisters.NAVX_REG_LAST + 1 - first_address)
        else:
            success, current_data = read(first_address, IMURegisters.NAVX_REG_QUAT_OFFSET_Z_H + 1 - first_address)

        if success:
            timestamp_low = AHRSProtocol.decodeBinaryUint16(current_data, IMURegisters.NAVX_REG_TIMESTAMP_L_L - first_address)
            timestamp_high = AHRSProtocol.decodeBinaryUint16(current_data, IMURegisters.NAVX_REG_TIMESTAMP_H_L - first_address)

            sensor_timestamp = (timestamp_high << 16) + timestamp_low

            if sensor_timestamp == self.last_sensor_timestamp:
                return

            self.last_sensor_timestamp = sensor_timestamp;
            self.ahrspos_update.op_status       = current_data[IMURegisters.NAVX_REG_OP_STATUS - first_address]
            self.ahrspos_update.selftest_status = current_data[IMURegisters.NAVX_REG_SELFTEST_STATUS - first_address]
            self.ahrspos_update.cal_status      = current_data[IMURegisters.NAVX_REG_CAL_STATUS]
            self.ahrspos_update.sensor_status   = current_data[IMURegisters.NAVX_REG_SENSOR_STATUS_L - first_address]
            self.ahrspos_update.yaw             = AHRSProtocol.decodeProtocolSignedHundredthsFloat(current_data, IMURegisters.NAVX_REG_YAW_L - first_address)
            self.ahrspos_update.pitch           = AHRSProtocol.decodeProtocolSignedHundredthsFloat(current_data, IMURegisters.NAVX_REG_PITCH_L - first_address)
            self.ahrspos_update.roll            = AHRSProtocol.decodeProtocolSignedHundredthsFloat(current_data, IMURegisters.NAVX_REG_ROLL_L - first_address)
            self.ahrspos_update.compass_heading = AHRSProtocol.decodeProtocolUnsignedHundredthsFloat(current_data, IMURegisters.NAVX_REG_HEADING_L - first_address)
            self.ahrspos_update.mpu_temp        = AHRSProtocol.decodeProtocolSignedHundredthsFloat(current_data, IMURegisters.NAVX_REG_MPU_TEMP_C_L - first_address)
            self.ahrspos_update.linear_accel_x  = AHRSProtocol.decodeProtocolSignedThousandthsFloat(current_data, IMURegisters.NAVX_REG_LINEAR_ACC_X_L - first_address)
            self.ahrspos_update.linear_accel_y  = AHRSProtocol.decodeProtocolSignedThousandthsFloat(current_data, IMURegisters.NAVX_REG_LINEAR_ACC_Y_L - first_address)
            self.ahrspos_update.linear_accel_z  = AHRSProtocol.decodeProtocolSignedThousandthsFloat(current_data, IMURegisters.NAVX_REG_LINEAR_ACC_Z_L - first_address)
            self.ahrspos_update.altitude        = AHRSProtocol.decodeProtocol1616Float(current_data, IMURegisters.NAVX_REG_ALTITUDE_D_L - first_address)
            self.ahrspos_update.barometric_pressure = AHRSProtocol.decodeProtocol1616Float(current_data, IMURegisters.NAVX_REG_PRESSURE_DL - first_address)
            self.ahrspos_update.fused_heading   = AHRSProtocol.decodeProtocolUnsignedHundredthsFloat(current_data, IMURegisters.NAVX_REG_FUSED_HEADING_L - first_address)
            self.ahrspos_update.quat_w          = AHRSProtocol.decodeBinaryInt16(current_data, IMURegisters.NAVX_REG_QUAT_W_L - first_address)
            self.ahrspos_update.quat_x          = AHRSProtocol.decodeBinaryInt16(current_data, IMURegisters.NAVX_REG_QUAT_X_L - first_address)
            self.ahrspos_update.quat_y          = AHRSProtocol.decodeBinaryInt16(current_data, IMURegisters.NAVX_REG_QUAT_Y_L - first_address)
            self.ahrspos_update.quat_z          = AHRSProtocol.decodeBinaryInt16(current_data, IMURegisters.NAVX_REG_QUAT_Z_L - first_address)

            if displacement_registers:
                self.ahrspos_update.vel_x       = AHRSProtocol.decodeProtocol1616Float(current_data, IMURegisters.NAVX_REG_VEL_X_I_L - first_address)
                self.ahrspos_update.vel_y       = AHRSProtocol.decodeProtocol1616Float(current_data, IMURegisters.NAVX_REG_VEL_Y_I_L - first_address)
                self.ahrspos_update.vel_z       = AHRSProtocol.decodeProtocol1616Float(current_data, IMURegisters.NAVX_REG_VEL_Z_I_L - first_address)
                self.ahrspos_update.disp_x      = AHRSProtocol.decodeProtocol1616Float(current_data, IMURegisters.NAVX_REG_DISP_X_I_L - first_address)
                self.ahrspos_update.disp_y      = AHRSProtocol.decodeProtocol1616Float(current_data, IMURegisters.NAVX_REG_DISP_Y_I_L - first_address)
                self.ahrspos_update.disp_z      = AHRSProtocol.decodeProtocol1616Float(current_data, IMURegisters.NAVX_REG_DISP_Z_I_L - first_address)
                self.ahrs.setAHRSPosData(ahrspos_update, sensor_timestamp)
            else:
                self.ahrs_update.op_status           = ahrspos_update.op_status
                self.ahrs_update.selftest_status     = ahrspos_update.selftest_status
                self.ahrs_update.cal_status          = ahrspos_update.cal_status
                self.ahrs_update.sensor_status       = ahrspos_update.sensor_status
                self.ahrs_update.yaw                 = ahrspos_update.yaw
                self.ahrs_update.pitch               = ahrspos_update.pitch
                self.ahrs_update.roll                = ahrspos_update.roll
                self.ahrs_update.compass_heading     = ahrspos_update.compass_heading
                self.ahrs_update.mpu_temp            = ahrspos_update.mpu_temp
                self.ahrs_update.linear_accel_x      = ahrspos_update.linear_accel_x
                self.ahrs_update.linear_accel_y      = ahrspos_update.linear_accel_y
                self.ahrs_update.linear_accel_z      = ahrspos_update.linear_accel_z
                self.ahrs_update.altitude            = ahrspos_update.altitude
                self.ahrs_update.barometric_pressure = ahrspos_update.barometric_pressure
                self.ahrs_update.fused_heading       = ahrspos_update.fused_heading
                self.ahrs.setAHRSData( ahrs_update, sensor_timestamp )

            self.board_state.cal_status       = current_data[IMURegisters.NAVX_REG_CAL_STATUS - first_address]
            self.board_state.op_status        = current_data[IMURegisters.NAVX_REG_OP_STATUS - first_address]
            self.board_state.selftest_status  = current_data[IMURegisters.NAVX_REG_SELFTEST_STATUS - first_address]
            self.board_state.sensor_status    = AHRSProtocol.decodeBinaryUint16(current_data, IMURegisters.NAVX_REG_SENSOR_STATUS_L - first_address)
            self.board_state.update_rate_hz   = current_data[IMURegisters.NAVX_REG_UPDATE_RATE_HZ - first_address]
            self.board_state.gyro_fsr_dps     = AHRSProtocol.decodeBinaryUint16(current_data, IMURegisters.NAVX_REG_GYRO_FSR_DPS_L)
            self.board_state.accel_fsr_g      = current_data[IMURegisters.NAVX_REG_ACCEL_FSR_G]
            self.board_state.capability_flags = AHRSProtocol.decodeBinaryUint16(current_data, IMURegisters.NAVX_REG_CAPABILITY_FLAGS_L - first_address)
            self.ahrs.setBoardState(board_state)

            self.raw_data_update.gyro_x      = AHRSProtocol.decodeBinaryInt16(current_data,  IMURegisters.NAVX_REG_GYRO_X_L - first_address)
            self.raw_data_update.gyro_y      = AHRSProtocol.decodeBinaryInt16(current_data,  IMURegisters.NAVX_REG_GYRO_Y_L - first_address)
            self.raw_data_update.gyro_z      = AHRSProtocol.decodeBinaryInt16(current_data,  IMURegisters.NAVX_REG_GYRO_Z_L - first_address)
            self.raw_data_update.accel_x     = AHRSProtocol.decodeBinaryInt16(current_data,  IMURegisters.NAVX_REG_ACC_X_L - first_address)
            self.raw_data_update.accel_y     = AHRSProtocol.decodeBinaryInt16(current_data,  IMURegisters.NAVX_REG_ACC_Y_L - first_address)
            self.raw_data_update.accel_z     = AHRSProtocol.decodeBinaryInt16(current_data,  IMURegisters.NAVX_REG_ACC_Z_L - first_address)
            self.raw_data_update.mag_x       = AHRSProtocol.decodeBinaryInt16(current_data,  IMURegisters.NAVX_REG_MAG_X_L - first_address)
            self.raw_data_update.mag_y       = AHRSProtocol.decodeBinaryInt16(current_data,  IMURegisters.NAVX_REG_MAG_Y_L - first_address)
            self.raw_data_update.mag_z       = AHRSProtocol.decodeBinaryInt16(current_data,  IMURegisters.NAVX_REG_MAG_Z_L - first_address)
            self.raw_data_update.temp_c      = ahrspos_update.mpu_temp
            self.ahrs.setRawData(raw_data_update, sensor_timestamp)

            self.last_update_time = time.time();
            byte_count += current_data.length;
            update_count += 1;

    def isConnected(self):
        time_since_last_update = time.time() - self.last_update_time;
        return time_since_last_update <= IO_TIMEOUT_SECONDS

    def getByteCount(self):
        return self.byte_count

    def getUpdateCount(self):
        return self.update_count

    def setUpdateRateHz(self, update_rate_hz):
        self.write(IMURegisters.NAVX_REG_UPDATE_RATE_HZ, update_rate_hz)

    def zeroYaw(self):
        self.write( IMURegisters.NAVX_REG_INTEGRATION_CTL,
                                   AHRSProtocol.NAVX_INTEGRATION_CTL_RESET_YAW );
    def zeroDisplacement(self):
        self.write( IMURegisters.NAVX_REG_INTEGRATION_CTL,
                            (int)(AHRSProtocol.NAVX_INTEGRATION_CTL_RESET_DISP_X |
                                   AHRSProtocol.NAVX_INTEGRATION_CTL_RESET_DISP_Y |
                                   AHRSProtocol.NAVX_INTEGRATION_CTL_RESET_DISP_Z ) )

    def read(self, first_address, length):
        return self.i2c.readList(first_address, length)

    def write(self, first_address, value):
        self.i2c.write8(first_address, value)
