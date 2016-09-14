import time

import Adafruit_GPIO.I2C as I2C

from Structs.AHRSPosUpdate import AHRSPosUpdate
from Structs.AHRSUpdate import AHRSUpdate
from Structs.GyroUpdate import GyroUpdate
from Structs.BoardID import BoardID
from Structs.BoardState import BoardState

from IMURegisters import IMURegisters
from AHRSProtocol import AHRSProtocol

import logging
from ourlogging import setup_logging
setup_logging(__file__)
logger = logging.getLogger(__name__)


class I2C_IO:
    DELAY_OVERHEAD_SECONDS = 0.004
    IO_TIMEOUT_SECONDS = 1.0

    def __init__(self, ahrs, update_rate_hz, busnum=None, i2c_interface=None, **kwargs):
        self.i2c = I2C.get_i2c_device(0x32, busnum, i2c_interface, **kwargs)

        self.update_rate_hz = update_rate_hz
        self.running = True

        self.ahrs = ahrs
        self.raw_data_update = GyroUpdate()
        self.ahrs_update = AHRSUpdate()
        self.ahrspos_update = AHRSPosUpdate()
        self.board_id = BoardID()
        self.board_state = BoardState()

        self.last_update_time = 0.0
        self.byte_count = 0
        self.update_count = 0

    def stop(self):
        self.running = False

    def isRunning(self):
        return self.running

    def run(self):
        # Initial Device Configuration
        self.setUpdateRateHz(self.update_rate_hz)
        if not self.getConfiguration():
            logger.warn("-- Did not get configuration data")
        else:
            logger.info("-- Board is %s (rev %s)", IMURegisters.model_type(self.board_id.type), self.board_id.hw_rev)
            logger.info("-- Firmware %s.%s", self.board_id.fw_ver_major, self.board_id.fw_ver_minor)

        '''
            Calculate delay to match configured update rate
            Note:  some additional time is removed from the
            1/update_rate value to ensure samples are not
            dropped, esp. at higher update rates.
        '''

        update_rate = 1.0/(self.update_rate_hz & 0xFF)

        if (update_rate > self.DELAY_OVERHEAD_SECONDS):
            update_rate -= self.DELAY_OVERHEAD_SECONDS

        log_error = True

        # IO Loop
        while self.running:
            if not self.board_state.update_rate_hz == self.update_rate_hz:
                self.setUpdateRateHz(self.update_rate_hz)

            try:
                self.getCurrentData()
            except IOError:
                if log_error:
                    logger.exception("Error getting data")
                    log_error = False
            else:
                log_error = True
            time.sleep(update_rate)

    def getConfiguration(self):
        success = False
        retry_count = 0

        while retry_count < 5 and not success:
            try:
                config = self.read(IMURegisters.NAVX_REG_WHOAMI, IMURegisters.NAVX_REG_SENSOR_STATUS_H + 1)
            except IOError as e:
                logger.warn("Error reading configuration data, retrying (%s)", e)
                success = False
                time.delay(0.5)
            else:
                board_id = self.board_id
                board_id.hw_rev                 = config[IMURegisters.NAVX_REG_HW_REV]
                board_id.fw_ver_major           = config[IMURegisters.NAVX_REG_FW_VER_MAJOR]
                board_id.fw_ver_minor           = config[IMURegisters.NAVX_REG_FW_VER_MINOR]
                board_id.type                   = config[IMURegisters.NAVX_REG_WHOAMI]
                self.ahrs._setBoardID(board_id);

                board_state = self.board_state
                board_state.cal_status          = config[IMURegisters.NAVX_REG_CAL_STATUS]
                board_state.op_status           = config[IMURegisters.NAVX_REG_OP_STATUS]
                board_state.selftest_status     = config[IMURegisters.NAVX_REG_SELFTEST_STATUS]
                board_state.sensor_status       = AHRSProtocol.decodeBinaryUint16(config, IMURegisters.NAVX_REG_SENSOR_STATUS_L)
                board_state.gyro_fsr_dps        = AHRSProtocol.decodeBinaryUint16(config, IMURegisters.NAVX_REG_GYRO_FSR_DPS_L)
                board_state.accel_fsr_g         = config[IMURegisters.NAVX_REG_ACCEL_FSR_G]
                board_state.update_rate_hz      = config[IMURegisters.NAVX_REG_UPDATE_RATE_HZ]
                board_state.capability_flags    = AHRSProtocol.decodeBinaryUint16(config, IMURegisters.NAVX_REG_CAPABILITY_FLAGS_L)
                self.ahrs._setBoardState(board_state);
                success = True

            retry_count += 1

        return success

    def getCurrentData(self):

        first_address = IMURegisters.NAVX_REG_UPDATE_RATE_HZ
        displacement_registers = self.ahrs._isDisplacementSupported()

        # If firmware supports displacement data, acquire it - otherwise implement
        # similar (but potentially less accurate) calculations on this processor.
        if displacement_registers:
            read_count = IMURegisters.NAVX_REG_LAST + 1 - first_address
        else:
            read_count = IMURegisters.NAVX_REG_QUAT_OFFSET_Z_H + 1 - first_address

        current_data = self.read(first_address, read_count)

        timestamp_low = AHRSProtocol.decodeBinaryUint16(current_data, IMURegisters.NAVX_REG_TIMESTAMP_L_L-first_address)
        timestamp_high = AHRSProtocol.decodeBinaryUint16(curr_data, IMURegisters.NAVX_REG_TIMESTAMP_H_L-first_address)
        sensor_timestamp = (timestamp_high << 16) + timestamp_low

        if sensor_timestamp == self.last_sensor_timestamp:
            	return;
        self.last_sensor_timestamp = sensor_timestamp

        ahrspos_update = self.ahrspos_update
        ahrspos_update.op_status       = current_data[IMURegisters.NAVX_REG_OP_STATUS - first_address]
        ahrspos_update.selftest_status = current_data[IMURegisters.NAVX_REG_SELFTEST_STATUS - first_address]
        ahrspos_update.cal_status      = current_data[IMURegisters.NAVX_REG_CAL_STATUS]
        ahrspos_update.sensor_status   = current_data[IMURegisters.NAVX_REG_SENSOR_STATUS_L - first_address]
        ahrspos_update.yaw             = AHRSProtocol.decodeProtocolSignedHundredthsFloat(current_data, IMURegisters.NAVX_REG_YAW_L-first_address)
        ahrspos_update.pitch           = AHRSProtocol.decodeProtocolSignedHundredthsFloat(current_data, IMURegisters.NAVX_REG_PITCH_L-first_address)
        ahrspos_update.roll            = AHRSProtocol.decodeProtocolSignedHundredthsFloat(current_data, IMURegisters.NAVX_REG_ROLL_L-first_address)
        ahrspos_update.compass_heading = AHRSProtocol.decodeProtocolUnsignedHundredthsFloat(current_data, IMURegisters.NAVX_REG_HEADING_L-first_address)
        ahrspos_update.mpu_temp_c      = AHRSProtocol.decodeProtocolSignedHundredthsFloat(current_data, IMURegisters.NAVX_REG_MPU_TEMP_C_L - first_address)
        ahrspos_update.world_linear_accel_x  = AHRSProtocol.decodeProtocolSignedThousandthsFloat(current_data, IMURegisters.NAVX_REG_LINEAR_ACC_X_L-first_address)
        ahrspos_update.world_linear_accel_y  = AHRSProtocol.decodeProtocolSignedThousandthsFloat(current_data, IMURegisters.NAVX_REG_LINEAR_ACC_Y_L-first_address)
        ahrspos_update.world_linear_accel_z  = AHRSProtocol.decodeProtocolSignedThousandthsFloat(current_data, IMURegisters.NAVX_REG_LINEAR_ACC_Z_L-first_address)
        ahrspos_update.altitude        = AHRSProtocol.decodeProtocol1616Float(current_data, IMURegisters.NAVX_REG_ALTITUDE_D_L - first_address)
        ahrspos_update.baro_pressure   = AHRSProtocol.decodeProtocol1616Float(current_data, IMURegisters.NAVX_REG_PRESSURE_DL - first_address)
        ahrspos_update.fused_heading   = AHRSProtocol.decodeProtocolUnsignedHundredthsFloat(current_data, IMURegisters.NAVX_REG_FUSED_HEADING_L-first_address)
        ahrspos_update.quaternionW     = AHRSProtocol.decodeBinaryInt16(current_data, IMURegisters.NAVX_REG_QUAT_W_L-first_address)
        ahrspos_update.quaternionX     = AHRSProtocol.decodeBinaryInt16(current_data, IMURegisters.NAVX_REG_QUAT_X_L-first_address)
        ahrspos_update.quaternionY     = AHRSProtocol.decodeBinaryInt16(current_data, IMURegisters.NAVX_REG_QUAT_Y_L-first_address)
        ahrspos_update.quaternionZ     = AHRSProtocol.decodeBinaryInt16(current_data, IMURegisters.NAVX_REG_QUAT_Z_L-first_address)
        if displacement_registers:
            ahrspos_update.vel_x       = AHRSProtocol.decodeProtocol1616Float(current_data, IMURegisters.NAVX_REG_VEL_X_I_L-first_address)
            ahrspos_update.vel_y       = AHRSProtocol.decodeProtocol1616Float(current_data, IMURegisters.NAVX_REG_VEL_Y_I_L-first_address)
            ahrspos_update.vel_z       = AHRSProtocol.decodeProtocol1616Float(current_data, IMURegisters.NAVX_REG_VEL_Z_I_L-first_address)
            ahrspos_update.disp_x      = AHRSProtocol.decodeProtocol1616Float(current_data, IMURegisters.NAVX_REG_DISP_X_I_L-first_address)
            ahrspos_update.disp_y      = AHRSProtocol.decodeProtocol1616Float(current_data, IMURegisters.NAVX_REG_DISP_Y_I_L-first_address)
            ahrspos_update.disp_z      = AHRSProtocol.decodeProtocol1616Float(current_data, IMURegisters.NAVX_REG_DISP_Z_I_L-first_address)

            self.ahrs._setAHRSPosData(ahrspos_update, sensor_timestamp)
        else:
            self.ahrs._setAHRSData(ahrspos_update, sensor_timestamp)

        board_state = self.board_state
        board_state.cal_status      = current_data[IMURegisters.NAVX_REG_CAL_STATUS-first_address]
        board_state.op_status       = current_data[IMURegisters.NAVX_REG_OP_STATUS-first_address]
        board_state.selftest_status = current_data[IMURegisters.NAVX_REG_SELFTEST_STATUS-first_address]
        board_state.sensor_status   = AHRSProtocol.decodeBinaryUint16(current_data, IMURegisters.NAVX_REG_SENSOR_STATUS_L-first_address)
        board_state.update_rate_hz  = current_data[IMURegisters.NAVX_REG_UPDATE_RATE_HZ-first_address]
        board_state.gyro_fsr_dps    = AHRSProtocol.decodeBinaryUint16(current_data, IMURegisters.NAVX_REG_GYRO_FSR_DPS_L)
        board_state.accel_fsr_g     = current_data[IMURegisters.NAVX_REG_ACCEL_FSR_G]
        board_state.capability_flags= AHRSProtocol.decodeBinaryUint16(current_data, IMURegisters.NAVX_REG_CAPABILITY_FLAGS_L-first_address)
        self.ahrs._setBoardState(board_state)

        raw_data_update = self.raw_data_update
        raw_data_update.raw_gyro_x      = AHRSProtocol.decodeBinaryInt16(current_data,  IMURegisters.NAVX_REG_GYRO_X_L-first_address)
        raw_data_update.raw_gyro_y      = AHRSProtocol.decodeBinaryInt16(current_data,  IMURegisters.NAVX_REG_GYRO_Y_L-first_address)
        raw_data_update.raw_gyro_z      = AHRSProtocol.decodeBinaryInt16(current_data,  IMURegisters.NAVX_REG_GYRO_Z_L-first_address)
        raw_data_update.raw_accel_x     = AHRSProtocol.decodeBinaryInt16(current_data,  IMURegisters.NAVX_REG_ACC_X_L-first_address)
        raw_data_update.raw_accel_y     = AHRSProtocol.decodeBinaryInt16(current_data,  IMURegisters.NAVX_REG_ACC_Y_L-first_address)
        raw_data_update.raw_accel_z     = AHRSProtocol.decodeBinaryInt16(current_data,  IMURegisters.NAVX_REG_ACC_Z_L-first_address)
        raw_data_update.cal_mag_x       = AHRSProtocol.decodeBinaryInt16(current_data,  IMURegisters.NAVX_REG_MAG_X_L-first_address)
        raw_data_update.cal_mag_y       = AHRSProtocol.decodeBinaryInt16(current_data,  IMURegisters.NAVX_REG_MAG_Y_L-first_address)
        raw_data_update.cal_mag_z       = AHRSProtocol.decodeBinaryInt16(current_data,  IMURegisters.NAVX_REG_MAG_Z_L-first_address)
        raw_data_update.mpu_temp_c      = ahrspos_update.mpu_temp
        self.ahrs._setRawData(raw_data_update, sensor_timestamp)

        self.last_update_time = time.time()
        self.byte_count += len(current_data)
        self.update_count += 1

    def isConnected(self):
        time_since_last_update = time.time() - self.last_update_time
        return time_since_last_update <= self.IO_TIMEOUT_SECONDS

    def getByteCount(self):
        return self.byte_count

    def getUpdateCount(self):
        return self.update_count

    def setUpdateRateHz(self, update_rate_hz):
        self.write(IMURegisters.NAVX_REG_UPDATE_RATE_HZ, update_rate_hz)

    def zeroYaw(self):
        self.write(IMURegisters.NAVX_REG_INTEGRATION_CTL, AHRSProtocol.NAVX_INTEGRATION_CTL_RESET_YAW)

    def zeroDisplacement(self):
        self.write(IMURegisters.NAVX_REG_INTEGRATION_CTL, (AHRSProtocol.NAVX_INTEGRATION_CTL_RESET_DISP_X | AHRSProtocol.NAVX_INTEGRATION_CTL_RESET_DISP_Y | AHRSProtocol.NAVX_INTEGRATION_CTL_RESET_DISP_Z))

    def read(self, first_address, length):
        return self.i2c.readList(first_address, length)

    def write(self, first_address, value):
        self.i2c.write8(first_address | 0x80, value)
