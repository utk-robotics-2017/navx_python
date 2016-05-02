from IMUProtocol import IMUProtocol


class AHRSProtocol(IMUProtocol):

    # NAVX_CAL_STATUS
    NAVX_CAL_STATUS_IMU_CAL_STATE_MASK = 0x03
    NAVX_CAL_STATUS_IMU_CAL_INPROGRESS = 0x00
    NAVX_CAL_STATUS_IMU_CAL_ACCUMULATE = 0x01
    NAVX_CAL_STATUS_IMU_CAL_COMPLETE = 0x02

    NAVX_CAL_STATUS_MAG_CAL_COMPLETE = 0x04
    NAVX_CAL_STATUS_BARO_CAL_COMPLETE = 0x08

    # NAVX_SELFTEST_STATUS
    NAVX_SELFTEST_STATUS_COMPLETE = 0x80

    NAVX_SELFTEST_RESULT_GYRO_PASSED = 0x01
    NAVX_SELFTEST_RESULT_ACCEL_PASSED = 0x02
    NAVX_SELFTEST_RESULT_MAG_PASSED = 0x04
    NAVX_SELFTEST_RESULT_BARO_PASSED = 0x08

    # NAVX_OP_STATUS
    NAVX_OP_STATUS_INITIALIZING = 0x00
    NAVX_OP_STATUS_SELFTEST_IN_PROGRESS = 0x01
    NAVX_OP_STATUS_ERROR = 0x02
    NAVX_OP_STATUS_IMU_AUTOCAL_IN_PROGRESS = 0x03
    NAVX_OP_STATUS_NORMAL = 0x04

    # NAVX_SENSOR_STATUS
    NAVX_SENSOR_STATUS_MOVING = 0x01
    NAVX_SENSOR_STATUS_YAW_STABLE = 0x02
    NAVX_SENSOR_STATUS_MAG_DISTURBANCE = 0x04
    NAVX_SENSOR_STATUS_ALTITUDE_VALID = 0x08
    NAVX_SENSOR_STATUS_SEALEVEL_PRESS_SET = 0x10
    NAVX_SENSOR_STATUS_FUSED_HEADING_VALID = 0x20

    # NAVX_REG_CAPABILITY_FLAGS (Aligned w/NAV6 Flags, see IMUProtocol.h)
    NAVX_CAPABILITY_FLAG_OMNIMOUNT = 0x0004
    NAVX_CAPABILITY_FLAG_OMNIMOUNT_CONFIG_MASK = 0x0038
    NAVX_CAPABILITY_FLAG_VEL_AND_DISP = 0x0040
    NAVX_CAPABILITY_FLAG_YAW_RESET = 0x0080

    # NAVX_OMNIMOUNT_CONFIG
    OMNIMOUNT_DEFAULT = 0  # Same as Y_Z_UP
    OMNIMOUNT_YAW_X_UP = 1
    OMNIMOUNT_YAW_X_DOWN = 2
    OMNIMOUNT_YAW_Y_UP = 3
    OMNIMOUNT_YAW_Y_DOWN = 4
    OMNIMOUNT_YAW_Z_UP = 5
    OMNIMOUNT_YAW_Z_DOWN = 6

    # NAVX_INTEGRATION_CTL
    NAVX_INTEGRATION_CTL_RESET_VEL_X = 0x01
    NAVX_INTEGRATION_CTL_RESET_VEL_Y = 0x02
    NAVX_INTEGRATION_CTL_RESET_VEL_Z = 0x04
    NAVX_INTEGRATION_CTL_RESET_DISP_X = 0x08
    NAVX_INTEGRATION_CTL_RESET_DISP_Y = 0x10
    NAVX_INTEGRATION_CTL_RESET_DISP_Z = 0x20
    NAVX_INTEGRATION_CTL_RESET_YAW = 0x80

    class AHRS_TUNING_VAR_ID:
        UNSPECIFIED = 0
        MOTION_THRESHOLD = 1  # In G
        YAW_STABLE_THRESHOLD = 2  # In Degrees
        MAG_DISTURBANCE_THRESHOLD = 3  # Ratio
        SEA_LEVEL_PRESSURE = 4  # Millibars

    class AHRS_DATA_TYPE:
        TUNING_VARIABLE = 0
        MAG_CALIBRATION = 1
        BOARD_IDENTITY = 2

    class AHRS_DATA_ACTION:
        DATA_GET = 0
        DATA_SET = 1

    BINARY_PACKET_INDICATOR_CHAR = '#'

    '''
        AHRS Protocol encodes certain data in binary format, unlike the IMU
        protocol, which encodes all data in ASCII characters.  Thus, the
        packet start and message termination sequences may occur within the
        message content itself.  To support the binary format, the binary
        message has this format:

        [start][binary indicator][len][msgid]<MESSAGE>[checksum][terminator]

        (The binary indicator and len are not present in the ASCII protocol)

        The [len] does not include the length of the start and binary
        indicator characters, but does include all other message items,
        including the checksum and terminator sequence.
    '''

    MSGID_AHRS_UPDATE = 'a'
    AHRS_UPDATE_YAW_VALUE_INDEX = 4  # Degrees.  Signed Hundredths
    AHRS_UPDATE_PITCH_VALUE_INDEX = 6  # Degrees.  Signed Hundredeths
    AHRS_UPDATE_ROLL_VALUE_INDEX = 8  # Degrees.  Signed Hundredths
    AHRS_UPDATE_HEADING_VALUE_INDEX = 10  # Degrees.  Unsigned Hundredths
    AHRS_UPDATE_ALTITUDE_VALUE_INDEX = 12  # Meters.   Signed 16:16
    AHRS_UPDATE_FUSED_HEADING_VALUE_INDEX = 16  # Degrees.  Unsigned Hundredths
    AHRS_UPDATE_LINEAR_ACCEL_X_VALUE_INDEX = 18  # Inst. G.  Signed Thousandths
    AHRS_UPDATE_LINEAR_ACCEL_Y_VALUE_INDEX = 20  # Inst. G.  Signed Thousandths
    AHRS_UPDATE_LINEAR_ACCEL_Z_VALUE_INDEX = 22  # Inst. G.  Signed Thousandths
    AHRS_UPDATE_CAL_MAG_X_VALUE_INDEX = 24  # Int16 (Device Units)
    AHRS_UPDATE_CAL_MAG_Y_VALUE_INDEX = 26  # Int16 (Device Units)
    AHRS_UPDATE_CAL_MAG_Z_VALUE_INDEX = 28  # Int16 (Device Units)
    AHRS_UPDATE_CAL_MAG_NORM_RATIO_VALUE_INDEX = 30  # Ratio.  Unsigned Hundredths
    AHRS_UPDATE_CAL_MAG_SCALAR_VALUE_INDEX = 32  # Coefficient. Signed 16:16
    AHRS_UPDATE_MPU_TEMP_VAUE_INDEX = 36  # Centigrade.  Signed Hundredths
    AHRS_UPDATE_RAW_MAG_X_VALUE_INDEX = 38  # INT16 (Device Units)
    AHRS_UPDATE_RAW_MAG_Y_VALUE_INDEX = 40  # INT16 (Device Units)
    AHRS_UPDATE_RAW_MAG_Z_VALUE_INDEX = 42  # INT16 (Device Units)
    AHRS_UPDATE_QUAT_W_VALUE_INDEX = 44  # INT16
    AHRS_UPDATE_QUAT_X_VALUE_INDEX = 46  # INT16
    AHRS_UPDATE_QUAT_Y_VALUE_INDEX = 48  # INT16
    AHRS_UPDATE_QUAT_Z_VALUE_INDEX = 50  # INT16
    AHRS_UPDATE_BARO_PRESSURE_VALUE_INDEX = 52  # millibar.  Signed 16:16
    AHRS_UPDATE_BARO_TEMP_VAUE_INDEX = 56  # Centigrade.  Signed  Hundredths
    AHRS_UPDATE_OPSTATUS_VALUE_INDEX = 58  # NAVX_OP_STATUS_XXX
    AHRS_UPDATE_SENSOR_STATUS_VALUE_INDEX = 59  # NAVX_SENSOR_STATUS_XXX
    AHRS_UPDATE_CAL_STATUS_VALUE_INDEX = 60  # NAVX_CAL_STATUS_XXX
    AHRS_UPDATE_SELFTEST_STATUS_VALUE_INDEX = 61  # NAVX_SELFTEST_STATUS_XXX
    AHRS_UPDATE_MESSAGE_CHECKSUM_INDEX = 62
    AHRS_UPDATE_MESSAGE_TERMINATOR_INDEX = 64
    AHRS_UPDATE_MESSAGE_LENGTH = 66

    # AHRSAndPositioning Update Packet (similar to AHRS, but removes magnetometer and adds velocity/displacement)

    MSGID_AHRSPOS_UPDATE = 'p'
    AHRSPOS_UPDATE_YAW_VALUE_INDEX = 4  # Degrees.  Signed Hundredths
    AHRSPOS_UPDATE_PITCH_VALUE_INDEX = 6  # Degrees.  Signed Hundredeths
    AHRSPOS_UPDATE_ROLL_VALUE_INDEX = 8  # Degrees.  Signed Hundredths
    AHRSPOS_UPDATE_HEADING_VALUE_INDEX = 10  # Degrees.  Unsigned Hundredths
    AHRSPOS_UPDATE_ALTITUDE_VALUE_INDEX = 12  # Meters.   Signed 16:16
    AHRSPOS_UPDATE_FUSED_HEADING_VALUE_INDEX = 16  # Degrees.  Unsigned Hundredths
    AHRSPOS_UPDATE_LINEAR_ACCEL_X_VALUE_INDEX = 18  # Inst. G.  Signed Thousandths
    AHRSPOS_UPDATE_LINEAR_ACCEL_Y_VALUE_INDEX = 20  # Inst. G.  Signed Thousandths
    AHRSPOS_UPDATE_LINEAR_ACCEL_Z_VALUE_INDEX = 22  # Inst. G.  Signed Thousandths
    AHRSPOS_UPDATE_VEL_X_VALUE_INDEX = 24  # Signed 16:16, in meters/sec
    AHRSPOS_UPDATE_VEL_Y_VALUE_INDEX = 28  # Signed 16:16, in meters/sec
    AHRSPOS_UPDATE_VEL_Z_VALUE_INDEX = 32  # Signed 16:16, in meters/sec
    AHRSPOS_UPDATE_DISP_X_VALUE_INDEX = 36  # Signed 16:16, in meters
    AHRSPOS_UPDATE_DISP_Y_VALUE_INDEX = 40  # Signed 16:16, in meters
    AHRSPOS_UPDATE_DISP_Z_VALUE_INDEX = 44  # Signed 16:16, in meters
    AHRSPOS_UPDATE_QUAT_W_VALUE_INDEX = 48  # INT16
    AHRSPOS_UPDATE_QUAT_X_VALUE_INDEX = 50  # INT16
    AHRSPOS_UPDATE_QUAT_Y_VALUE_INDEX = 52  # INT16
    AHRSPOS_UPDATE_QUAT_Z_VALUE_INDEX = 54  # INT16
    AHRSPOS_UPDATE_MPU_TEMP_VAUE_INDEX = 56  # Centigrade.  Signed Hundredths
    AHRSPOS_UPDATE_OPSTATUS_VALUE_INDEX = 58  # NAVX_OP_STATUS_XXX
    AHRSPOS_UPDATE_SENSOR_STATUS_VALUE_INDEX = 59  # NAVX_SENSOR_STATUS_XXX
    AHRSPOS_UPDATE_CAL_STATUS_VALUE_INDEX = 60  # NAVX_CAL_STATUS_XXX
    AHRSPOS_UPDATE_SELFTEST_STATUS_VALUE_INDEX = 61  # NAVX_SELFTEST_STATUS_XXX
    AHRSPOS_UPDATE_MESSAGE_CHECKSUM_INDEX = 62
    AHRSPOS_UPDATE_MESSAGE_TERMINATOR_INDEX = 64
    AHRSPOS_UPDATE_MESSAGE_LENGTH = 66

    # Data Get Request:  Tuning Variable, Mag Cal, Board Identity (Response message depends upon request type)
    MSGID_DATA_REQUEST = 'D'
    DATA_REQUEST_DATATYPE_VALUE_INDEX = 4
    DATA_REQUEST_VARIABLEID_VALUE_INDEX = 5
    DATA_REQUEST_CHECKSUM_INDEX = 6
    DATA_REQUEST_TERMINATOR_INDEX = 8
    DATA_REQUEST_MESSAGE_LENGTH = 10

    # Data Set Response Packet
    MSGID_DATA_SET_RESPONSE = 'v'
    DATA_SET_RESPONSE_DATATYPE_VALUE_INDEX = 4
    DATA_SET_RESPONSE_VARID_VALUE_INDEX = 5
    DATA_SET_RESPONSE_STATUS_VALUE_INDEX = 6
    DATA_SET_RESPONSE_MESSAGE_CHECKSUM_INDEX = 7
    DATA_SET_RESPONSE_MESSAGE_TERMINATOR_INDEX = 9
    DATA_SET_RESPONSE_MESSAGE_LENGTH = 11

    # Integration Control Command Packet
    MSGID_INTEGRATION_CONTROL_CMD = 'I'
    INTEGRATION_CONTROL_CMD_ACTION_INDEX = 4
    INTEGRATION_CONTROL_CMD_PARAMETER_INDEX = 5
    INTEGRATION_CONTROL_CMD_MESSAGE_CHECKSUM_INDEX = 9
    INTEGRATION_CONTROL_CMD_MESSAGE_TERMINATOR_INDEX = 11
    INTEGRATION_CONTROL_CMD_MESSAGE_LENGTH = 13

    # Integration Control Response Packet
    MSGID_INTEGRATION_CONTROL_RESP = 'i'
    INTEGRATION_CONTROL_RESP_ACTION_INDEX = 4
    INTEGRATION_CONTROL_RESP_PARAMETER_INDEX = 5
    INTEGRATION_CONTROL_RESP_MESSAGE_CHECKSUM_INDEX = 9
    INTEGRATION_CONTROL_RESP_MESSAGE_TERMINATOR_INDEX = 11
    INTEGRATION_CONTROL_RESP_MESSAGE_LENGTH = 13

    # Magnetometer Calibration Packet - e.g., !m[x_bias][y_bias][z_bias][m1,1 ... m3,3][cr][lf]
    MSGID_MAG_CAL_CMD = 'M'
    MAG_CAL_DATA_ACTION_VALUE_INDEX = 4
    MAG_X_BIAS_VALUE_INDEX = 5  # signed short
    MAG_Y_BIAS_VALUE_INDEX = 7
    MAG_Z_BIAS_VALUE_INDEX = 9
    MAG_XFORM_1_1_VALUE_INDEX = 11  # signed 16:16
    MAG_XFORM_1_2_VALUE_INDEX = 15
    MAG_XFORM_1_3_VALUE_INDEX = 19
    MAG_XFORM_2_1_VALUE_INDEX = 23
    MAG_XFORM_2_2_VALUE_INDEX = 25
    MAG_XFORM_2_3_VALUE_INDEX = 31
    MAG_XFORM_3_1_VALUE_INDEX = 35
    MAG_XFORM_3_2_VALUE_INDEX = 39
    MAG_XFORM_3_3_VALUE_INDEX = 43
    MAG_CAL_EARTH_MAG_FIELD_NORM_VALUE_INDEX = 47
    MAG_CAL_CMD_MESSAGE_CHECKSUM_INDEX = 51
    MAG_CAL_CMD_MESSAGE_TERMINATOR_INDEX = 53
    MAG_CAL_CMD_MESSAGE_LENGTH = 55

    # Tuning Variable Packet
    MSGID_FUSION_TUNING_CMD = 'T'
    FUSION_TUNING_DATA_ACTION_VALUE_INDEX = 4
    FUSION_TUNING_CMD_VAR_ID_VALUE_INDEX = 5
    FUSION_TUNING_CMD_VAR_VALUE_INDEX = 6
    FUSION_TUNING_CMD_MESSAGE_CHECKSUM_INDEX = 10
    FUSION_TUNING_CMD_MESSAGE_TERMINATOR_INDEX = 12
    FUSION_TUNING_CMD_MESSAGE_LENGTH = 14

    # Board Identity Response Packet- e.g., !c[type][hw_rev][fw_major][fw_minor][unique_id[12]]
    MSGID_BOARD_IDENTITY_RESPONSE = 'i'
    BOARD_IDENTITY_BOARDTYPE_VALUE_INDEX = 4
    BOARD_IDENTITY_HWREV_VALUE_INDEX = 5
    BOARD_IDENTITY_FW_VER_MAJOR = 6
    BOARD_IDENTITY_FW_VER_MINOR = 7
    BOARD_IDENTITY_FW_VER_REVISION_VALUE_INDEX = 8
    BOARD_IDENTITY_UNIQUE_ID_0 = 10
    BOARD_IDENTITY_UNIQUE_ID_1 = 11
    BOARD_IDENTITY_UNIQUE_ID_2 = 12
    BOARD_IDENTITY_UNIQUE_ID_3 = 13
    BOARD_IDENTITY_UNIQUE_ID_4 = 14
    BOARD_IDENTITY_UNIQUE_ID_5 = 15
    BOARD_IDENTITY_UNIQUE_ID_6 = 16
    BOARD_IDENTITY_UNIQUE_ID_7 = 17
    BOARD_IDENTITY_UNIQUE_ID_8 = 18
    BOARD_IDENTITY_UNIQUE_ID_9 = 19
    BOARD_IDENTITY_UNIQUE_ID_10 = 20
    BOARD_IDENTITY_UNIQUE_ID_11 = 21
    BOARD_IDENTITY_RESPONSE_CHECKSUM_INDEX = 22
    BOARD_IDENTITY_RESPONSE_TERMINATOR_INDEX = 24
    BOARD_IDENTITY_RESPONSE_MESSAGE_LENGTH = 26

    MAX_BINARY_MESSAGE_LENGTH = AHRSPOS_UPDATE_MESSAGE_LENGTH

    @staticmethod
    def decodeAHRSUpdate(buffer, offset, length, update):
        if (length < AHRSProtocol.AHRS_UPDATE_MESSAGE_LENGTH):
            return 0
        if ((buffer[offset+0] == AHRSProtocol.PACKET_START_CHAR) and (buffer[offset+1] == AHRSProtocol.BINARY_PACKET_INDICATOR_CHAR) and (buffer[offset+2] == AHRSProtocol.AHRS_UPDATE_MESSAGE_LENGTH - 2) and (buffer[offset+3] == AHRSProtocol.MSGID_AHRS_UPDATE)):
            if (not AHRSProtocol.verifyChecksum(buffer, offset, AHRSProtocol.AHRS_UPDATE_MESSAGE_CHECKSUM_INDEX)):
                return 0
            update.yaw = AHRSProtocol.decodeProtocolSignedHundredthsFloat(buffer, offset + AHRSProtocol.AHRS_UPDATE_YAW_VALUE_INDEX)
            update.pitch = AHRSProtocol.decodeProtocolSignedHundredthsFloat(buffer, offset + AHRSProtocol.AHRS_UPDATE_ROLL_VALUE_INDEX)  # FIXME - was in the original code
            update.roll = AHRSProtocol.decodeProtocolSignedHundredthsFloat(buffer, offset + AHRSProtocol.AHRS_UPDATE_PITCH_VALUE_INDEX)  # FIXME - was in the original code
            update.compass_heading = AHRSProtocol.decodeProtocolUnsignedHundredthsFloat(buffer, offset + AHRSProtocol.AHRS_UPDATE_HEADING_VALUE_INDEX)
            update.altitude = AHRSProtocol.decodeProtocol1616Float(buffer, offset + AHRSProtocol.AHRS_UPDATE_ALTITUDE_VALUE_INDEX)
            update.fused_heading = AHRSProtocol.decodeProtocolUnsignedHundredthsFloat(buffer, offset + AHRSProtocol.AHRS_UPDATE_FUSED_HEADING_VALUE_INDEX)
            update.linear_accel_x = AHRSProtocol.decodeProtocolSignedThousandthsFloat(buffer, offset + AHRSProtocol.AHRS_UPDATE_LINEAR_ACCEL_X_VALUE_INDEX)
            update.linear_accel_y = AHRSProtocol.decodeProtocolSignedThousandthsFloat(buffer, offset + AHRSProtocol.AHRS_UPDATE_LINEAR_ACCEL_Y_VALUE_INDEX)
            update.linear_accel_z = AHRSProtocol.decodeProtocolSignedThousandthsFloat(buffer, offset + AHRSProtocol.AHRS_UPDATE_LINEAR_ACCEL_Z_VALUE_INDEX)
            update.cal_mag_x = AHRSProtocol.decodeBinaryInt16(buffer, offset + AHRSProtocol.AHRS_UPDATE_CAL_MAG_X_VALUE_INDEX)
            update.cal_mag_y = AHRSProtocol.decodeBinaryInt16(buffer, offset + AHRSProtocol.AHRS_UPDATE_CAL_MAG_Y_VALUE_INDEX)
            update.cal_mag_z = AHRSProtocol.decodeBinaryInt16(buffer, offset + AHRSProtocol.AHRS_UPDATE_CAL_MAG_Z_VALUE_INDEX)
            update.mag_field_norm_ratio = AHRSProtocol.decodeProtocolUnsignedHundredthsFloat(buffer, offset + AHRSProtocol.AHRS_UPDATE_CAL_MAG_NORM_RATIO_VALUE_INDEX)
            update.mag_field_norm_scalar = AHRSProtocol.decodeProtocol1616Float(buffer, offset + AHRSProtocol.AHRS_UPDATE_CAL_MAG_SCALAR_VALUE_INDEX)
            update.mpu_temp = AHRSProtocol.decodeProtocolSignedHundredthsFloat(buffer, offset + AHRSProtocol.AHRS_UPDATE_MPU_TEMP_VAUE_INDEX)
            update.raw_mag_x = AHRSProtocol.decodeBinaryInt16(buffer, offset + AHRSProtocol.AHRS_UPDATE_RAW_MAG_X_VALUE_INDEX)
            update.raw_mag_y = AHRSProtocol.decodeBinaryInt16(buffer, offset + AHRSProtocol.AHRS_UPDATE_RAW_MAG_Y_VALUE_INDEX)
            update.raw_mag_z = AHRSProtocol.AHRSProtocol.decodeBinaryInt16(buffer, offset + AHRSProtocol.AHRS_UPDATE_RAW_MAG_Z_VALUE_INDEX)
            update.quat_w = AHRSProtocol.decodeBinaryInt16(buffer, offset + AHRSProtocol.AHRS_UPDATE_QUAT_W_VALUE_INDEX)
            update.quat_x = AHRSProtocol.decodeBinaryInt16(buffer, offset + AHRSProtocol.AHRS_UPDATE_QUAT_X_VALUE_INDEX)
            update.quat_y = AHRSProtocol.decodeBinaryInt16(buffer, offset + AHRSProtocol.AHRS_UPDATE_QUAT_Y_VALUE_INDEX)
            update.quat_z = AHRSProtocol.decodeBinaryInt16(buffer, offset + AHRSProtocol.AHRS_UPDATE_QUAT_Z_VALUE_INDEX)
            update.barometric_pressure = AHRSProtocol.decodeProtocol1616Float(buffer, offset + AHRSProtocol.AHRS_UPDATE_BARO_PRESSURE_VALUE_INDEX)
            update.baro_temp = AHRSProtocol.decodeProtocolSignedHundredthsFloat(buffer, offset + AHRSProtocol.AHRS_UPDATE_BARO_TEMP_VAUE_INDEX)
            update.op_status = buffer[AHRSProtocol.AHRS_UPDATE_OPSTATUS_VALUE_INDEX]
            update.sensor_status = buffer[AHRSProtocol.AHRS_UPDATE_SENSOR_STATUS_VALUE_INDEX]
            update.cal_status = buffer[AHRSProtocol.AHRS_UPDATE_CAL_STATUS_VALUE_INDEX]
            update.selftest_status = buffer[AHRSProtocol.AHRS_UPDATE_SELFTEST_STATUS_VALUE_INDEX]
            return AHRSProtocol.AHRS_UPDATE_MESSAGE_LENGTH
        return 0

    @staticmethod
    def decodeAHRSPosUpdate(buffer, offset, length, pos_update):
        if (length < AHRSProtocol.AHRSPOS_UPDATE_MESSAGE_LENGTH):
            return 0
        if ((buffer[offset+0] == AHRSProtocol.PACKET_START_CHAR) and (buffer[offset+1] == AHRSProtocol.BINARY_PACKET_INDICATOR_CHAR) and (buffer[offset+2] == AHRSProtocol.AHRSPOS_UPDATE_MESSAGE_LENGTH - 2) and (buffer[offset+3] == AHRSProtocol.MSGID_AHRSPOS_UPDATE)):
            if (not AHRSProtocol.verifyChecksum(buffer, offset, AHRSProtocol.AHRSPOS_UPDATE_MESSAGE_CHECKSUM_INDEX)):
                return 0
            pos_update.yaw = AHRSProtocol.decodeProtocolSignedHundredthsFloat(buffer, offset + AHRSProtocol.AHRSPOS_UPDATE_YAW_VALUE_INDEX)
            pos_update.pitch = AHRSProtocol.decodeProtocolSignedHundredthsFloat(buffer, offset + AHRSProtocol.AHRSPOS_UPDATE_ROLL_VALUE_INDEX)  # FIXME - in original code
            pos_update.roll = AHRSProtocol.decodeProtocolSignedHundredthsFloat(buffer, offset + AHRSProtocol.AHRSPOS_UPDATE_PITCH_VALUE_INDEX)  # FIXME - in original code
            pos_update.compass_heading = AHRSProtocol.decodeProtocolUnsignedHundredthsFloat(buffer, offset + AHRSProtocol.AHRSPOS_UPDATE_HEADING_VALUE_INDEX)
            pos_update.altitude = AHRSProtocol.decodeProtocol1616Float(buffer, offset + AHRSProtocol.AHRSPOS_UPDATE_ALTITUDE_VALUE_INDEX)
            pos_update.fused_heading = AHRSProtocol.decodeProtocolUnsignedHundredthsFloat(buffer, offset + AHRSProtocol.AHRSPOS_UPDATE_FUSED_HEADING_VALUE_INDEX)
            pos_update.linear_accel_x = AHRSProtocol.decodeProtocolSignedThousandthsFloat(buffer, offset + AHRSProtocol.AHRSPOS_UPDATE_LINEAR_ACCEL_X_VALUE_INDEX)
            pos_update.linear_accel_y = AHRSProtocol.decodeProtocolSignedThousandthsFloat(buffer, offset + AHRSProtocol.AHRSPOS_UPDATE_LINEAR_ACCEL_Y_VALUE_INDEX)
            pos_update.linear_accel_z = AHRSProtocol.decodeProtocolSignedThousandthsFloat(buffer, offset + AHRSProtocol.AHRSPOS_UPDATE_LINEAR_ACCEL_Z_VALUE_INDEX)
            pos_update.vel_x = AHRSProtocol.decodeProtocol1616Float(buffer, offset + AHRSProtocol.AHRSPOS_UPDATE_VEL_X_VALUE_INDEX)
            pos_update.vel_y = AHRSProtocol.decodeProtocol1616Float(buffer, offset + AHRSProtocol.AHRSPOS_UPDATE_VEL_Y_VALUE_INDEX)
            pos_update.vel_z = AHRSProtocol.decodeProtocol1616Float(buffer, offset + AHRSProtocol.AHRSPOS_UPDATE_VEL_Z_VALUE_INDEX)
            pos_update.disp_x = AHRSProtocol.decodeProtocol1616Float(buffer, offset + AHRSProtocol.AHRSPOS_UPDATE_DISP_X_VALUE_INDEX)
            pos_update.disp_y = AHRSProtocol.decodeProtocol1616Float(buffer, offset + AHRSProtocol.AHRSPOS_UPDATE_DISP_Y_VALUE_INDEX)
            pos_update.disp_z = AHRSProtocol.decodeProtocol1616Float(buffer, offset + AHRSProtocol.AHRSPOS_UPDATE_DISP_Z_VALUE_INDEX)
            pos_update.mpu_temp = AHRSProtocol.decodeProtocolSignedHundredthsFloat(buffer, offset + AHRSProtocol.AHRSPOS_UPDATE_MPU_TEMP_VAUE_INDEX)
            pos_update.quat_w = AHRSProtocol.decodeBinaryInt16(buffer, offset + AHRSProtocol.AHRSPOS_UPDATE_QUAT_W_VALUE_INDEX)
            pos_update.quat_x = AHRSProtocol.decodeBinaryInt16(buffer, offset + AHRSProtocol.AHRSPOS_UPDATE_QUAT_X_VALUE_INDEX)
            pos_update.quat_y = AHRSProtocol.decodeBinaryInt16(buffer, offset + AHRSProtocol.AHRSPOS_UPDATE_QUAT_Y_VALUE_INDEX)
            pos_update.quat_z = AHRSProtocol.decodeBinaryInt16(buffer, offset + AHRSProtocol.AHRSPOS_UPDATE_QUAT_Z_VALUE_INDEX)
            pos_update.op_status = buffer[AHRSProtocol.AHRSPOS_UPDATE_OPSTATUS_VALUE_INDEX]
            pos_update.sensor_status = buffer[AHRSProtocol.AHRSPOS_UPDATE_SENSOR_STATUS_VALUE_INDEX]
            pos_update.cal_status = buffer[AHRSProtocol.AHRSPOS_UPDATE_CAL_STATUS_VALUE_INDEX]
            pos_update.selftest_status = buffer[AHRSProtocol.AHRSPOS_UPDATE_SELFTEST_STATUS_VALUE_INDEX]
            return AHRSProtocol.AHRSPOS_UPDATE_MESSAGE_LENGTH
        return 0

    # Mag Cal, Tuning Variable, or Board ID Retrieval Request
    @staticmethod
    def encodeDataGetRequest(buffer, type, var_id):
        # Header
        buffer[0] = AHRSProtocol.PACKET_START_CHAR
        buffer[1] = AHRSProtocol.BINARY_PACKET_INDICATOR_CHAR
        buffer[2] = AHRSProtocol.DATA_REQUEST_MESSAGE_LENGTH - 2
        buffer[3] = AHRSProtocol.MSGID_DATA_REQUEST
        # Data
        buffer[AHRSProtocol.DATA_REQUEST_DATATYPE_VALUE_INDEX] = type
        buffer[AHRSProtocol.DATA_REQUEST_VARIABLEID_VALUE_INDEX] = var_id
        # Footer
        AHRSProtocol.encodeTermination(buffer, AHRSProtocol.DATA_REQUEST_MESSAGE_LENGTH, AHRSProtocol.DATA_REQUEST_MESSAGE_LENGTH - 4)
        return AHRSProtocol.DATA_REQUEST_MESSAGE_LENGTH

    # Mag Cal Data Storage Request
    @staticmethod
    def encodeMagCalDataSetRequest(buffer, d):
        # Header
        buffer[0] = AHRSProtocol.PACKET_START_CHAR
        buffer[1] = AHRSProtocol.BINARY_PACKET_INDICATOR_CHAR
        buffer[2] = AHRSProtocol.MAG_CAL_CMD_MESSAGE_LENGTH - 2
        buffer[3] = AHRSProtocol.MSGID_MAG_CAL_CMD

        # Data
        buffer[AHRSProtocol.MAG_CAL_DATA_ACTION_VALUE_INDEX] = d.action
        for i in range(3):
            AHRSProtocol.encodeBinaryInt16(d.mag_bias[i], buffer, AHRSProtocol.MAG_X_BIAS_VALUE_INDEX + (i * 2))

        for i in range(3):
            for j in range(3):
                AHRSProtocol.encodeProtocol1616Float(d.mag_xform[i][j], buffer, AHRSProtocol.MAG_XFORM_1_1_VALUE_INDEX + (i * 6) + (j * 2))

        AHRSProtocol.encodeProtocol1616Float(d.earth_mag_field_norm, buffer, AHRSProtocol.MAG_CAL_EARTH_MAG_FIELD_NORM_VALUE_INDEX)
        # Footer
        AHRSProtocol.encodeTermination(buffer, AHRSProtocol.MAG_CAL_CMD_MESSAGE_LENGTH, AHRSProtocol.MAG_CAL_CMD_MESSAGE_LENGTH - 4)
        return AHRSProtocol.MAG_CAL_CMD_MESSAGE_LENGTH

    # Mag Cal Data Retrieval Response
    @staticmethod
    def decodeMagCalDataGetResponse(buffer, offset, length, d):
        if (length < AHRSProtocol.MAG_CAL_CMD_MESSAGE_LENGTH):
            return 0
        if ((buffer[0] == AHRSProtocol.PACKET_START_CHAR) and (buffer[1] == AHRSProtocol.BINARY_PACKET_INDICATOR_CHAR) and (buffer[2] == AHRSProtocol.MAG_CAL_CMD_MESSAGE_LENGTH - 2) and (buffer[3] == AHRSProtocol.MSGID_MAG_CAL_CMD)):

            if(not AHRSProtocol.verifyChecksum(buffer, offset, AHRSProtocol.MAG_CAL_CMD_MESSAGE_CHECKSUM_INDEX)):
                return 0

            d.action = buffer[AHRSProtocol.MAG_CAL_DATA_ACTION_VALUE_INDEX]
            for i in range(3):
                d.mag_bias[i] = AHRSProtocol.decodeBinaryInt16(buffer, AHRSProtocol.MAG_X_BIAS_VALUE_INDEX + (i * 2))

            for i in range(3):
                for j in range(3):
                    d.mag_xform[i][j] = AHRSProtocol.decodeProtocol1616Float(buffer, AHRSProtocol.MAG_XFORM_1_1_VALUE_INDEX + (i * 6) + (j * 2))

            d.earth_mag_field_norm = AHRSProtocol.decodeProtocol1616Float(buffer, AHRSProtocol.MAG_CAL_EARTH_MAG_FIELD_NORM_VALUE_INDEX)
            return AHRSProtocol.MAG_CAL_CMD_MESSAGE_LENGTH
        return 0

    # Tuning Variable Storage Request
    @staticmethod
    def encodeTuningVarSetRequest(buffer, r):
        # Header
        buffer[0] = AHRSProtocol.PACKET_START_CHAR
        buffer[1] = AHRSProtocol.BINARY_PACKET_INDICATOR_CHAR
        buffer[2] = AHRSProtocol.FUSION_TUNING_CMD_MESSAGE_LENGTH - 2
        buffer[3] = AHRSProtocol.MSGID_FUSION_TUNING_CMD
        # Data
        buffer[AHRSProtocol.FUSION_TUNING_DATA_ACTION_VALUE_INDEX] = r.action
        buffer[AHRSProtocol.FUSION_TUNING_CMD_VAR_ID_VALUE_INDEX] = r.var_id
        AHRSProtocol.encodeProtocol1616Float(r.value, buffer, AHRSProtocol.FUSION_TUNING_CMD_VAR_VALUE_INDEX)
        # Footer
        AHRSProtocol.encodeTermination(buffer, AHRSProtocol.FUSION_TUNING_CMD_MESSAGE_LENGTH, AHRSProtocol.FUSION_TUNING_CMD_MESSAGE_LENGTH - 4)
        return AHRSProtocol.FUSION_TUNING_CMD_MESSAGE_LENGTH

    # Tuning Variable Retrieval Response
    @staticmethod
    def decodeTuningVarGetResponse(buffer, offset, length, r):
        if (length < AHRSProtocol.FUSION_TUNING_CMD_MESSAGE_LENGTH):
            return 0
        if ((buffer[0] == AHRSProtocol.PACKET_START_CHAR) and (buffer[1] == AHRSProtocol.BINARY_PACKET_INDICATOR_CHAR) and (buffer[2] == AHRSProtocol.FUSION_TUNING_CMD_MESSAGE_LENGTH - 2) and (buffer[3] == AHRSProtocol.MSGID_FUSION_TUNING_CMD)):
            if (not AHRSProtocol.verifyChecksum(buffer, offset, AHRSProtocol.FUSION_TUNING_CMD_MESSAGE_CHECKSUM_INDEX)):
                return 0

            # Data
            r.action = buffer[AHRSProtocol.FUSION_TUNING_DATA_ACTION_VALUE_INDEX]
            r.var_id = buffer[AHRSProtocol.FUSION_TUNING_CMD_VAR_ID_VALUE_INDEX]
            r.value = AHRSProtocol.decodeProtocol1616Float(buffer, AHRSProtocol.FUSION_TUNING_CMD_VAR_VALUE_INDEX)
            return AHRSProtocol.FUSION_TUNING_CMD_MESSAGE_LENGTH
        return 0

    @staticmethod
    def encodeIntegrationControlCmd(buffer, u):
        # Header
        buffer[0] = AHRSProtocol.PACKET_START_CHAR
        buffer[1] = AHRSProtocol.BINARY_PACKET_INDICATOR_CHAR
        buffer[2] = AHRSProtocol.INTEGRATION_CONTROL_CMD_MESSAGE_LENGTH - 2
        buffer[3] = AHRSProtocol.MSGID_INTEGRATION_CONTROL_CMD
        # Data
        buffer[AHRSProtocol.INTEGRATION_CONTROL_CMD_ACTION_INDEX] = u.action
        AHRSProtocol.encodeBinaryUint32(u.parameter, buffer, AHRSProtocol.INTEGRATION_CONTROL_CMD_PARAMETER_INDEX)
        # Footer
        AHRSProtocol.encodeTermination(buffer, AHRSProtocol.INTEGRATION_CONTROL_CMD_MESSAGE_LENGTH, AHRSProtocol.INTEGRATION_CONTROL_CMD_MESSAGE_LENGTH - 4)
        return AHRSProtocol.INTEGRATION_CONTROL_CMD_MESSAGE_LENGTH

    @staticmethod
    def decodeIntegrationControlResponse(buffer, offset, length, u):
        if (length < AHRSProtocol.INTEGRATION_CONTROL_RESP_MESSAGE_LENGTH):
            return 0
        if ((buffer[0] == AHRSProtocol.PACKET_START_CHAR) and (buffer[1] == AHRSProtocol.BINARY_PACKET_INDICATOR_CHAR) and (buffer[2] == AHRSProtocol.INTEGRATION_CONTROL_RESP_MESSAGE_LENGTH - 2) and (buffer[3] == AHRSProtocol.MSGID_INTEGRATION_CONTROL_RESP)):
            if (not AHRSProtocol.verifyChecksum(buffer, offset, AHRSProtocol.INTEGRATION_CONTROL_RESP_MESSAGE_CHECKSUM_INDEX)):
                return 0

            # Data
            u.action = buffer[AHRSProtocol.INTEGRATION_CONTROL_RESP_ACTION_INDEX]
            u.parameter = AHRSProtocol.decodeBinaryUint32(buffer, AHRSProtocol.INTEGRATION_CONTROL_RESP_PARAMETER_INDEX)
            return AHRSProtocol.INTEGRATION_CONTROL_RESP_MESSAGE_LENGTH
        return 0

    # MagCal or Tuning Variable Storage Response
    @staticmethod
    def decodeDataSetResponse(buffer, offset, length, d):
        if (length < AHRSProtocol.DATA_SET_RESPONSE_MESSAGE_LENGTH):
            return 0
        if ((buffer[0] == AHRSProtocol.PACKET_START_CHAR) and (buffer[1] == AHRSProtocol.BINARY_PACKET_INDICATOR_CHAR) and (buffer[2] == AHRSProtocol.DATA_SET_RESPONSE_MESSAGE_LENGTH - 2) and (buffer[3] == AHRSProtocol.MSGID_DATA_SET_RESPONSE)):
            if (not AHRSProtocol.verifyChecksum(buffer, offset, AHRSProtocol.DATA_SET_RESPONSE_MESSAGE_CHECKSUM_INDEX)):
                return 0

            d.data_type = buffer[AHRSProtocol.DATA_SET_RESPONSE_DATATYPE_VALUE_INDEX]
            d.var_id = buffer[AHRSProtocol.DATA_SET_RESPONSE_VARID_VALUE_INDEX]
            d.status = buffer[AHRSProtocol.DATA_SET_RESPONSE_STATUS_VALUE_INDEX]
            return AHRSProtocol.DATA_SET_RESPONSE_MESSAGE_LENGTH
        return 0

    # Board ID Retrieval Response
    @staticmethod
    def decodeBoardIDGetResponse(buffer, offset, length, id):
        if (length < AHRSProtocol.BOARD_IDENTITY_RESPONSE_MESSAGE_LENGTH):
            return 0
        if ((buffer[0] == AHRSProtocol.PACKET_START_CHAR) and (buffer[1] == AHRSProtocol.BINARY_PACKET_INDICATOR_CHAR) and (buffer[2] == AHRSProtocol.BOARD_IDENTITY_RESPONSE_MESSAGE_LENGTH - 2) and (buffer[3] == AHRSProtocol.MSGID_BOARD_IDENTITY_RESPONSE)):
            if (not AHRSProtocol.verifyChecksum(buffer, offset, AHRSProtocol.BOARD_IDENTITY_RESPONSE_CHECKSUM_INDEX)):
                return 0
            id.type = buffer[AHRSProtocol.BOARD_IDENTITY_BOARDTYPE_VALUE_INDEX]
            id.hw_rev = buffer[AHRSProtocol.BOARD_IDENTITY_HWREV_VALUE_INDEX]
            id.fw_ver_major = buffer[AHRSProtocol.BOARD_IDENTITY_FW_VER_MAJOR]
            id.fw_ver_minor = buffer[AHRSProtocol.BOARD_IDENTITY_FW_VER_MINOR]
            id.fw_revision = AHRSProtocol.decodeBinaryUint16(buffer, AHRSProtocol.BOARD_IDENTITY_FW_VER_REVISION_VALUE_INDEX)
            for i in range(12):
                id.unique_id[i] = buffer[AHRSProtocol.BOARD_IDENTITY_UNIQUE_ID_0 + i]
            return AHRSProtocol.BOARD_IDENTITY_RESPONSE_MESSAGE_LENGTH
        return 0

    # protocol data is encoded little endian, convert to big endian format
    @staticmethod
    def decodeBinaryUint16(buffer, offset):
        lowbyte = (buffer[offset] & 0xff)
        highbyte = buffer[offset+1]
        highbyte <<= 8
        decoded_value = (highbyte + lowbyte)
        return decoded_value

    @staticmethod
    def encodeBinaryUint16(val, buffer, offset):
        buffer[offset+0] = (val & 0xFF)
        buffer[offset+1] = ((val >> 8) & 0xFF)

    @staticmethod
    def decodeBinaryUint32(buffer, offset):
        lowlowbyte = (int(buffer[offset]) & 0xff)
        lowhighbyte = (int(buffer[offset+1]) & 0xff)
        highlowbyte = (int(buffer[offset+2]) & 0xff)
        highhighbyte = (int(buffer[offset+3]))

        lowhighbyte <<= 8
        highlowbyte <<= 16
        highhighbyte <<= 24

        result = highhighbyte + highlowbyte + lowhighbyte + lowlowbyte
        return result

    @staticmethod
    def encodeBinaryUint32(val, buffer, offset):
        buffer[offset+0] = (val & 0xFF)
        buffer[offset+1] = ((val >> 8) & 0xFF)
        buffer[offset+2] = ((val >> 16) & 0xFF)
        buffer[offset+3] = ((val >> 24) & 0xFF)

    @staticmethod
    def decodeBinaryInt16(buffer, offset):
        return AHRSProtocol.decodeBinaryUint16(buffer, offset)

    @staticmethod
    def encodeBinaryInt16(val, buffer, offset):
        AHRSProtocol.encodeBinaryUint16(val, buffer, offset)

    # -327.68 to +327.68
    @staticmethod
    def decodeProtocolSignedHundredthsFloat(buffer, offset):
        signed_angle = float(AHRSProtocol.decodeBinaryUint16(buffer, offset))
        signed_angle /= 100
        return signed_angle

    @staticmethod
    def encodeProtocolSignedHundredthsFloat(input, buffer, offset):
        input_as_int = int(input * 100)
        AHRSProtocol.encodeBinaryInt16(input_as_int, buffer, offset)

    @staticmethod
    def encodeSignedHundredthsFloat(input):
        return int(input * 100)

    @staticmethod
    def encodeUnsignedHundredthsFloat(input):
        return int(input * 100)

    @staticmethod
    def encodeRatioFloat(input_ratio):
        return float(input_ratio * 32768)

    @staticmethod
    def encodeSignedThousandthsFloat(input):
        return float(input * 1000)

    # 0 to 655.35
    @staticmethod
    def decodeProtocolUnsignedHundredthsFloat(buffer, offset):
        uint16 = int(AHRSProtocol.decodeBinaryUint16(buffer, offset))
        if (uint16 < 0):
            uint16 = uint16 + 65536
        unsigned_float = float(uint16)
        unsigned_float /= 100
        return unsigned_float

    @staticmethod
    def encodeProtocolUnsignedHundredthsFloat(input, buffer, offset):
        input_as_uint = int(input * 100)
        AHRSProtocol.encodeBinaryUint16(input_as_uint, buffer, offset)

    # -32.768 to +32.768
    @staticmethod
    def decodeProtocolSignedThousandthsFloat(buffer, offset):
        signed_angle = float(AHRSProtocol.decodeBinaryUint16(buffer, offset))
        signed_angle /= 1000
        return signed_angle

    @staticmethod
    def encodeProtocolSignedThousandthsFloat(input, buffer, offset):
        input_as_int = int(input * 1000)
        AHRSProtocol.encodeBinaryInt16(input_as_int, buffer, offset)

    # In units of -1 to 1, multiplied by 16384
    @staticmethod
    def decodeProtocolRatio(buffer, offset):
        ratio = float(AHRSProtocol.decodeBinaryUint16(buffer, offset))
        ratio /= 32768
        return ratio

    @staticmethod
    def encodeProtocolRatio(ratio, buffer, offset):
        ratio *= 32768
        AHRSProtocol.encodeBinaryInt16(int(ratio), buffer, offset)

    # <int16>.<uint16> (-32768.9999 to 32767.9999)
    @staticmethod
    def decodeProtocol1616Float(buffer, offset):
        result = float(AHRSProtocol.decodeBinaryUint32(buffer, offset))
        result /= 65536
        return result

    @staticmethod
    def encodeProtocol1616Float(val, buffer, offset):
        val *= 65536
        int_val = int(val)
        AHRSProtocol.encodeBinaryUint32(int_val, buffer, offset)

    CRC7_POLY = 0x0091

    @staticmethod
    def getCRC(buffer, length):
        crc = 0

        for i in range(length):
            crc ^= int(0x00ff & buffer[i])
            for j in range(8):
                if((crc & 0x0001) != 0):
                    crc ^= AHRSProtocol.CRC7_POLY
                crc >>= 1
        return crc
