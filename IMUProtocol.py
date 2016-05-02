class IMUProtocol:
    PACKET_START_CHAR = '!'
    PROTOCOL_FLOAT_LENGTH = 7
    CHECKSUM_LENGTH = 2
    TERMINATOR_LENGTH = 2

    # Yaw/Pitch/Roll (YPR) Update Packet - e.g., !y[yaw][pitch][roll][compass_heading]
    MSGID_YPR_UPDATE = 'y'
    YPR_UPDATE_YAW_VALUE_INDEX = 2
    YPR_UPDATE_PITCH_VALUE_INDEX = 9
    YPR_UPDATE_ROLL_VALUE_INDEX = 16
    YPR_UPDATE_COMPASS_VALUE_INDEX = 23
    YPR_UPDATE_CHECKSUM_INDEX = 30
    YPR_UPDATE_TERMINATOR_INDEX = 32
    YPR_UPDATE_MESSAGE_LENGTH = 34

    # Quaternion Data Update Packet - e.g., !r[q1][q2][q3][q4][accelx][accely][accelz][magx][magy][magz]
    MSGID_QUATERNION_UPDATE = 'q'
    QUATERNION_UPDATE_MESSAGE_LENGTH = 53
    QUATERNION_UPDATE_QUAT1_VALUE_INDEX = 2
    QUATERNION_UPDATE_QUAT2_VALUE_INDEX = 4
    QUATERNION_UPDATE_QUAT3_VALUE_INDEX = 10
    QUATERNION_UPDATE_QUAT4_VALUE_INDEX = 14
    QUATERNION_UPDATE_ACCEL_X_VALUE_INDEX = 18
    QUATERNION_UPDATE_ACCEL_Y_VALUE_INDEX = 22
    QUATERNION_UPDATE_ACCEL_Z_VALUE_INDEX = 26
    QUATERNION_UPDATE_MAG_X_VALUE_INDEX = 30
    QUATERNION_UPDATE_MAG_Y_VALUE_INDEX = 34
    QUATERNION_UPDATE_MAG_Z_VALUE_INDEX = 38
    QUATERNION_UPDATE_TEMP_VALUE_INDEX = 42
    QUATERNION_UPDATE_CHECKSUM_INDEX = 49
    QUATERNION_UPDATE_TERMINATOR_INDEX = 51

    # Gyro/Raw Data Update packet - e.g., !g[gx][gy][gz][accelx][accely][accelz][magx][magy][magz][temp_c][cr][lf]
    MSGID_GYRO_UPDATE = 'g'
    GYRO_UPDATE_GYRO_X_VALUE_INDEX = 2
    GYRO_UPDATE_GYRO_Y_VALUE_INDEX = 6
    GYRO_UPDATE_GYRO_Z_VALUE_INDEX = 10
    GYRO_UPDATE_ACCEL_X_VALUE_INDEX = 14
    GYRO_UPDATE_ACCEL_Y_VALUE_INDEX = 18
    GYRO_UPDATE_ACCEL_Z_VALUE_INDEX = 22
    GYRO_UPDATE_MAG_X_VALUE_INDEX = 26
    GYRO_UPDATE_MAG_Y_VALUE_INDEX = 30
    GYRO_UPDATE_MAG_Z_VALUE_INDEX = 34
    GYRO_UPDATE_TEMP_VALUE_INDEX = 38
    GYRO_UPDATE_CHECKSUM_INDEX = 42
    GYRO_UPDATE_TERMINATOR_INDEX = 44
    GYRO_UPDATE_MESSAGE_LENGTH = 46

    # EnableStream Command Packet - e.g., !S[stream type][checksum][cr][lf]
    MSGID_STREAM_CMD = 'S'
    STREAM_CMD_STREAM_TYPE_YPR = MSGID_YPR_UPDATE
    STREAM_CMD_STREAM_TYPE_QUATERNION = MSGID_QUATERNION_UPDATE
    STREAM_CMD_STREAM_TYPE_GYRO = MSGID_GYRO_UPDATE
    STREAM_CMD_STREAM_TYPE_INDEX = 2
    STREAM_CMD_UPDATE_RATE_HZ_INDEX = 3
    STREAM_CMD_CHECKSUM_INDEX = 5
    STREAM_CMD_TERMINATOR_INDEX = 7
    STREAM_CMD_MESSAGE_LENGTH = 9

    # EnableStream Response Packet - e.g., !s[stream type][gyro full scale range][accel full scale range][update rate hz][yaw_offset_degrees][flags][checksum][cr][lf]
    MSG_ID_STREAM_RESPONSE = 's'
    STREAM_RESPONSE_MESSAGE_LENGTH = 46
    STREAM_RESPONSE_STREAM_TYPE_INDEX = 2
    STREAM_RESPONSE_GYRO_FULL_SCALE_DPS_RANGE = 3
    STREAM_RESPONSE_ACCEL_FULL_SCALE_G_RANGE = 7
    STREAM_RESPONSE_UPDATE_RATE_HZ = 11
    STREAM_RESPONSE_YAW_OFFSET_DEGREES = 15
    STREAM_RESPONSE_QUAT1_OFFSET = 22
    STREAM_RESPONSE_QUAT2_OFFSET = 26
    STREAM_RESPONSE_QUAT3_OFFSET = 30
    STREAM_RESPONSE_QUAT4_OFFSET = 34
    STREAM_RESPONSE_FLAGS = 38
    STREAM_RESPONSE_CHECKSUM_INDEX = 42
    STREAM_RESPONSE_TERMINATOR_INDEX = 44

    STREAM_MSG_TERMINATION_CHAR = '\n'

    NAV6_FLAG_MASK_CALIBRATION_STATE = 0x03

    NAV6_CALIBRATION_STATE_WAIT = 0x00
    NAV6_CALIBRATION_STATE_ACCUMULATE = 0x01
    NAV6_CALIBRATION_STATE_COMPLETE = 0x02

    IMU_PROTOCOL_MAX_MESSAGE_LENGTH = QUATERNION_UPDATE_MESSAGE_LENGTH

    @staticmethod
    def encodeStreamCommand(protocol_buffer, stream_type, update_rate_hz):

        #Header
        protocol_buffer[0] = IMUProtocol.PACKET_START_CHAR
        protocol_buffer[1] = IMUProtocol.MSGID_STREAM_CMD

        # Data
        protocol_buffer[IMUProtocol.STREAM_CMD_STREAM_TYPE_INDEX] = stream_type
        IMUProtocol.byteToHex(update_rate_hz, protocol_buffer, IMUProtocol.STREAM_CMD_UPDATE_RATE_HZ_INDEX)

        # Footer
        IMUProtocol.encodeTermination(protocol_buffer, IMUProtocol.STREAM_CMD_MESSAGE_LENGTH, IMUProtocol.STREAM_CMD_MESSAGE_LENGTH - 4)

        return IMUProtocol.STREAM_CMD_MESSAGE_LENGTH

    @staticmethod
    def decodeStreamResponse(buffer, offset, length, r):

        if (length < IMUProtocol.STREAM_RESPONSE_MESSAGE_LENGTH):
            return 0
        if ((buffer[offset + 0] == IMUProtocol.PACKET_START_CHAR) and (buffer[offset + 1] == IMUProtocol.MSG_ID_STREAM_RESPONSE)):
            if (not IMUProtocol.verifyChecksum(buffer, offset, IMUProtocol.STREAM_RESPONSE_CHECKSUM_INDEX)):
                return 0

            r.stream_type = buffer[offset + 2]
            r.gyro_fsr_dps = IMUProtocol.decodeProtocolUint16(buffer, offset + IMUProtocol.STREAM_RESPONSE_GYRO_FULL_SCALE_DPS_RANGE)
            r.accel_fsr_g = IMUProtocol.decodeProtocolUint16(buffer, offset + IMUProtocol.STREAM_RESPONSE_ACCEL_FULL_SCALE_G_RANGE)
            r.update_rate_hz = IMUProtocol.decodeProtocolUint16(buffer, offset + IMUProtocol.STREAM_RESPONSE_UPDATE_RATE_HZ)
            r.yaw_offset_degrees = IMUProtocol.decodeProtocolFloat(buffer, offset + IMUProtocol.STREAM_RESPONSE_YAW_OFFSET_DEGREES)
            r.q1_offset = IMUProtocol.decodeProtocolUint16(buffer, offset + IMUProtocol.STREAM_RESPONSE_QUAT1_OFFSET)
            r.q2_offset = IMUProtocol.decodeProtocolUint16(buffer, offset + IMUProtocol.STREAM_RESPONSE_QUAT2_OFFSET)
            r.q3_offset = IMUProtocol.decodeProtocolUint16(buffer, offset + IMUProtocol.STREAM_RESPONSE_QUAT3_OFFSET)
            r.q4_offset = IMUProtocol.decodeProtocolUint16(buffer, offset + IMUProtocol.STREAM_RESPONSE_QUAT4_OFFSET)
            r.flags = IMUProtocol.decodeProtocolUint16(buffer, offset + IMUProtocol.STREAM_RESPONSE_FLAGS)

            return IMUProtocol.STREAM_RESPONSE_MESSAGE_LENGTH

        return 0

    @staticmethod
    def decodeStreamCommand(buffer, offset, length, c):
        if (length < IMUProtocol.STREAM_CMD_MESSAGE_LENGTH):
            return 0
        if ((buffer[offset + 0] == '!') and (buffer[offset + 1] == IMUProtocol.MSGID_STREAM_CMD)):
            if (not IMUProtocol.verifyChecksum(buffer, offset, IMUProtocol.STREAM_CMD_CHECKSUM_INDEX)):
                return 0

            c.stream_type = buffer[offset + IMUProtocol.STREAM_CMD_STREAM_TYPE_INDEX]
            return IMUProtocol.STREAM_CMD_MESSAGE_LENGTH

        return 0

    @staticmethod
    def decodeYPRUpdate(buffer, offset, length, u):
        if (length < IMUProtocol.YPR_UPDATE_MESSAGE_LENGTH):
            return 0
        if ((buffer[offset + 0] == '!') and (buffer[offset + 1] == 'y')):
            if (not IMUProtocol.verifyChecksum(buffer, offset, IMUProtocol.YPR_UPDATE_CHECKSUM_INDEX)):
                return 0

            u.yaw = IMUProtocol.decodeProtocolFloat(buffer, offset + IMUProtocol.YPR_UPDATE_YAW_VALUE_INDEX)
            u.pitch = IMUProtocol.decodeProtocolFloat(buffer, offset + IMUProtocol.YPR_UPDATE_PITCH_VALUE_INDEX)
            u.roll = IMUProtocol.decodeProtocolFloat(buffer, offset + IMUProtocol.YPR_UPDATE_ROLL_VALUE_INDEX)
            u.compass_heading = IMUProtocol.decodeProtocolFloat(buffer, offset + IMUProtocol.YPR_UPDATE_COMPASS_VALUE_INDEX)
            return IMUProtocol.YPR_UPDATE_MESSAGE_LENGTH

        return 0

    @staticmethod
    def decodeQuaternionUpdate(buffer, offset, length, u):
        if (length < IMUProtocol.QUATERNION_UPDATE_MESSAGE_LENGTH):
            return 0

        if ((buffer[offset + 0] == IMUProtocol.PACKET_START_CHAR) and(buffer[offset+1] == IMUProtocol.MSGID_QUATERNION_UPDATE)):
            if (not IMUProtocol.verifyChecksum(buffer, offset, IMUProtocol.QUATERNION_UPDATE_CHECKSUM_INDEX)):
                return 0

            u.q1 = IMUProtocol.decodeProtocolUint16(buffer, offset + IMUProtocol.QUATERNION_UPDATE_QUAT1_VALUE_INDEX)
            u.q2 = IMUProtocol.decodeProtocolUint16(buffer, offset + IMUProtocol.QUATERNION_UPDATE_QUAT2_VALUE_INDEX)
            u.q3 = IMUProtocol.decodeProtocolUint16(buffer, offset + IMUProtocol.QUATERNION_UPDATE_QUAT3_VALUE_INDEX)
            u.q4 = IMUProtocol.decodeProtocolUint16(buffer, offset + IMUProtocol.QUATERNION_UPDATE_QUAT4_VALUE_INDEX)
            u.accel_x = IMUProtocol.decodeProtocolUint16(buffer, offset + IMUProtocol.QUATERNION_UPDATE_ACCEL_X_VALUE_INDEX)
            u.accel_y = IMUProtocol.decodeProtocolUint16(buffer, offset + IMUProtocol.QUATERNION_UPDATE_ACCEL_Y_VALUE_INDEX)
            u.accel_z = IMUProtocol.decodeProtocolUint16(buffer, offset + IMUProtocol.QUATERNION_UPDATE_ACCEL_Z_VALUE_INDEX)
            u.mag_x = IMUProtocol.decodeProtocolUint16(buffer, offset + IMUProtocol.QUATERNION_UPDATE_MAG_X_VALUE_INDEX)
            u.mag_y = IMUProtocol.decodeProtocolUint16(buffer, offset + IMUProtocol.QUATERNION_UPDATE_MAG_Y_VALUE_INDEX)
            u.mag_z = IMUProtocol.decodeProtocolUint16(buffer, offset + IMUProtocol.QUATERNION_UPDATE_MAG_Z_VALUE_INDEX)
            u.temp_c = IMUProtocol.decodeProtocolFloat(buffer, offset + IMUProtocol.QUATERNION_UPDATE_TEMP_VALUE_INDEX)
            return IMUProtocol.QUATERNION_UPDATE_MESSAGE_LENGTH

        return 0

    @classmethod
    def decodeGyroUpdate(buffer, offset, length, u):
        if (length < IMUProtocol.GYRO_UPDATE_MESSAGE_LENGTH):
            return 0

        if ((buffer[offset + 0] == IMUProtocol.PACKET_START_CHAR) and (buffer[offset + 1] == IMUProtocol.MSGID_GYRO_UPDATE)):
            if (not IMUProtocol.verifyChecksum(buffer, offset, IMUProtocol.GYRO_UPDATE_CHECKSUM_INDEX)):
                return 0

            u.gyro_x = IMUProtocol.decodeProtocolUint16(buffer, offset + IMUProtocol.GYRO_UPDATE_GYRO_X_VALUE_INDEX)
            u.gyro_y = IMUProtocol.decodeProtocolUint16(buffer, offset + IMUProtocol.GYRO_UPDATE_GYRO_Y_VALUE_INDEX)
            u.gyro_z = IMUProtocol.decodeProtocolUint16(buffer, offset + IMUProtocol.GYRO_UPDATE_GYRO_Z_VALUE_INDEX)
            u.accel_x = IMUProtocol.decodeProtocolUint16(buffer, offset + IMUProtocol.GYRO_UPDATE_ACCEL_X_VALUE_INDEX)
            u.accel_y = IMUProtocol.decodeProtocolUint16(buffer, offset + IMUProtocol.GYRO_UPDATE_ACCEL_Y_VALUE_INDEX)
            u.accel_z = IMUProtocol.decodeProtocolUint16(buffer, offset + IMUProtocol.GYRO_UPDATE_ACCEL_Z_VALUE_INDEX)
            u.mag_x = IMUProtocol.decodeProtocolUint16(buffer, offset + IMUProtocol.GYRO_UPDATE_MAG_X_VALUE_INDEX)
            u.mag_y = IMUProtocol.decodeProtocolUint16(buffer, offset + IMUProtocol.GYRO_UPDATE_MAG_Y_VALUE_INDEX)
            u.mag_z = IMUProtocol.decodeProtocolUint16(buffer, offset + IMUProtocol.GYRO_UPDATE_MAG_Z_VALUE_INDEX)
            u.temp_c = IMUProtocol.decodeProtocolUnsignedHundredthsFloat(buffer, offset + IMUProtocol.GYRO_UPDATE_TEMP_VALUE_INDEX)
            return IMUProtocol.GYRO_UPDATE_MESSAGE_LENGTH

        return 0

    @staticmethod
    def encodeTermination(buffer, total_length, content_length):
        if ((total_length >= (IMUProtocol.CHECKSUM_LENGTH + IMUProtocol.TERMINATOR_LENGTH)) and (total_length >= content_length + (IMUProtocol.CHECKSUM_LENGTH + IMUProtocol.TERMINATOR_LENGTH))):
            # Checksum
            checksum = 0
            for i in range(content_length):
                checksum = checksum + buffer[i]
        # convert checksum to two ascii bytes

            IMUProtocol.byteToHex(checksum, buffer, content_length)
            # Message Terminator
            buffer[content_length + IMUProtocol.CHECKSUM_LENGTH + 0] = '\r'
            buffer[content_length + IMUProtocol.CHECKSUM_LENGTH + 1] = '\n'

    hexArray = bytearray([i for i in range(15)])

    @staticmethod
    def byteToHex(thebyte, dest, offset):
        v = thebyte & 0xFF
        dest[offset + 0] = IMUProtocol.hexArray[v >> 4]
        dest[offset + 1] = IMUProtocol.hexArray[v & 0x0F]

    @staticmethod
    def decodeProtocolUint16(uint16_string, offset):
        decoded_uint16 = 0
        shift_left = 12
        for i in range(offset + 0, offset + 4):
            digit = (uint16_string[i] - '0' if uint16_string[i] <= '9' else ((uint16_string[i] - 'A') + 10))
            decoded_uint16 = decoded_uint16 + (digit << shift_left)
            shift_left = shift_left - 4

        return decoded_uint16

    # 0 to 655.35
    @staticmethod
    def decodeProtocolUnsignedHundredthsFloat(uint8_unsigned_hundredths_float, offset):
        unsigned_float = float(IMUProtocol.decodeProtocolUint16(uint8_unsigned_hundredths_float, offset))
        unsigned_float /= 100
        return unsigned_float

    @staticmethod
    def verifyChecksum(buffer, offset, content_length):
        # Calculate Checksum
        checksum = 0
        for i in range(content_length):
            checksum = checksum + buffer[offset + i]

        # Decode Checksum
        decoded_checksum = IMUProtocol.decodeUint8(buffer, offset + content_length)

        return (checksum == decoded_checksum)

    @staticmethod
    def decodeUint8(checksum, offset):
        first_digit = (checksum[0 + offset] - '0' if checksum[0 + offset] <= '9' else ((checksum[0 + offset] - 'A') + 10))
        second_digit = (checksum[1 + offset] - '0' if checksum[1 + offset] <= '9' else ((checksum[1 + offset] - 'A') + 10))
        decoded_checksum = ((first_digit * 16) + second_digit)
        return decoded_checksum

    @staticmethod
    def decodeProtocolFloat(buffer, offset):
        float_string = buffer[offset: offset + IMUProtocol.PROTOCOL_FLOAT_LENGTH].decode("utf-8")
        return float(float_string)
