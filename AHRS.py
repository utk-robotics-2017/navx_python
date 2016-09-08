from enum import Enum
from multiprocessing import Process
from I2C_IO import I2C_IO
from AHRSProtocol import AHRSProtocol
from ContinuousAngleTracker import ContinuousAngleTracker
from OffsetTracker import OffsetTracker
from InertialDataIntegrator import InertialDataIntegrator


class BoardAxis(Enum):
    '''
    Identifies one of the three sensing axes on the navX sensor board.  Note that these axes are
    board-relative ("Board Frame"), and are not necessarily the same as the logical axes of the
    chassis on which the sensor is mounted.

    For more information on sensor orientation, please see the navX sensor <a href=http://navx-mxp.kauailabs.com/installation/orientation-2/>Orientation</a> page.
    '''
    kBoardAxisX = 0
    kBoardAxisY = 1
    kBoardAxisZ = 2


class BoardYawAxis:
    '''
    Indicates which sensor board axis is used as the "yaw" (gravity) axis.

    This selection may be modified via the <a href=http://navx-mxp.kauailabs.com/installation/omnimount/>Omnimount</a> feature.
    '''
    board_axis = BoardAxis.kBoardAxisX
    up = False


class BoardCapabilities:
    def __init__(self):
        self.capability_flags = 0

    def isOmniMountSupported(self):
        return ((self.capability_flags & AHRSProtocol.NAVX_CAPABILITY_FLAG_OMNIMOUNT) != 0)

    def isBoardYawResetSupported(self):
        return ((self.capability_flags & AHRSProtocol.NAVX_CAPABILITY_FLAG_YAW_RESET) != 0)

    def isDisplacementSupported(self):
        return ((self.capability_flags & AHRSProtocol.NAVX_CAPABILITY_FLAG_VEL_AND_DISP) != 0)


class AHRS:
    NAVX_DEFAULT_UPDATE_RATE_HZ = 60
    YAW_HISTORY_LENGTH = 10
    DEFAULT_ACCEL_FSR_G = 2
    DEFAULT_GYRO_FSR_DPS = 2000

    def __init__(self):
        # Processed Data
        self.yaw = 0.0
        self.pitch = 0.0
        self.roll = 0.0
        self.compass_heading = 0.0
        self.world_linear_accel_x = 0.0
        self.world_linear_accel_y = 0.0
        self.world_linear_accel_z = 0.0
        self.mpu_temp_c = 0.0
        self.fused_heading = 0.0
        self.altitude = 0.0
        self.baro_pressure = 0.0
        self.is_moving = False
        self.is_rotating = False
        self.baro_sensor_temp_c = 0.0
        self.altitude_valid = False
        self.is_magnetometer_calibrated = False
        self.magnetic_disturbance = False
        self.quaternionW = 0
        self.quaternionX = 0
        self.quaternionY = 0
        self.quaternionZ = 0

        # Integrated Data
        self.velocity = [0.0, 0.0, 0.0]
        self.displacement = [0.0, 0.0, 0.0]

        # Raw Data
        self.raw_gyro_x = 0
        self.raw_gyro_y = 0
        self.raw_gyro_z = 0
        self.raw_accel_x = 0
        self.raw_accel_y = 0
        self.raw_accel_z = 0
        self.cal_mag_x = 0
        self.cal_mag_y = 0
        self.cal_mag_z = 0

        # Configuration/Status
        self.update_rate_hz = 0
        self.accel_fsr_g = AHRS.DEFAULT_ACCEL_FSR_G
        self.gyro_fsr_dps = AHRS.DEFAULT_GYRO_FSR_DPS
        self.capability_flags = 0
        self.op_status = 0
        self.sensor_status = 0
        self.cal_status = 0
        self.selftest_status = 0

        # Board ID
        self.board_type = 0
        self.hw_rev = 0
        self.fw_ver_major = 0
        self.fw_ver_minor = 0

        self.last_sensor_timestamp = 0
        self.last_update_time = 0.0

        self.integrator = InertialDataIntegrator()
        self.yaw_angle_tracker = ContinuousAngleTracker()
        self.yaw_offset_tracker = OffsetTracker(AHRS.YAW_HISTORY_LENGTH)
        self.board_capabilities = BoardCapabilities()

        self.io = I2C_IO(self)


    def start(self):
        self.p = Process(target=self.io.run, args=())
        self.p.start()

    def stop(self):
        self.io.stop()

    '''
        Returns the current pitch value (in degrees, from -180 to 180)
        reported by the sensor.  Pitch is a measure of rotation around
        the X Axis.
        :return The current pitch value in degrees (-180 to 180).
    '''
    def getPitch(self):
        return self.pitch

    '''
        Returns the current roll value (in degrees, from -180 to 180)
        reported by the sensor.  Roll is a measure of rotation around
        the X Axis.
        :return The current roll value in degrees (-180 to 180).
    '''
    def getRoll(self):
        return self.roll

    '''
        Returns the current yaw value (in degrees, from -180 to 180)
        reported by the sensor.  Yaw is a measure of rotation around
        the Z Axis (which is perpendicular to the earth).

        Note that the returned yaw value will be offset by a user-specified
        offset value; this user-specified offset value is set by
        invoking the zeroYaw() method.
        :return The current yaw value in degrees (-180 to 180).
    '''
    def getYaw(self):
        if (self.board_capabilities.isBoardYawResetSupported()):
            return self.yaw
        else:
            return float(self.yaw_offset_tracker.applyOffset(self.yaw))

    '''
        Returns the current tilt-compensated compass heading
        value (in degrees, from 0 to 360) reported by the sensor.

        Note that this value is sensed by a magnetometer,
        which can be affected by nearby magnetic fields (e.g., the
        magnetic fields generated by nearby motors).

        Before using this value, ensure that (a) the magnetometer
        has been calibrated and (b) that a magnetic disturbance is
        not taking place at the instant when the compass heading
        was generated.
        :return The current tilt-compensated compass heading, in degrees (0-360).
    '''
    def getCompassHeading(self):
        return self.compass_heading

    '''
        Sets the user-specified yaw offset to the current
        yaw value reported by the sensor.

        This user-specified yaw offset is automatically
        subtracted from subsequent yaw values reported by
        the getYaw() method.
    '''
    def zeroYaw(self):
        if (self.board_capabilities.isBoardYawResetSupported(self.capability_flags)):
            #TODO: write zero yaw function (deal with serial)
            pass
        else:
            self.yaw_offset_tracker.setOffset()

    '''
        Returns true if the sensor is currently performing automatic
        gyro/accelerometer calibration.  Automatic calibration occurs
        when the sensor is initially powered on, during which time the
        sensor should be held still, with the Z-axis pointing up
        (perpendicular to the earth).

        NOTE:  During this automatic calibration, the yaw, pitch and roll
        values returned may not be accurate.

        Once calibration is complete, the sensor will automatically remove
        an internal yaw offset value from all reported values.

        :return Returns true if the sensor is currently automatically
        calibrating the gyro and accelerometer sensors.
    '''
    def isCalibrating(self):
        return not ((self.cal_status & AHRSProtocol.NAVX_CAL_STATUS_IMU_CAL_STATE_MASK) == AHRSProtocol.NAVX_CAL_STATUS_IMU_CAL_COMPLETE)

    '''
        Returns the current linear acceleration in the X-axis (in G).

        World linear acceleration refers to raw acceleration data, which
        has had the gravity component removed, and which has been rotated to
        the same reference frame as the current yaw value.  The resulting
        value represents the current acceleration in the x-axis of the
        body (e.g., the robot) on which the sensor is mounted.

        :return Current world linear acceleration in the X-axis (in G).
    '''
    def getWorldLinearAccelX(self):
        return self.world_linear_accel_x

    '''
        Returns the current linear acceleration in the Y-axis (in G).

        World linear acceleration refers to raw acceleration data, which
        has had the gravity component removed, and which has been rotated to
        the same reference frame as the current yaw value.  The resulting
        value represents the current acceleration in the Y-axis of the
        body (e.g., the robot) on which the sensor is mounted.

        :return Current world linear acceleration in the Y-axis (in G).
    '''
    def getWorldLinearAccelY(self):
        return self.world_linear_accel_y

    '''
        Returns the current linear acceleration in the Z-axis (in G).

        World linear acceleration refers to raw acceleration data, which
        has had the gravity component removed, and which has been rotated to
        the same reference frame as the current yaw value.  The resulting
        value represents the current acceleration in the Z-axis of the
        body (e.g., the robot) on which the sensor is mounted.

        :return Current world linear acceleration in the Z-axis (in G).
    '''
    def getWorldLinearAccelZ(self):
        return self.world_linear_accel_z

    '''
        Indicates if the sensor is currently detecting motion,
        based upon the X and Y-axis world linear acceleration values.
        If the sum of the absolute values of the X and Y axis exceed
        a "motion threshold", the motion state is indicated.

        :return Returns true if the sensor is currently detecting motion.
    '''
    def isMoving(self):
        return self.is_moving

    '''
        Indicates if the sensor is currently detecting yaw rotation,
        based upon whether the change in yaw over the last second
        exceeds the "Rotation Threshold."

        Yaw Rotation can occur either when the sensor is rotating, or
        when the sensor is not rotating AND the current gyro calibration
        is insufficiently calibrated to yield the standard yaw drift rate.

        :return Returns true if the sensor is currently detecting motion.
    '''
    def isRotating(self):
        return self.is_rotating

    '''
        Returns the current barometric pressure, based upon calibrated readings
        from the onboard pressure sensor.  This value is in units of millibar.

        NOTE:  This value is only valid for a navX Aero.  To determine
        whether this value is valid, see isAltitudeValid().
        :return Returns current barometric pressure (navX Aero only).
    '''
    def getBarometricPressure(self):
        return self.baro_pressure

    '''
        Returns the current altitude, based upon calibrated readings
        from a barometric pressure sensor, and the currently-configured
        sea-level barometric pressure [navX Aero only].  This value is in units of meters.

        NOTE:  This value is only valid sensors including a pressure
        sensor.  To determine whether this value is valid, see
        isAltitudeValid().

        :return Returns current altitude in meters (as long as the sensor includes
        an installed on-board pressure sensor).
    '''
    def getAltitude(self):
        return self.altitude

    '''
        Indicates whether the current altitude (and barometric pressure) data is
        valid. This value will only be true for a sensor with an onboard
        pressure sensor installed.

        If this value is false for a board with an installed pressure sensor,
        this indicates a malfunction of the onboard pressure sensor.

        :return Returns true if a working pressure sensor is installed.
    '''
    def isAltitudeValid(self):
        return self.altitude_valid

    '''
        Returns the "fused" (9-axis) heading.

        The 9-axis heading is the fusion of the yaw angle, the tilt-corrected
        compass heading, and magnetic disturbance detection.  Note that the
        magnetometer calibration procedure is required in order to
        achieve valid 9-axis headings.

        The 9-axis Heading represents the sensor's best estimate of current heading,
        based upon the last known valid Compass Angle, and updated by the change in the
        Yaw Angle since the last known valid Compass Angle.  The last known valid Compass
        Angle is updated whenever a Calibrated Compass Angle is read and the sensor
        has recently rotated less than the Compass Noise Bandwidth (~2 degrees).
        :return Fused Heading in Degrees (range 0-360)
    '''
    def getFusedHeading(self):
        return self.fused_heading

    '''
        Indicates whether the current magnetic field strength diverges from the
        calibrated value for the earth's magnetic field by more than the currently-
        configured Magnetic Disturbance Ratio.

        This function will always return false if the sensor's magnetometer has
        not yet been calibrated; see isMagnetometerCalibrated().
        :return true if a magnetic disturbance is detected (or the magnetometer is uncalibrated).
    '''
    def isMagneticDisturbance(self):
        return self.magnetic_disturbance

    '''
        Indicates whether the magnetometer has been calibrated.

        Magnetometer Calibration must be performed by the user.

        Note that if this function does indicate the magnetometer is calibrated,
        this does not necessarily mean that the calibration quality is sufficient
        to yield valid compass headings.

        :return Returns true if magnetometer calibration has been performed.
    '''
    def isMagnetometerCalibrated(self):
        return self.is_magnetometer_calibrated

    # Unit Quaternions

    '''
        Returns the imaginary portion (W) of the Orientation Quaternion which
        fully describes the current sensor orientation with respect to the
        reference angle defined as the angle at which the yaw was last "zeroed".

        Each quaternion value (W,X,Y,Z) is expressed as a value ranging from -2
        to 2.  This total range (4) can be associated with a unit circle, since
        each circle is comprised of 4 PI Radians.

        For more information on Quaternions and their use, please see this <a href=https://en.wikipedia.org/wiki/Quaternions_and_spatial_rotation>definition</a>.
        :return Returns the imaginary portion (W) of the quaternion.
    '''
    def getQuaternionW(self):
        return (float(self.quaternionW) / 16384.0)

    '''
        Returns the real portion (X axis) of the Orientation Quaternion which
        fully describes the current sensor orientation with respect to the
        reference angle defined as the angle at which the yaw was last "zeroed".

        Each quaternion value (W,X,Y,Z) is expressed as a value ranging from -2
        to 2.  This total range (4) can be associated with a unit circle, since
        each circle is comprised of 4 PI Radians.

        For more information on Quaternions and their use, please see this <a href=https://en.wikipedia.org/wiki/Quaternions_and_spatial_rotation>description</a>.
        :return Returns the real portion (X) of the quaternion.
    '''
    def getQuaternionX(self):
        return (float(self.quaternionX) / 16384.0)

    '''
        Returns the real portion (X axis) of the Orientation Quaternion which
        fully describes the current sensor orientation with respect to the
        reference angle defined as the angle at which the yaw was last "zeroed".

        Each quaternion value (W,X,Y,Z) is expressed as a value ranging from -2
        to 2.  This total range (4) can be associated with a unit circle, since
        each circle is comprised of 4 PI Radians.

        For more information on Quaternions and their use, please see:

        https://en.wikipedia.org/wiki/Quaternions_and_spatial_rotation

        :return Returns the real portion (X) of the quaternion.
    '''
    def getQuaternionY(self):
        return (float(self.quaternionY) / 16384.0)

    '''
        Returns the real portion (X axis) of the Orientation Quaternion which
        fully describes the current sensor orientation with respect to the
        reference angle defined as the angle at which the yaw was last "zeroed".

        Each quaternion value (W,X,Y,Z) is expressed as a value ranging from -2
        to 2.  This total range (4) can be associated with a unit circle, since
        each circle is comprised of 4 PI Radians.

        For more information on Quaternions and their use, please see:

        https://en.wikipedia.org/wiki/Quaternions_and_spatial_rotation

        :return Returns the real portion (X) of the quaternion.
    '''
    def getQuaternionZ(self):
        return (float(self.quaternionZ) / 16384.0)

    '''
        Zeros the displacement integration variables.   Invoke this at the moment when
        integration begins.
    '''
    def resetDisplacement(self):
        if (self.board_capabilities.isDisplacementSupported(self.capability_flags)):
            # TODO: serial zero displacement command
            pass
        else:
            self.integrator.resetDisplacement()

    '''
        Each time new linear acceleration samples are received, this function should be invoked.
        This function transforms acceleration in G to meters/sec^2, then converts this value to
        Velocity in meters/sec (based upon velocity in the previous sample).  Finally, this value
        is converted to displacement in meters, and integrated.
        :return none.
    '''
    def updateDisplacement(self, accel_x_g, accel_y_g, update_rate_hz, is_moving):
        self.integrator.updateDisplacement(accel_x_g, accel_y_g, update_rate_hz, is_moving)

    '''
        Returns the velocity (in meters/sec) of the X axis [Experimental].

        NOTE:  This feature is experimental.  Velocity measures rely on integration
        of acceleration values from MEMS accelerometers which yield "noisy" values.  The
        resulting velocities are not known to be very accurate.
        :return Current Velocity (in meters/squared).
    '''
    def getVelocityX(self):
        return (self.velocity[0] if self.board_capabilities.isDisplacementSupported(self.capability_flags) else self.integrator.getVelocityX())

    '''
        Returns the velocity (in meters/sec) of the Y axis [Experimental].

        NOTE:  This feature is experimental.  Velocity measures rely on integration
        of acceleration values from MEMS accelerometers which yield "noisy" values.  The
        resulting velocities are not known to be very accurate.
        :return Current Velocity (in meters/squared).
    '''
    def getVelocityY(self):
        return (self.velocity[1] if self.board_capabilities.isDisplacementSupported(self.capability_flags) else self.integrator.getVelocityY())

    '''
        Returns the velocity (in meters/sec) of the Z axis [Experimental].

        NOTE:  This feature is experimental.  Velocity measures rely on integration
        of acceleration values from MEMS accelerometers which yield "noisy" values.  The
        resulting velocities are not known to be very accurate.
        :return Current Velocity (in meters/squared).
    '''
    def getVelocityZ(self):
        return (self.velocity[2] if self.board_capabilities.isDisplacementSupported(self.capability_flags) else 0.0)

    '''
        Returns the displacement (in meters) of the X axis since resetDisplacement()
        was last invoked [Experimental].

        NOTE:  This feature is experimental.  Displacement measures rely on double-integration
        of acceleration values from MEMS accelerometers which yield "noisy" values.  The
        resulting displacement are not known to be very accurate, and the amount of error
        increases quickly as time progresses.
        :return Displacement since last reset (in meters).
    '''
    def getDisplacementX(self):
        return (self.displacement[0] if self.board_capabilities.isDisplacementSupported(self.capability_flags) else self.integrator.getDisplacementX())

    '''
        Returns the displacement (in meters) of the Y axis since resetDisplacement()
        was last invoked [Experimental].

        NOTE:  This feature is experimental.  Displacement measures rely on double-integration
        of acceleration values from MEMS accelerometers which yield "noisy" values.  The
        resulting displacement are not known to be very accurate, and the amount of error
        increases quickly as time progresses.
        :return Displacement since last reset (in meters).
    '''
    def getDisplacementY(self):
        return (self.displacement[1] if self.board_capabilities.isDisplacementSupported(self.capability_flags) else self.integrator.getDisplacementY())

    '''
        Returns the displacement (in meters) of the Z axis since resetDisplacement()
        was last invoked [Experimental].

        NOTE:  This feature is experimental.  Displacement measures rely on double-integration
        of acceleration values from MEMS accelerometers which yield "noisy" values.  The
        resulting displacement are not known to be very accurate, and the amount of error
        increases quickly as time progresses.
        :return Displacement since last reset (in meters).
    '''
    def getDisplacementZ(self):
        return (self.displacement[2] if self.board_capabilities.isDisplacementSupported(self.capability_flags) else 0.0)

    '''
        Returns the current temperature (in degrees centigrade) reported by
        the sensor's gyro/accelerometer circuit.

        This value may be useful in order to perform advanced temperature-
        correction of raw gyroscope and accelerometer values.

        :return The current temperature (in degrees centigrade).
    '''
    def getTempC(self):
        return self.mpu_temp_c

    '''
        Returns information regarding which sensor board axis (X,Y or Z) and
        direction (up/down) is currently configured to report Yaw (Z) angle
        values.   NOTE:  If the board firmware supports Omnimount, the board yaw
        axis/direction are configurable.

        For more information on Omnimount, please see:

        http://navx-mxp.kauailabs.com/navx-mxp/installation/omnimount/

        :return The currently-configured board yaw axis/direction.
    '''
    def getBoardYawAxis(self):
        yaw_axis = BoardYawAxis()
        yaw_axis_info = (self.capability_flags >> 3)
        yaw_axis_info &= 7
        if (yaw_axis_info == AHRSProtocol.OMNIMOUNT_DEFAULT):
            yaw_axis.up = True
            yaw_axis.board_axis = BoardAxis.kBoardAxisZ
        else:
            yaw_axis.up = ((yaw_axis_info & 0x01) != 0)
            yaw_axis_info >>= 1
            if(yaw_axis_info == 0):
                yaw_axis.board_axis = BoardAxis.kBoardAxisX
            elif(yaw_axis_info == 1):
                yaw_axis.board_axis = BoardAxis.kBoardAxisY
            elif(yaw_axis_info == 2):
                yaw_axis.board_axis = BoardAxis.kBoardAxisZ
        return yaw_axis

    '''
        Returns the version number of the firmware currently executing
        on the sensor.

        To update the firmware to the latest version, please see:

        http://navx-mxp.kauailabs.com/navx-mxp/support/updating-firmware/

        :return The firmware version in the format [MajorVersion].[MinorVersion]
    '''
    def getFirmwareVersion(self):
        version_number = float(self.fw_ver_major)
        version_number += (float(self.fw_ver_minor) / 10.0)
        fw_version = str(version_number)
        return fw_version

    def setYawPitchRoll(self, ypr_update, sensor_timestamp):
        self.yaw = ypr_update.yaw
        self.pitch = ypr_update.pitch
        self.roll = ypr_update.roll
        self.compass_heading = ypr_update.compass_heading
        self.last_sensor_timestamp = sensor_timestamp

    def setAHRSPosData(self, ahrs_update, sensor_timestamp):
        # Update base IMU class variables

        self.yaw = ahrs_update.yaw
        AHRS.this.pitch = ahrs_update.pitch
        AHRS.this.roll = ahrs_update.roll
        AHRS.this.compass_heading = ahrs_update.compass_heading
        self.yaw_offset_tracker.updateHistory(ahrs_update.yaw)

        # Update AHRS class variables

        # 9-axis data
        self.fused_heading = ahrs_update.fused_heading

        # Gravity-corrected linear acceleration (world-frame)
        self.world_linear_accel_x = ahrs_update.linear_accel_x
        self.world_linear_accel_y = ahrs_update.linear_accel_y
        self.world_linear_accel_z = ahrs_update.linear_accel_z

        # Gyro/Accelerometer Die Temperature
        self.mpu_temp_c = ahrs_update.mpu_temp

        # Barometric Pressure/Altitude
        self.altitude = ahrs_update.altitude
        self.baro_pressure = ahrs_update.barometric_pressure

        # Status/Motion Detection
        self.is_moving = ((ahrs_update.sensor_status & AHRSProtocol.NAVX_SENSOR_STATUS_MOVING) != 0)

        self.is_rotating = ((ahrs_update.sensor_status & AHRSProtocol.NAVX_SENSOR_STATUS_YAW_STABLE) != 0)

        self.altitude_valid = ((ahrs_update.sensor_status & AHRSProtocol.NAVX_SENSOR_STATUS_ALTITUDE_VALID) != 0)

        self.is_magnetometer_calibrated = ((ahrs_update.cal_status & AHRSProtocol.NAVX_CAL_STATUS_MAG_CAL_COMPLETE) != 0)

        self.magnetic_disturbance = ((ahrs_update.sensor_status & AHRSProtocol.NAVX_SENSOR_STATUS_MAG_DISTURBANCE) != 0)

        self.quaternionW = ahrs_update.quat_w
        self.quaternionX = ahrs_update.quat_x
        self.quaternionY = ahrs_update.quat_y
        self.quaternionZ = ahrs_update.quat_z

        self.last_sensor_timestamp = sensor_timestamp

        self.velocity[0] = ahrs_update.vel_x
        self.velocity[1] = ahrs_update.vel_y
        self.velocity[2] = ahrs_update.vel_z
        self.displacement[0] = ahrs_update.disp_x
        self.displacement[1] = ahrs_update.disp_y
        self.displacement[2] = ahrs_update.disp_z

        self.yaw_angle_tracker.nextAngle(self.getYaw())

    def setRawData(self, raw_data_update, sensor_timestamp):
        self.raw_gyro_x = raw_data_update.gyro_x
        self.raw_gyro_y = raw_data_update.gyro_y
        self.raw_gyro_z = raw_data_update.gyro_z
        self.raw_accel_x = raw_data_update.accel_x
        self.raw_accel_y = raw_data_update.accel_y
        self.raw_accel_z = raw_data_update.accel_z
        self.cal_mag_x = raw_data_update.mag_x
        self.cal_mag_y = raw_data_update.mag_y
        self.cal_mag_z = raw_data_update.mag_z
        self.mpu_temp_c = raw_data_update.temp_c

        self.last_sensor_timestamp = sensor_timestamp

    def setAHRSData(self, ahrs_update, sensor_timestamp):
        # Update base IMU class variables
        self.yaw = ahrs_update.yaw
        self.pitch = ahrs_update.pitch
        self.roll = ahrs_update.roll
        self.compass_heading = ahrs_update.compass_heading
        self.yaw_offset_tracker.updateHistory(ahrs_update.yaw)

        # Update AHRS class variables

        # 9-axis data
        self.fused_heading = ahrs_update.fused_heading

        # Gravity-corrected linear acceleration (world-frame)
        self.world_linear_accel_x = ahrs_update.linear_accel_x
        self.world_linear_accel_y = ahrs_update.linear_accel_y
        self.world_linear_accel_z = ahrs_update.linear_accel_z

        # Gyro/Accelerometer Die Temperature
        self.mpu_temp_c = ahrs_update.mpu_temp

        # Barometric Pressure/Altitude
        self.altitude = ahrs_update.altitude
        self.baro_pressure = ahrs_update.barometric_pressure

        # Magnetometer Data
        self.cal_mag_x = ahrs_update.cal_mag_x
        self.cal_mag_y = ahrs_update.cal_mag_y
        self.cal_mag_z = ahrs_update.cal_mag_z

        # Status/Motion Detection
        self.is_moving = ((ahrs_update.sensor_status & AHRSProtocol.NAVX_SENSOR_STATUS_MOVING) != 0)
        self.is_rotating = ((ahrs_update.sensor_status & AHRSProtocol.NAVX_SENSOR_STATUS_YAW_STABLE) != 0)
        self.altitude_valid = ((ahrs_update.sensor_status & AHRSProtocol.NAVX_SENSOR_STATUS_ALTITUDE_VALID) != 0)
        self.is_magnetometer_calibrated = ((ahrs_update.cal_status & AHRSProtocol.NAVX_CAL_STATUS_MAG_CAL_COMPLETE) != 0)
        self.magnetic_disturbance = ((ahrs_update.sensor_status & AHRSProtocol.NAVX_SENSOR_STATUS_MAG_DISTURBANCE) != 0)

        self.quaternionW = ahrs_update.quat_w
        self.quaternionX = ahrs_update.quat_x
        self.quaternionY = ahrs_update.quat_y
        self.quaternionZ = ahrs_update.quat_z

        self.last_sensor_timestamp = sensor_timestamp

        self.updateDisplacement(self.world_linear_accel_x, self.world_linear_accel_y, self.update_rate_hz, self.is_moving)

        self.yaw_angle_tracker.nextAngle(self.getYaw())

    def setBoardID(self, board_id):
        self.board_type = board_id.type
        self.hw_rev = board_id.hw_rev
        self.fw_ver_major = board_id.fw_ver_major
        self.fw_ver_minor = board_id.fw_ver_minor

    def setBoardState(self, board_state):
        self.update_rate_hz = board_state.update_rate_hz
        self.accel_fsr_g = board_state.accel_fsr_g
        self.gyro_fsr_dps = board_state.gyro_fsr_dps
        self.capability_flags = board_state.capability_flags
        self.op_status = board_state.op_status
        self.sensor_status = board_state.sensor_status
        self.cal_status = board_state.cal_status
        self.selftest_status = board_state.selftest_status
