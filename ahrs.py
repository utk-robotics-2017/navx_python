from threading import Thread, Lock
from i2c_io import I2C_IO
from ahrs_protocol import AHRSProtocol
from continuous_angle_tracker import ContinuousAngleTracker
from offset_tracker import OffsetTracker
from inertial_data_integrator import InertialDataIntegrator
from timestamped_quaternion_history import TimestampedQuaternionHistory
from process_commands import ProcessCommands
from units import *

import time

import logging
from ourlogging import setup_logging
setup_logging(__file__)
logger = logging.getLogger(__name__)


class get_ahrs:
    '''
        Controlled environment for reading from the NavX.
    '''
    def __init__(self, update_rate_hz=None):
        self.update_rate_hz = update_rate_hz

    def __enter__(self):
        if self.update_rate_hz is None:
            self.navx = AHRS(True)
        else:
            self.navx = AHRS(True, self.update_rate_hz)
        return self.navx

    def __exit__(self, type, value, traceback):
        self.navx.free()


class AHRS:
    '''
    The AHRS class provides an interface to AHRS capabilities
    of the KauaiLabs navX Robotics Navigation Sensor via I2C
    communications interfaces on the Raspberry Pi.

    The AHRS class enables access to basic connectivity and state information,
    as well as key 6-axis and 9-axis orientation information (yaw, pitch, roll,
    compass heading, fused (9-axis) heading and magnetic disturbance detection.

    Additionally, the ARHS class also provides access to extended information
    including linear acceleration, motion detection, rotation detection and sensor
    temperature.

    If used with the navX Aero, the AHRS class also provides access to
    altitude, barometric pressure and pressure sensor temperature data

    .. note:: This implementation does not provide access to the NavX via
              a serial port
    '''

    NAVX_DEFAULT_UPDATE_RATE_HZ = 60
    YAW_HISTORY_LENGTH = 10
    DEFAULT_ACCEL_FSR_G = 2
    DEFAULT_GYRO_FSR_DPS = 2000
    QUATERNION_HISTORY_SECONDS = 5.0

    def __init__(self, start_immediately=False, update_rate_hz=NAVX_DEFAULT_UPDATE_RATE_HZ):
        # Processed Data
        self.yaw = 0
        self.pitch = 0
        self.roll = 0
        self.compass_heading = 0.0
        self.world_linear_accel_x = 0.0
        self.world_linear_accel_y = 0.0
        self.world_linear_accel_z = 0.0
        self.mpu_temp_c = 0.0
        self.fused_heading = 0.0
        self.altitude = 0.0
        self.baro_pressure = 0.0
        # self.is_moving = False
        # self.is_rotating = False
        self.baro_sensor_temp_c = 0.0
        # self.altitude_valid = False
        # self.is_magnetometer_calibrated = False
        # self.magnetic_disturbance = False
        self.quaternion_w = 0
        self.quaternion_x = 0
        self.quaternion_y = 0
        self.quaternion_z = 0

        # Integrated Data
        self.vel_x = 0
        self.vel_y = 0
        self.vel_z = 0
        self.disp_x = 0
        self.disp_y = 0
        self.disp_z = 0

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
        self.update_rate_hz = update_rate_hz
        self.accel_fsr_g = self.DEFAULT_ACCEL_FSR_G
        self.gyro_fsr_dps = self.DEFAULT_GYRO_FSR_DPS
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
        self.yaw_offset_tracker = OffsetTracker(self.YAW_HISTORY_LENGTH)
        self.quaternion_history = TimestampedQuaternionHistory()

        self.lock = Lock()
        self.io = I2C_IO(self, self.update_rate_hz)
        if start_immediately:
            self.start()

    def start(self):
        self.t = Thread(target=self.io.run, name="AHRS_I2C")
        self.t.start()

    def stop(self):
        self.running = False
        self.io.stop()

    def free(self):
        self.stop()

        self.t.join(timeout=5)
        if self.t.is_alive():
            self.t.terminate()

    def response(self, child_conn):
        self.running = True

        if not hasattr(self, 't') or self.t is None or self.t.is_alive() is False:
            self.start()

        while self.running:
            if child_conn.poll():
                cmd = child_conn.recv()
            else:
                time.sleep(.01)
                continue
            if cmd == ProcessCommands.TIME:
                child_conn.send(self.get_last_time_stamp())
            elif cmd == ProcessCommands.YAW:
                child_conn.send(self.get_yaw())
            elif cmd == ProcessCommands.PITCH:
                child_conn.send(self.get_pitch())
            elif cmd == ProcessCommands.ROLL:
                child_conn.send(self.get_roll())
            elif cmd == ProcessCommands.COMPASS_HEADING:
                child_conn.send(self.get_compass_heading())
            elif cmd == ProcessCommands.ZERO_YAW:
                self.zero_yaw()
                child_conn.send("ok")
            elif cmd == ProcessCommands.IS_CALIBRATING:
                child_conn.send(self.is_calibrating())
            elif cmd == ProcessCommands.IS_CONNECTED:
                child_conn.send(self.is_connected())
            elif cmd == ProcessCommands.BYTE_COUNT:
                child_conn.send(self.get_byte_count())
            elif cmd == ProcessCommands.UPDATE_COUNT:
                child_conn.send(self.get_update_count())
            elif cmd == ProcessCommands.WORLD_LINEAR_ACCEL_X:
                child_conn.send(self.get_world_linear_accel_x())
            elif cmd == ProcessCommands.WORLD_LINEAR_ACCEL_Y:
                child_conn.send(self.get_world_linear_accel_y())
            elif cmd == ProcessCommands.WORLD_LINEAR_ACCEL_Z:
                child_conn.send(self.get_world_linear_accel_z())
            elif cmd == ProcessCommands.BAROMETRIC_PRESSUE:
                child_conn.send(self.get_barometric_pressure())
            elif cmd == ProcessCommands.ALTITUDE:
                child_conn.send(self.get_altitude())
            elif cmd == ProcessCommands.FUSED_HEADING:
                child_conn.send(self.get_fused_heading())
            elif cmd == ProcessCommands.QUATERNION_W:
                child_conn.send(self.get_quaternion_w())
            elif cmd == ProcessCommands.QUATERNION_X:
                child_conn.send(self.get_quaternion_x())
            elif cmd == ProcessCommands.QUATERNION_Y:
                child_conn.send(self.get_quaternion_y())
            elif cmd == ProcessCommands.QUATERNION_Z:
                child_conn.send(self.get_quaternion_z())
            elif cmd == ProcessCommands.RESET_DISPLACEMENT:
                self.reset_displacement()
                child_conn.send("ok")
            elif cmd == ProcessCommands.VELOCITY_X:
                child_conn.send(self.get_velocity_x())
            elif cmd == ProcessCommands.VELOCITY_Y:
                child_conn.send(self.get_velocity_y())
            elif cmd == ProcessCommands.VELOCITY_Z:
                child_conn.send(self.get_velocity_z())
            elif cmd == ProcessCommands.DISPLACEMENT_X:
                child_conn.send(self.get_displacement_x())
            elif cmd == ProcessCommands.DISPLACEMENT_Y:
                child_conn.send(self.get_displacement_y())
            elif cmd == ProcessCommands.DISPLACEMENT_Z:
                child_conn.send(self.get_displacement_z())
            elif cmd == ProcessCommands.TEMP_C:
                child_conn.send(self.getTempC())
            elif cmd == ProcessCommands.IS_MOVING:
                child_conn.send(self.is_moving())
            elif cmd == ProcessCommands.IS_ROTATING:
                child_conn.send(self.is_rotating())
            elif cmd == ProcessCommands.ALTITUDE_VALID:
                child_conn.send(self.is_altitude_valid())
            elif cmd == ProcessCommands.IS_MAGNETOMETER_CALIBRATED:
                child_conn.send(self.is_magnetometer_calibrated())
            elif cmd == ProcessCommands.MAGNETIC_DISTURBANCE:
                child_conn.send(self.is_magnetic_disturbance())
            elif cmd == ProcessCommands.STOP:
                self.stop()
                break
            elif cmd == ProcessCommands.FREE:
                self.free()
                break
            else:
                logger.warn("Unknown Command: {}".format(cmd))
                child_conn.send("Unknown Command")

    # calculated properties
    @property
    def moving(self):
        return (self.sensor_status & AHRSProtocol.NAVX_SENSOR_STATUS_MOVING) != 0

    @property
    def rotating(self):
        return (self.sensor_status & AHRSProtocol.NAVX_SENSOR_STATUS_YAW_STABLE) == 0

    @property
    def altitude_valid(self):
        return (self.sensor_status & AHRSProtocol.NAVX_SENSOR_STATUS_ALTITUDE_VALID) != 0

    @property
    def magnetometer_calibrated(self):
        return (self.sensor_status & AHRSProtocol.NAVX_CAL_STATUS_MAG_CAL_COMPLETE) != 0

    @property
    def magnetic_disturbance(self):
        return (self.sensor_status & AHRSProtocol.NAVX_SENSOR_STATUS_MAG_DISTURBANCE) != 0

    def get_last_time_stamp(self):
        '''
            Returns the last time stamp reported by the sensor

            :return The last time stamp
        '''
        return Time(self.last_sensor_timestamp, Time.s)

    def get_pitch(self):
        '''
            Returns the current pitch value (in degrees, from -180 to 180)
            reported by the sensor.  Pitch is a measure of rotation around
            the X Axis.

            :return The current pitch value in degrees (-180 to 180).
        '''
        return Angle(self.pitch, Angle.degree)

    def get_roll(self):
        '''
            Returns the current roll value (in degrees, from -180 to 180)
            reported by the sensor.  Roll is a measure of rotation around
            the X Axis.

            :return The current roll value in degrees (-180 to 180).
        '''
        return Angle(self.roll, Angle.degree)

    def get_yaw(self):
        '''
            Returns the current yaw value (in degrees, from -180 to 180)
            reported by the sensor.  Yaw is a measure of rotation around
            the Z Axis (which is perpendicular to the earth).

            Note that the returned yaw value will be offset by a user-specified
            offset value; this user-specified offset value is set by
            invoking the zero_yaw() method.

            :return The current yaw value in degrees (-180 to 180).
        '''
        if (self._is_board_reset_supported()):
            return Angle(self.yaw, Angle.degree)
        else:
            yaw = self.yaw_offset_tracker.apply_offset(self.yaw)
            return Angle(yaw, Angle.degree)

    def get_compass_heading(self):
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
        return Angle(self.compass_heading, Angle.degree)

    def zero_yaw(self):
        '''
            Sets the user-specified yaw offset to the current
            yaw value reported by the sensor.

            This user-specified yaw offset is automatically
            subtracted from subsequent yaw values reported by
            the get_yaw() method.
        '''
        if (self._is_board_reset_supported()):
            self.io.zero_yaw()
        else:
            self.yaw_offset_tracker.setOffset()

    def is_calibrating(self):
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
        return not ((self.cal_status &
                     AHRSProtocol.NAVX_CAL_STATUS_IMU_CAL_STATE_MASK) ==
                    AHRSProtocol.NAVX_CAL_STATUS_IMU_CAL_COMPLETE)

    def is_connected(self):
        '''
            Indicates whether the sensor is currently connected
            to the host computer.  A connection is considered established
            whenever communication with the sensor has occurred recently.

            :returns: Returns true if a valid update has been recently received
                  from the sensor.
        '''
        return self.io.is_connected()

    def get_byte_count(self):
        '''
            Returns the count in bytes of data received from the
            sensor. This could can be useful for diagnosing
            connectivity issues.

            If the byte count is increasing, but the update count
            (see :meth:`get_update_count`) is not, this indicates a software
            misconfiguration.

            :returns: The number of bytes received from the sensor.
        '''
        return self.io.get_byte_count()

    def get_update_count(self):
        '''
            Returns the count of valid updates which have
            been received from the sensor.  This count should increase
            at the same rate indicated by the configured update rate.

            :returns: The number of valid updates received from the sensor.
        '''

        return self.io.get_update_count()

    def get_world_linear_accel_x(self):
        '''
            Returns the current linear acceleration in the X-axis (in G).

            World linear acceleration refers to raw acceleration data, which
            has had the gravity component removed, and which has been rotated to
            the same reference frame as the current yaw value.  The resulting
            value represents the current acceleration in the x-axis of the
            body (e.g., the robot) on which the sensor is mounted.

            :return Current world linear acceleration in the X-axis (in G).
        '''
        return Acceleration(self.world_linear_accel_x, Acceleration.G)

    def get_world_linear_accel_y(self):
        '''
            Returns the current linear acceleration in the Y-axis (in G).

            World linear acceleration refers to raw acceleration data, which
            has had the gravity component removed, and which has been rotated to
            the same reference frame as the current yaw value.  The resulting
            value represents the current acceleration in the Y-axis of the
            body (e.g., the robot) on which the sensor is mounted.

            :return Current world linear acceleration in the Y-axis (in G).
        '''
        return Acceleration(self.world_linear_accel_y, Acceleration.G)

    def get_world_linear_accel_z(self):
        '''
            Returns the current linear acceleration in the Z-axis (in G).

            World linear acceleration refers to raw acceleration data, which
            has had the gravity component removed, and which has been rotated to
            the same reference frame as the current yaw value.  The resulting
            value represents the current acceleration in the Z-axis of the
            body (e.g., the robot) on which the sensor is mounted.

            :return Current world linear acceleration in the Z-axis (in G).
        '''
        return Acceleration(self.world_linear_accel_z, Acceleration.G)

    def is_moving(self):
        '''
            Indicates if the sensor is currently detecting motion,
            based upon the X and Y-axis world linear acceleration values.
            If the sum of the absolute values of the X and Y axis exceed
            a "motion threshold", the motion state is indicated.

            :return Returns true if the sensor is currently detecting motion.
        '''
        return self.moving

    def is_rotating(self):
        '''
            Indicates if the sensor is currently detecting yaw rotation,
            based upon whether the change in yaw over the last second
            exceeds the "Rotation Threshold."

            Yaw Rotation can occur either when the sensor is rotating, or
            when the sensor is not rotating AND the current gyro calibration
            is insufficiently calibrated to yield the standard yaw drift rate.

            :return Returns true if the sensor is currently detecting motion.
        '''
        return self.rotating

    def get_barometric_pressure(self):
        '''
            Returns the current barometric pressure, based upon calibrated readings
            from the onboard pressure sensor.  This value is in units of millibar.

            NOTE:  This value is only valid for a navX Aero.  To determine
            whether this value is valid, see is_altitude_valid().

            :return Returns current barometric pressure (navX Aero only).
        '''
        return Pressure(self.baro_pressure, Pressure.mb)

    def get_altitude(self):
        '''
            Returns the current altitude, based upon calibrated readings
            from a barometric pressure sensor, and the currently-configured
            sea-level barometric pressure [navX Aero only].  This value is in units of meters.

            NOTE:  This value is only valid sensors including a pressure
            sensor.  To determine whether this value is valid, see
            is_altitude_valid().

            :return Returns current altitude in meters (as long as the sensor includes
            an installed on-board pressure sensor).
        '''
        return Distance(self.altitude, Distance.m)

    def is_altitude_valid(self):
        '''
            Indicates whether the current altitude (and barometric pressure) data is
            valid. This value will only be true for a sensor with an onboard
            pressure sensor installed.

            If this value is false for a board with an installed pressure sensor,
            this indicates a malfunction of the onboard pressure sensor.

            :return Returns true if a working pressure sensor is installed.
        '''
        return self.altitude_valid

    def get_fused_heading(self):
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
        return Angle(self.fused_heading, Angle.degree)

    def is_magnetic_disturbance(self):
        '''
            Indicates whether the current magnetic field strength diverges from the
            calibrated value for the earth's magnetic field by more than the currently-
            configured Magnetic Disturbance Ratio.

            This function will always return false if the sensor's magnetometer has
            not yet been calibrated; see is_magnetometer_calibrated().

            :return true if a magnetic disturbance is detected (or the magnetometer is uncalibrated).
        '''
        return self.magnetic_disturbance

    def is_magnetometer_calibrated(self):
        '''
            Indicates whether the magnetometer has been calibrated.

            Magnetometer Calibration must be performed by the user.

            Note that if this function does indicate the magnetometer is calibrated,
            this does not necessarily mean that the calibration quality is sufficient
            to yield valid compass headings.

            :return Returns true if magnetometer calibration has been performed.
        '''
        return self.magnetometer_calibrated

    # Unit Quaternions

    def get_quaternion_w(self):
        '''
            Returns the imaginary portion (W) of the Orientation Quaternion which
            fully describes the current sensor orientation with respect to the
            reference angle defined as the angle at which the yaw was last "zeroed".

            Each quaternion value (W,X,Y,Z) is expressed as a value ranging from -2
            to 2.  This total range (4) can be associated with a unit circle, since
            each circle is comprised of 4 PI Radians.

            For more information on Quaternions and their use, please see this
            <a href=https://en.wikipedia.org/wiki/Quaternions_and_spatial_rotation>definition</a>.
            :return Returns the imaginary portion (W) of the quaternion.
        '''
        return Angle(self.quaternion_w / 16384.0, Angle.Q)

    def get_quaternion_x(self):
        '''
            Returns the real portion (X axis) of the Orientation Quaternion which
            fully describes the current sensor orientation with respect to the
            reference angle defined as the angle at which the yaw was last "zeroed".

            Each quaternion value (W,X,Y,Z) is expressed as a value ranging from -2
            to 2.  This total range (4) can be associated with a unit circle, since
            each circle is comprised of 4 PI Radians.

            For more information on Quaternions and their use, please see this
            <a href=https://en.wikipedia.org/wiki/Quaternions_and_spatial_rotation>description</a>.
            :return Returns the real portion (X) of the quaternion.
        '''
        return Angle(self.quaternion_x / 16384.0, Angle.Q)

    def get_quaternion_y(self):
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
        return Angle(self.quaternion_y / 16384.0, Angle.Q)

    def get_quaternion_z(self):
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
        return Angle(self.quaternion_z / 16384.0, Angle.Q)

    def reset_displacement(self):
        '''
            Zeros the displacement integration variables.   Invoke this at the moment when
            integration begins.
        '''
        if (self._is_displacement_supported()):
            self.io.zero_displacement()
        else:
            self.integrator.reset_displacement()

    def update_displacement(self, accel_x_g, accel_y_g, update_rate_hz, is_moving):
        '''
            Each time new linear acceleration samples are received, this function should be invoked.
            This function transforms acceleration in G to meters/sec^2, then converts this value to
            Velocity in meters/sec (based upon velocity in the previous sample).  Finally, this value
            is converted to displacement in meters, and integrated.
            :return none.
        '''
        self.integrator.update_displacement(accel_x_g, accel_y_g, update_rate_hz, is_moving)

    def get_velocity_x(self):
        '''
            Returns the velocity (in meters/sec) of the X axis [Experimental].

            NOTE:  This feature is experimental.  Velocity measures rely on integration
            of acceleration values from MEMS accelerometers which yield "noisy" values.  The
            resulting velocities are not known to be very accurate.
            :return Current Velocity (in meters/sec).
        '''
        if self._is_displacement_supported():
            return Velocity(self.vel_x, Velocity.m_s)
        else:
            return Velocity(self.integrator.get_velocity_x(), Velocity.m_s)

    def get_velocity_y(self):
        '''
            Returns the velocity (in meters/sec) of the Y axis [Experimental].

            NOTE:  This feature is experimental.  Velocity measures rely on integration
            of acceleration values from MEMS accelerometers which yield "noisy" values.  The
            resulting velocities are not known to be very accurate.
            :return Current Velocity (in meters/sec).
        '''
        if self._is_displacement_supported():
            return Velocity(self.vel_y, Velocity.m_s)
        else:
            return Velocity(self.integrator.get_velocity_y(), Velocity.m_s)

    def get_velocity_z(self):
        '''
            Returns the velocity (in meters/sec) of the Z axis [Experimental].

            NOTE:  This feature is experimental.  Velocity measures rely on integration
            of acceleration values from MEMS accelerometers which yield "noisy" values.  The
            resulting velocities are not known to be very accurate.
            :return Current Velocity (in meters/sec).
        '''
        if self._is_displacement_supported():
            return Velocity(self.vel_z, Velocity.m_s)
        else:
            return Velocity(self.integrator.get_velocity_z(), Velocity.m_s)

    def get_displacement_x(self):
        '''
            Returns the displacement (in meters) of the X axis since reset_displacement()
            was last invoked [Experimental].

            NOTE:  This feature is experimental.  Displacement measures rely on double-integration
            of acceleration values from MEMS accelerometers which yield "noisy" values.  The
            resulting displacement are not known to be very accurate, and the amount of error
            increases quickly as time progresses.
            :return Displacement since last reset (in meters).
        '''
        if self._is_displacement_supported():
            return Distance(self.disp_x, Distance.m)
        else:
            return Distance(self.integrator.get_displacement_x(), Distance.m)

    def get_displacement_y(self):
        '''
            Returns the displacement (in meters) of the Y axis since reset_displacement()
            was last invoked [Experimental].

            NOTE:  This feature is experimental.  Displacement measures rely on double-integration
            of acceleration values from MEMS accelerometers which yield "noisy" values.  The
            resulting displacement are not known to be very accurate, and the amount of error
            increases quickly as time progresses.
            :return Displacement since last reset (in meters).
        '''
        if self._is_displacement_supported():
            return Distance(self.disp_y, Distance.m)
        else:
            return Distance(self.integrator.get_displacement_y(), Distance.m)

    def get_displacement_z(self):
        '''
            Returns the displacement (in meters) of the Z axis since reset_displacement()
            was last invoked [Experimental].

            NOTE:  This feature is experimental.  Displacement measures rely on double-integration
            of acceleration values from MEMS accelerometers which yield "noisy" values.  The
            resulting displacement are not known to be very accurate, and the amount of error
            increases quickly as time progresses.
            :return Displacement since last reset (in meters).
        '''
        if self._is_displacement_supported():
            return Distance(self.disp_z, Distance.m)
        else:
            return Distance(self.integrator.get_displacement_z(), Distance.m)

    def get_temp(self):
        '''
            Returns the current temperature (in degrees centigrade) reported by
            the sensor's gyro/accelerometer circuit.

            This value may be useful in order to perform advanced temperature-
            correction of raw gyroscope and accelerometer values.

            :return The current temperature (in degrees centigrade).
        '''
        return Temperature(self.mpu_temp_c, Temperature.C)

    def get_board_yaw_axis(self):
        '''
            Returns information regarding which sensor board axis (X,Y or Z) and
            direction (up/down) is currently configured to report Yaw (Z) angle
            values.   NOTE:  If the board firmware supports Omnimount, the board yaw
            axis/direction are configurable.

            For more information on Omnimount, please see:

            http://navx-mxp.kauailabs.com/navx-mxp/installation/omnimount/

            :returns: The currently-configured board yaw axis/direction as a
                  tuple of (up, axis). Up can be True/False, axis is 'x', 'y', or 'z')
        '''
        yaw_axis_info = self.capability_flags >> 3
        yaw_axis_info &= 7
        if yaw_axis_info == AHRSProtocol.OMNIMOUNT_DEFAULT:
            up = True
            yaw_axis = 'z'
        else:
            up = True if yaw_axis_info & 0x01 != 0 else False
            yaw_axis_info >>= 1
            if yaw_axis_info == 0:
                yaw_axis = 'x'
            elif yaw_axis_info == 1:
                yaw_axis = 'y'
            elif yaw_axis_info == 2:
                yaw_axis = 'z'

        return up, yaw_axis

    def get_firmware_version(self):
        '''
            Returns the version number of the firmware currently executing
            on the sensor.

            To update the firmware to the latest version, please see:

            http://navx-mxp.kauailabs.com/navx-mxp/support/updating-firmware/

            :return The firmware version in the format [MajorVersion].[MinorVersion]
        '''
        return '%s.%s' % (self.fw_ver_major, self.fw_ver_minor)

    def get_quaternion_at_time(self, requested_timestamp):
        return self.quaternion_history.get(requested_timestamp)

    def get_yaw_at_time(self, requested_timestamp):
        match = self.quaternion_history.get(requested_timestamp)
        if match is not None:
            return match.get_yaw()
        return 0.0

    def get_pitch_at_time(self, requested_timestamp):
        match = self.quaternion_history.get(requested_timestamp)
        if match is not None:
            return match.get_pitch()
        return 0.0

    def get_roll_at_time(self, requested_timestamp):
        match = self.quaternion_history.get(requested_timestamp)
        if match is not None:
            return match.get_roll()
        return 0.0

    # Internal API

    def _is_omni_mount_supported(self):
        return ((self.capability_flags & AHRSProtocol.NAVX_CAPABILITY_FLAG_OMNIMOUNT) != 0)

    def _is_board_reset_supported(self):
        return ((self.capability_flags & AHRSProtocol.NAVX_CAPABILITY_FLAG_YAW_RESET) != 0)

    def _is_displacement_supported(self):
        return ((self.capability_flags & AHRSProtocol.NAVX_CAPABILITY_FLAG_VEL_AND_DISP) != 0)

    def _is_ahrs_pos_time_stamp_supported(self):
        return ((self.capability_flags & AHRSProtocol.NAVX_CAPABILITY_FLAG_AHRSPOS_TS) != 0)

    def _set_yaw_pitch_roll(self, o):
        self.lock.acquire()
        self.__dict__.update(o.__dict__)
        self.lock.release()

    def _set_ahrs_pos_data(self, o, ts):
        self.lock.acquire()
        self.__dict__.update(o.__dict__)
        self.last_sensor_timestamp = ts
        if self.quaternion_history is not None:
            self.quaternion_history.add(self.quaternion_w, self.quaternion_x, self.quaternion_y,
                                        self.quaternion_z, self.last_sensor_timestamp)

        self.yaw_offset_tracker.update_history(self.yaw)
        self.yaw_angle_tracker.next_angle(self.get_yaw())
        self.lock.release()

    def _set_raw_data(self, o, ts):
        self.lock.acquire()
        self.__dict__.update(o.__dict__)
        self.last_sensor_timestamp = ts
        self.lock.release()

    def _set_ahrs_data(self, o, ts):
        self.lock.acquire()
        self.__dict__.update(o.__dict__)
        self.last_sensor_timestamp = ts
        if self.quaternion_history is not None:
            self.quaternion_history.add(self.quaternion_w, self.quaternion_x, self.quaternion_y,
                                        self.quaternion_z, self.last_sensor_timestamp)

        self.yaw_offset_tracker.update_history(self.yaw)
        self._update_displacement(o.world_linear_accel_x, o.world_linear_accel_y, self.update_rate_hz, self.moving)
        self.yaw_angle_tracker.next_angle(self.get_yaw())
        self.lock.acquire()

    def _set_board_id(self, o):
        self.lock.acquire()
        self.__dict__.update(o.__dict__)
        self.lock.release()

    def _set_board_state(self, o):
        self.lock.acquire()
        self.__dict__.update(o.__dict__)
        self.lock.release()

if __name__ == "__main__":
    with get_ahrs(10) as navx:
        time.sleep(1)
        while True:
            time.sleep(1)
            print("Time: {}, Yaw: {}, Pitch: {}, Roll: {}".format(navx.get_last_time_stamp().to(Time.s),
                                                                  navx.get_yaw().to(Angle.degree), navx.get_pitch().to(Angle.degree),
                                                                  navx.get_roll().to(Angle.degree)))
