from quaternion import Quaternion
from timestamped_quaternion import TimestampedQuaternion


class TimestampedQuaternionHistory:
    def __init__(self):
        self.history = {}

    def add(self, w, x, y, z, new_timestamp):
        self.history[new_timestamp] = TimestampedQuaternion(Quaternion(w, x, y, z), new_timestamp)

    def get(self, requested_timestamp):
        if len(self.history) == 0:
            return None

        if requested_timestamp in self.history:
            match = self.history[requested_timestamp]
            return TimestampedQuaternion(match.w, match.x, match.y, match.z, requested_timestamp)
        else:

            keys = list(self.history.keys())
            keys.sort()

            if keys[0] > requested_timestamp or keys[-1] < requested_timestamp:
                return None

            for i, key in enumerate(keys):
                if key > requested_timestamp:
                    previous_timestamp = keys[i - 1]
                    next_timestamp = key
                    timestamp_delta = next_timestamp - previous_timestamp
                    requested_timestamp_offset = requested_timestamp - previous_timestamp
                    requested_timestamp_ratio = requested_timestamp_offset / timestamp_delta
                    interpolated_quaternion = Quaternion.slerp(self.history[previous_timestamp],
                                                               self.history[next_timestamp],
                                                               requested_timestamp_ratio)

                    match = TimestampedQuaternion(interpolated_quaternion.w,
                                                  interpolated_quaternion.x,
                                                  interpolated_quaternion.y,
                                                  interpolated_quaternion.z, requested_timestamp)
                    match.setInterpolated(True)
                    return match
            return None
