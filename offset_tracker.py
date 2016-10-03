from collections import deque


class OffsetTracker:
    def __init__(self, history_length):
        self.value_history = deque([0] * history_length, maxlen=history_length)
        self.value_offset = 0

    def update_history(self, value):
        self.value_history.append(value)

    def get_average_from_history(self):
        return sum(self.value_history) / float(len(self.value_history))

    def set_offset(self):
        self.value_offset = self.get_average_from_history()

    def get_offset(self):
        return self.value_offset

    def apply_offset(self, value):
        offseted_value = (value - self.value_offset)
        if offseted_value < -180:
            offseted_value += 360

        if offseted_value > 180:
            offseted_value -= 360

        return offseted_value
