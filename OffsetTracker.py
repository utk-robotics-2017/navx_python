class OffsetTracker:
    def __init__(self, history_length):
        self.history_length = history_length
        self.value_history = [0 for i in range(history_length)]
        self.next_value_history_index = 0
        self.value_offset = 0.0

    def updateHistory(self, curr_value):
        if (self.next_value_history_index >= self.history_length):
            self.next_value_history_index = 0

        self.value_history[self.next_value_history_index] = curr_value
        self.next_value_history_index += 1

    def getAverageFromHistory(self):
        value_history_sum = 0.0
        for i in range(self.history_length):
            value_history_sum += self.value_history[i]
        value_history_avg = value_history_sum / self.history_length
        return value_history_avg

    def setOffset(self):
        self.value_offset = self.getAverageFromHistory()

    def getOffset(self):
        return self.value_offset

    def applyOffset(self, value):
        offseted_value = float(value - self.value_offset)
        if (offseted_value < -180):
            offseted_value += 360
        if (offseted_value > 180):
            offseted_value -= 360
        return offseted_value
