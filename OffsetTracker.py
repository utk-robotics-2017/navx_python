from collections import deque

class OffsetTracker:
    def __init__(self, history_length):
        self.value_history = deque([0]*history_length, maxlen=history_length)
        self.value_offset = 0

    def updateHistory(self, curr_value):
        self.value_history.append(value)

    def getAverageFromHistory(self):
        return sum(self.value_history) / float(len(self.value_history))

    def setOffset(self):
        self.value_offset = self.getAverageFromHistory()

    def getOffset(self):
        return self.value_offset

    def applyOffset(self, value):
        offseted_value = (value - self.value_offset)
        if offseted_value < -180:
            offseted_value += 360

        if offseted_value > 180:
            offseted_value -= 360

        return offseted_value
