from collections import deque

# enums in Python, thanks http://stackoverflow.com/questions/36932
def enum(*sequential, **named):
    enums = dict(zip(sequential, range(len(sequential))), **named)
    return type('Enum', (), enums)

class valuefilter:

    MAX_LENGTH=10

    def __init__(self, maxlen = MAX_LENGTH):
        self._vals = deque(maxlen = maxlen)

        self.lastval = 0.
        self.dirty = True

    def append(self, val):
        self.dirty = True
        self._vals.append(val)

    def get(self):

        if self.dirty:
            self.lastval = sum(self._vals) / len(self._vals)
            self.dirty = False

        return self.lastval
