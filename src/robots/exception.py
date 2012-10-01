# -*- coding: utf-8 -*-

class RobotError(Exception):
    def __init__(self, value):
        self.value = value
    def __str__(self):
        return repr(self.value)

class UnknownFrameError(RobotError):
    def __init__(self, value):
        self.value = value
    def __str__(self):
        return repr(self.value)
