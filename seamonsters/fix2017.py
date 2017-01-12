__author__ = "jacobvanthoog"

import wpilib

wpilib.CANTalon = wpilib.TalonSRX

def setFeedbackDevice(self, device):
    pass
wpilib.CANTalon.setFeedbackDevice = setFeedbackDevice

class FeedbackDevice:
    QuadEncoder = 0
    AnalogPot = 1
    AnalogEncoder = 2
    EncRising = 3
    EncFalling = 4
    CtreMagEncoder_Relative = 5
    CtreMagEncoder_Absolute = 6
    PulseWidth = 7
wpilib.CANTalon.FeedbackDevice = FeedbackDevice

def setPID(self, p, i, d, f):
    pass
wpilib.CANTalon.setPID = setPID
