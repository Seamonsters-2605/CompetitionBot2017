import cv2, numpy, math, importlib, inspect, sys, os, time
from networktables import NetworkTables

def main():
    #do stuff
    cam = cv2.VideoCapture(0) # use the 2nd camera
    while True:
        ret, frame = cam.read()
        frame = cv2.cvtColor(frame, cv2.COLORBGR2GRAY)
        cv2.imshow('camera2', frame)
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()

