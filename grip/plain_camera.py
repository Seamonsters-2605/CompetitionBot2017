import cv2, numpy, math, importlib, inspect, sys, os, time
from networktables import NetworkTables

def main():
    #do stuff
    cam = cv2.VideoCapture(1) # use the 2nd camera
    while True:
        cv2.imshow('camera2', cam.read())
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()