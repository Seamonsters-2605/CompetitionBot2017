import cv2, numpy, math, importlib, inspect, sys, os, time
from networktables import NetworkTables

def main():
    #do stuff
    cam = cv2.VideoCapture(1) # use the 2nd camera
    while True:
        img = cam.grab()
        cv2.imshow('camera2', img)
        print("Things should be working")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()

