import cv2, numpy, math, importlib, inspect, sys, os, time
from networktables import NetworkTables

def main():
    #do stuff
    cam = cv2.VideoCapture(1) # use the 2nd camera
    while True:
        ret, frame = cam.read()
        #gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        cv2.imshow('camera2', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
             break
    cam.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()

