import cv2
import numpy
import math
import importlib
import inspect
import sys
import os
import time

def main(pipeline, outputName):
    cam = cv2.VideoCapture(0)
    os.system('v4l2-ctl -c exposure_auto=1 -c exposure_absolute=10')
    current_time = time.time()
    print("Press escape to exit")
    while True:
            last_time = current_time
            current_time = time.time()
            time_elapsed = current_time - last_time
            if time_elapsed == 0:
                fps = 0
            else:
                fps = 1.0/time_elapsed
            print("Time step:", round(time_elapsed,4), "FPS:", round(fps,3)) 

            ret_val, img = cam.read()
            pipeline.process(img)
            out = getattr(pipeline, outputName)
            cv2.imshow('camera', img)
            cv2.imshow('filter', out)

            if cv2.waitKey(1) == 27:
                    break

    cv2.destroyAllWindows()

if __name__ == '__main__':
    pipeline = None
    
    filterFile = sys.argv[1]
    module = importlib.import_module(filterFile)
    for name, obj in inspect.getmembers(module):
        if inspect.isclass(obj):
            if name != "Enum":
                pipeline = obj()

    if pipeline is None:
        print("Couldn't load pipeline")
        exit()

    names = [ ]
    i = 0
    for name, obj in inspect.getmembers(pipeline):
        if not name.startswith('_'):
            i += 1
            print(str(i) + ":", name)
            names.append(name)
    number = input("Which number? ")

    main(pipeline, names[int(number) - 1])