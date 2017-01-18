import cv2
import numpy
import math
import importlib
import inspect
import sys

def main(pipeline, outputName):
    cam = cv2.VideoCapture(0)
    while True:
            ret_val, img = cam.read()
            pipeline.process(img)
            out = getattr(pipeline, outputName)
            cv2.imshow('my webcam', out)
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
    number = input("Which number?")

    main(pipeline, names[int(number) - 1])
