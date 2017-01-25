import cv2, numpy, math, importlib, inspect, sys, os, time
from networktables import NetworkTables

def main(pipeline, outputName):
    NetworkTables.initialize(server="roborio-2605-frc.local")
    table = NetworkTables.getTable("contours")
    
    cam = cv2.VideoCapture(0)
    os.system('v4l2-ctl -c exposure_auto=1 -c exposure_absolute=0'
              '-c brightness=0 -c contrast=100 -c saturation=47')
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

            if len(out) > 0:
                out = out[0]
                xCoords = [ ]
                yCoords = [ ]
                for point in out:
                    point = point[0]
                    xCoords.append(point[0])
                    yCoords.append(point[1])
                print(xCoords, yCoords)

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
