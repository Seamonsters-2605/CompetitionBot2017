import cv2, numpy, math, importlib, inspect, sys, os, time
from networktables import NetworkTables

def main(pipeline, outputName):
    NetworkTables.initialize(server="roborio-2605-frc.local")
    table = NetworkTables.getTable("contours")
    
    cam = cv2.VideoCapture(0)
    # use the raw_* values!
    os.system('v4l2-ctl '
              '-c brightness=30 '
              '-c contrast=10 '
              '-c saturation=94 '
			  '-c white_balance_temperature=4500 '
			  '-c sharpness=25 '
              '-c backlight_compensation=0 '
              '-c exposure_auto=1 '
              '-c exposure_absolute=5')
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

            xCoords = [ ]
            yCoords = [ ]
            if len(out) > 0:
                out = out[0]
                for point in out:
                    point = point[0]
                    xCoords.append(point[0])
                    yCoords.append(point[1])
            table.putNumberArray('x', xCoords)
            table.putNumberArray('y', yCoords)
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
