import cv2, numpy, math, os, time
from networktables import NetworkTables
import peg_filter_real

def main(pipeline):
    NetworkTables.initialize(server="roborio-2605-frc.local")
    table = NetworkTables.getTable("contours")
    
    cam = cv2.VideoCapture(0)
    # use the raw_* values!
    os.system('v4l2-ctl '
              '-c brightness=30 '
              '-c contrast=10 '
              '-c saturation=94 '
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
            out = pipeline.convex_hulls_output

            xCoords = [ ]
            yCoords = [ ]
            print(len(out), "contours")
            for contour in out:
                for point in contour:
                    point = point[0]
                    xCoords.append(point[0])
                    yCoords.append(point[1])
                xCoords.append(-1)
                yCoords.append(-1)
            table.putNumberArray('x', xCoords)
            table.putNumberArray('y', yCoords)
            print(xCoords, yCoords)

            if cv2.waitKey(1) == 27:
                    break

    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(peg_filter_real.PegFilter())
