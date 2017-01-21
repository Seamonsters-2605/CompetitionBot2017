import cv2
import numpy
import math
import time
from enum import Enum

from boiler_filter import BoilerFilter

def show_webcam(start_time):
	time_step = 0
	boiler_filter = BoilerFilter()
	cam = cv2.VideoCapture(0)
	while True:
		print("Time elapsed: {} Time step: {}".format(time.time() - start_time, time.time() - time_step))
		time_step = time.time()
		ret_val, img = cam.read()
		boiler_filter.process(img)
		cv2.imshow('my webcam', img)
		cv2.imshow('image 2', boiler_filter.cv_erode_output)
		print(boiler_filter.convex_hulls_output)
		if cv2.waitKey(1) == 27:
			break
	cv2.destroyAllWindows()

def main():
	
	show_webcam(time.time())

if __name__ == '__main__':
	main()
