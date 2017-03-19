try:
    import cscore

    camera = cscore.UsbCamera("usbcam", 0)
    camera.setVideoMode(cscore.VideoMode.PixelFormat.kMJPEG, 320, 240, 30)
    mjpegServer = cscore.MjpegServer("httpserver", 1187)
    mjpegServer.setSource(camera)

    otherCamera = cscore.UsbCamera("usbcam-boiler", 1)
    otherCamera.setVideoMode(cscore.VideoMode.PixelFormat.kMJPEG, 320, 240, 30)
    otherMjpegServer = cscore.MjpegServer("httpserver-boiler", 1188)
    otherMjpegServer.setSource(otherCamera)
except BaseException as e:
    print("cscore error")
    print(e)
