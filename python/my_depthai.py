#!/usr/bin/env python3

from pathlib import Path
import sys
import cv2
import depthai as dai
import numpy as np
import time
from threading import Lock
from copy import deepcopy
import math
from playsound import playsound

MAX_ATTEMPTS = 3
TOP_MOUNTED_OAK_D_ID = "14442C103147C2D200"
BOTTOM_MOUNTED_OAK_D_ID = "14442C10E18CC0D200";

'''
Spatial Tiny-yolo example
  Performs inference on RGB camera and retrieves spatial location coordinates: x,y,z relative to the center of depth map.
  Can be used for tiny-yolo-v3 or tiny-yolo-v4 networks
'''
class MyDetection(object):
    def __init__(self, label, use_tracker, det):
        if use_tracker:
            self.confidence = det.srcImgDetection.confidence
            self.xmin = det.srcImgDetection.xmin
            self.ymin = det.srcImgDetection.ymin
            self.xmax = det.srcImgDetection.xmax
            self.ymax = det.srcImgDetection.ymax
            self.id = det.id
            self.status = det.status
        else: # not use_tracker
            self.confidence = det.confidence
            self.xmin = det.xmin
            self.ymin = det.ymin
            self.xmax = det.xmax
            self.ymax = det.ymax
            self.id = 0
            self.status = ""
            
        self.label = label
        self.x = det.spatialCoordinates.x / 1000.0
        self.y = det.spatialCoordinates.y / 1000.0
        self.z = det.spatialCoordinates.z / 1000.0

        self.theta = math.degrees(-math.asin(self.x/self.z) if self.z != 0.0 else 0)
        self.bboxCtr = [ (self.xmin + self.xmax) / 2.0, (self.ymin + self.ymax) / 2.0]
        
class MyDepthAI:
    def __init__(
        self,
        model = "tinyYolo",
        use_tracker = False,
        syncNN = True,
    ):
        self.model = model
        self.use_tracker = use_tracker
        self.syncNN = syncNN
        self.detection_lock = Lock()

        self.detection_lock.acquire()
        self.personDetections = []
        self.objectDetections = []
        self.detection_lock.release()
        self.run_flag = False
        self.inner_run_flag = False
        self.outer_run_flag = False
        self.pipeline = None
        self.nnBlobPath =""
        self.labelMap = []
        self.takePictureNow = False
        self._showRgbWindow = False
        self._showDepthWindow = False
        self._loc = "TOP"
        self._get_picture_cb = None

        if self.model == "mobileNet":
            # Mobilenet ssd labels
            self.labelMap = ["background", "aeroplane", "bicycle", "bird", "boat", "bottle", "bus", "car", "cat", "chair", "cow",
                        "diningtable", "dog", "horse", "motorbike", "person", "pottedplant", "sheep", "sofa", "train", "tvmonitor"]
            self.nnBlobPath = str((Path(__file__).parent / Path('models/mobilenet-ssd_openvino_2021.4_5shave.blob')).resolve().absolute())
        elif self.model == "tinyYolo":
            # Tiny yolo v3/4 label texts
            self.labelMap = [
                "person",         "bicycle",    "car",           "motorbike",     "aeroplane",   "bus",           "train",
                "truck",          "boat",       "traffic light", "fire hydrant",  "stop sign",   "parking meter", "bench",
                "bird",           "cat",        "dog",           "horse",         "sheep",       "cow",           "elephant",
                "bear",           "zebra",      "giraffe",       "backpack",      "umbrella",    "handbag",       "tie",
                "suitcase",       "frisbee",    "skis",          "snowboard",     "sports ball", "kite",          "baseball bat",
                "baseball glove", "skateboard", "surfboard",     "tennis racket", "bottle",      "wine glass",    "cup",
                "fork",           "knife",      "spoon",         "bowl",          "banana",      "apple",         "sandwich",
                "orange",         "broccoli",   "carrot",        "hot dog",       "pizza",       "donut",         "cake",
                "chair",          "sofa",       "pottedplant",   "bed",           "diningtable", "toilet",        "tvmonitor",
                "laptop",         "mouse",      "remote",        "keyboard",      "cell phone",  "microwave",     "oven",
                "toaster",        "sink",       "refrigerator",  "book",          "clock",       "vase",          "scissors",
                "teddy bear",     "hair drier", "toothbrush"
            ]
            self.nnBlobPath = str((Path(__file__).parent / Path('models/tiny-yolo-v4_openvino_2021.2_6shave.blob')).resolve().absolute())
            #self.nnBlobPath = str((Path(__file__).parent / Path('models/yolo-v4-tiny-tf_openvino_2021.4_6shave.blob')).resolve().absolute())

        if not Path(self.nnBlobPath).exists():
            import sys
            raise FileNotFoundError(f'Required file/s not found, please run "{sys.executable} install_requirements.py"')

    def createPipeline(self):
        # Start defining a pipeline
        self.pipeline = dai.Pipeline()

        # Define a source - color camera
        colorCam = self.pipeline.createColorCamera()
        colorCam.initialControl.setManualFocus(130)

        if self.model == "mobileNet":
            spatialDetectionNetwork = self.pipeline.create(dai.node.MobileNetSpatialDetectionNetwork)
        elif self.model == "tinyYolo":
            spatialDetectionNetwork = self.pipeline.createYoloSpatialDetectionNetwork()

        monoLeft = self.pipeline.createMonoCamera()
        monoRight = self.pipeline.createMonoCamera()
        stereo = self.pipeline.createStereoDepth()

        xoutRgb = self.pipeline.createXLinkOut()
        xoutNN = self.pipeline.createXLinkOut()
        #xoutBoundingBoxDepthMapping = self.pipeline.createXLinkOut()
        xoutDepth = self.pipeline.createXLinkOut()

        xoutRgb.setStreamName("rgb")
        xoutNN.setStreamName("detections")
        #xoutBoundingBoxDepthMapping.setStreamName("boundingBoxDepthMapping")
        xoutDepth.setStreamName("depth")

        colorCam.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
        colorCam.setInterleaved(False)
        colorCam.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)

        monoLeft.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
        monoLeft.setBoardSocket(dai.CameraBoardSocket.LEFT)
        monoRight.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
        monoRight.setBoardSocket(dai.CameraBoardSocket.RIGHT)

        # setting node configs
        stereo.setConfidenceThreshold(255)
        # Align depth map to the perspective of RGB camera, on which inference is done
        stereo.setDepthAlign(dai.CameraBoardSocket.CAM_A)

        #stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY)

        spatialDetectionNetwork.setBlobPath(self.nnBlobPath)
        spatialDetectionNetwork.setConfidenceThreshold(0.7)
        spatialDetectionNetwork.input.setBlocking(False)
        spatialDetectionNetwork.setBoundingBoxScaleFactor(0.5)
        spatialDetectionNetwork.setDepthLowerThreshold(100)
        spatialDetectionNetwork.setDepthUpperThreshold(5000)

        if self.model == "tinyYolo":
            # Yolo specific parameters
            colorCam.setPreviewSize(416, 416)
            spatialDetectionNetwork.setNumClasses(80)
            spatialDetectionNetwork.setCoordinateSize(4)
            spatialDetectionNetwork.setAnchors(np.array([10,14, 23,27, 37,58, 81,82, 135,169, 344,319]))
            spatialDetectionNetwork.setAnchorMasks({ "side26": np.array([1,2,3]), "side13": np.array([3,4,5]) })
            spatialDetectionNetwork.setIouThreshold(0.7)
        elif self.model == "mobileNet":
            colorCam.setPreviewSize(300, 300)

        # Create outputs

        monoLeft.out.link(stereo.left)
        monoRight.out.link(stereo.right)

        colorCam.preview.link(spatialDetectionNetwork.input)

        #spatialDetectionNetwork.boundingBoxMapping.link(xoutBoundingBoxDepthMapping.input)

        stereo.depth.link(spatialDetectionNetwork.inputDepth)
        spatialDetectionNetwork.passthroughDepth.link(xoutDepth.input)

        if self.use_tracker:
            # Create object tracker
            objectTracker = self.pipeline.createObjectTracker()
            # track only person
            if self.model == "tinyYolo":
                objectTracker.setDetectionLabelsToTrack([0])
            elif self.model == "mobileNet":
                objectTracker.setDetectionLabelsToTrack([15])                
            # possible tracking types: ZERO_TERM_COLOR_HISTOGRAM, ZERO_TERM_IMAGELESS, SHORT_TERM_IMAGELESS, SHORT_TERM_KCF
            objectTracker.setTrackerType(dai.TrackerType.ZERO_TERM_COLOR_HISTOGRAM)
            # take the smallest ID when new object is tracked, possible options: SMALLEST_ID, UNIQUE_ID
            objectTracker.setTrackerIdAssignmentPolicy(dai.TrackerIdAssignmentPolicy.UNIQUE_ID)
            # rgb
            if self.syncNN:
                objectTracker.passthroughTrackerFrame.link(xoutRgb.input)
            else:
                colorCam.preview.link(xoutRgb.input)
            # Its input
            spatialDetectionNetwork.passthrough.link(objectTracker.inputTrackerFrame)
            spatialDetectionNetwork.passthrough.link(objectTracker.inputDetectionFrame)
            spatialDetectionNetwork.out.link(objectTracker.inputDetections)
            objectTracker.out.link(xoutNN.input)
        else:
            if self.syncNN:
                spatialDetectionNetwork.passthrough.link(xoutRgb.input)
            else:
                colorCam.preview.link(xoutRgb.input)
            spatialDetectionNetwork.out.link(xoutNN.input)
        
    def shutdown(self):
        self.outer_run_flag = False
        self.run_flag = False
        self.inner_run_flag = False

    def getPersonDetections(self):
        with self.detection_lock:
            return deepcopy(self.personDetections)

    def getObjectDetections(self):
        with self.detection_lock:
            return deepcopy(self.objectDetections)
        
    def takePicture(self):
        self.takePictureNow = True

    def setGetPictureCb(self, get_picture_cb):
         self._get_picture_cb = get_picture_cb

    def rgbWindowVisible(self):
        return self._showRgbWindow
    
    def depthWindowVisible(self):
        return self._showDepthWindow

    def changeCamera(self, loc):
        if loc != self._loc:
            print("changing camera to {}".format(loc))
            self._loc = loc
            self.run_flag = False
            self.inner_run_flag = False

    def waitUntilChangeFinished(self):
        while not self.inner_run_flag:
            time.sleep(0.1)

    def showRgbWindow(self, value):
        if value != self._showRgbWindow:
            self._showRgbWindow = value
            self.inner_run_flag = False            

    def showDepthWindow(self, value):
        if value != self._showDepthWindow:
            self._showDepthWindow = value
            self.inner_run_flag = False            
        
    def startUp(self, loc="TOP", showRgbWindow=False, showDepthWindow=False):
        # Connect and start the pipeline
        
        self.showRgbWindow(showRgbWindow)
        self.showDepthWindow(showDepthWindow)

        self.createPipeline()
        self._loc = loc

        self.outer_run_flag = True
        while self.outer_run_flag:
            if self._loc == "TOP":
                device_id = TOP_MOUNTED_OAK_D_ID
            else:
                device_id = BOTTOM_MOUNTED_OAK_D_ID

            try_count = 3
            while try_count > 0:
                found, device_info = dai.Device.getDeviceByMxId(device_id)

                if found:
                    break
                else:
                    if try_count > 1:
                        time.sleep(2)
                        try_count -= 1
                    else:
                        raise RuntimeError("Oak-D device not found!")

            rgb_win_name = "rgb"+loc
            depth_win_name = "depth"+loc

            self.run_flag = True
            while self.run_flag:
                try:
                    if self._showRgbWindow:
                        cv2.namedWindow(rgb_win_name, cv2.WINDOW_NORMAL)
                        cv2.resizeWindow(rgb_win_name, 832, 832)
                    with dai.Device(self.pipeline, device_info) as device:
                    
                        # Output queues will be used to get the rgb frames and nn data from the outputs ffined above
                        previewQueue = device.getOutputQueue(name="rgb", maxSize=4, blocking=False)
                        detectionNNQueue = device.getOutputQueue(name="detections", maxSize=4, blocking=False)
                        #xoutBoundingBoxDepthMapping = device.getOutputQueue(name="boundingBoxDepthMapping", maxSize=4, blocking=False)
                        depthQueue = device.getOutputQueue(name="depth", maxSize=4, blocking=False)
                    
                        frame = None
                        detections = []
                    
                        startTime = time.monotonic()
                        counter = 0
                        fps = 0
                        color = (255, 255, 255)
                    
                        self.inner_run_flag = True
                        while self.inner_run_flag:
                            inPreview = previewQueue.get()                  
                            inNN = detectionNNQueue.get()
                            depth = depthQueue.get()
                    
                            counter+=1
                            current_time = time.monotonic()
                            if (current_time - startTime) > 1 :
                                fps = counter / (current_time - startTime)
                                counter = 0
                                startTime = current_time
                                        
                            detections = inNN.tracklets if self.use_tracker else inNN.detections
                            
                            if self._showDepthWindow:
                                depthFrame = depth.getFrame()
                                depthFrameColor = cv2.normalize(depthFrame, None, 255, 0, cv2.NORM_INF, cv2.CV_8UC1)
                                depthFrameColor = cv2.equalizeHist(depthFrameColor)
                                depthFrameColor = cv2.applyColorMap(depthFrameColor, cv2.COLORMAP_HOT)
                                #if len(detections) != 0:
                                    #boundingBoxMapping = xoutBoundingBoxDepthMapping.get()
                                    #roiDatas = boundingBoxMapping.getConfigData()            
                                    # for roiData in roiDatas:
                                    #     roi = roiData.roi
                                    #     roi = roi.denormalize(depthFrameColor.shape[1], depthFrameColor.shape[0])
                                    #     topLeft = roi.topLeft()
                                    #     bottomRight = roi.bottomRight()
                                    #     xmin = int(topLeft.x)
                                    #     ymin = int(topLeft.y)
                                    #     xmax = int(bottomRight.x)
                                    #     ymax = int(bottomRight.y)
                                    #     cv2.rectangle(depthFrameColor, (xmin, ymin), (xmax, ymax), color, cv2.FONT_HERSHEY_SCRIPT_SIMPLEX)

                    
                            # If the frame is available, draw bounding boxes on it and show the frame
                            if self._showRgbWindow:
                                frame = inPreview.getCvFrame()
                                if self.takePictureNow:
                                    self.takePictureNow = False
                                    pic_filename = "capture_" + time.ctime().replace(' ', '-', -1).replace(":","-",-1) +".jpg"
                                    playsound("sounds\/camera-shutter.wav", block=True)
                                    cv2.imwrite("pictures_taken/" + pic_filename, frame)
                                    cv2.imshow("Snapshot", frame)
                                if self._get_picture_cb is not None:
                                    self._get_picture_cb(frame)
                                    self._get_picture_cb = None

                                height = frame.shape[0]
                                width  = frame.shape[1]
                                
                            with self.detection_lock:
                                self.personDetections = []
                                self.objectDetections = []

                                for detection in detections:                
                                    try:
                                        label = self.labelMap[detection.label]
                                    except:
                                        label = detection.label
                                        
                                    str_label = str(label)
                                    if str_label == "person":
                                        self.personDetections.append(MyDetection(str_label, self.use_tracker, detection))
                                        #if self._showRgbWindow:
                                            #theta = -math.degrees(math.asin(detection.spatialCoordinates.x/detection.spatialCoordinates.z))
                                            #cv2.putText(frame, "theta = {:.2f}".format(theta), (2, frame.shape[0] - 15), cv2.FONT_HERSHEY_TRIPLEX, 0.4, color)
                
                                    else:
                                        self.objectDetections.append(MyDetection(str_label, self.use_tracker, detection))
                                    
                                    if self._showRgbWindow:
                                        # Denormalize bounding box
                                        if self.use_tracker:
                                            x1 = int(detection.srcImgDetection.xmin * width)
                                            x2 = int(detection.srcImgDetection.xmax * width)
                                            y1 = int(detection.srcImgDetection.ymin * height)
                                            y2 = int(detection.srcImgDetection.ymax * height)
                                            cv2.putText(frame, "{:.2f}".format(detection.srcImgDetection.confidence), (x1 + 10, y1 + 35), cv2.FONT_HERSHEY_TRIPLEX, 0.5, color)
                                            cv2.putText(frame, f"ID: {[detection.id]}", (x1 + 10, y1 + 95), cv2.FONT_HERSHEY_TRIPLEX, 0.5, 255)
                                            cv2.putText(frame, detection.status.name, (x1 + 10, y1 + 110), cv2.FONT_HERSHEY_TRIPLEX, 0.5, 255)
                                        else:
                                            x1 = int(detection.xmin * width)
                                            x2 = int(detection.xmax * width)
                                            y1 = int(detection.ymin * height)
                                            y2 = int(detection.ymax * height)
                                            cv2.putText(frame, "{:.2f}".format(detection.confidence), (x1 + 10, y1 + 35), cv2.FONT_HERSHEY_TRIPLEX, 0.5, color)

                                        cv2.putText(frame, str(label), (x1 + 10, y1 + 20), cv2.FONT_HERSHEY_TRIPLEX, 0.5, color)
                                        cv2.putText(frame, f"X: {int(detection.spatialCoordinates.x)} mm", (x1 + 10, y1 + 50), cv2.FONT_HERSHEY_TRIPLEX, 0.5, color)
                                        cv2.putText(frame, f"Y: {int(detection.spatialCoordinates.y)} mm", (x1 + 10, y1 + 65), cv2.FONT_HERSHEY_TRIPLEX, 0.5, color)
                                        cv2.putText(frame, f"Z: {int(detection.spatialCoordinates.z)} mm", (x1 + 10, y1 + 80), cv2.FONT_HERSHEY_TRIPLEX, 0.5, color)
                                        cv2.rectangle(frame, (x1, y1), (x2, y2), color, cv2.FONT_HERSHEY_SIMPLEX)
                            
                            if self._showRgbWindow:           

                                cv2.putText(frame, "NN fps: {:.2f}".format(fps), (2, frame.shape[0] - 4), cv2.FONT_HERSHEY_TRIPLEX, 0.4, color)
                                cv2.imshow(rgb_win_name, frame)
                            
                            if self._showDepthWindow:
                                cv2.imshow(depth_win_name, depthFrameColor)
                            
                            cv2.waitKey(45)
                        
                        cv2.destroyAllWindows()
                except Exception as e:
                    print(repr(e))
                    cv2.destroyAllWindows()
                    time.sleep(1)

if __name__ == '__main__':
    import keyboard
    from my_depthai import MyDepthAI
    from threading import Thread
    mdai = MyDepthAI(model="tinyYolo", use_tracker=False)
    _cameras = ["TOP", "BOTTOM"]
    _cameraIndex = 0
    loc = _cameras[_cameraIndex]

    my_depthai_thread = Thread(target = mdai.startUp, args=(loc, True, False), name="mdai", daemon=False)
    my_depthai_thread.start()

    def toggleRgbWindow(a):
        mdai.showRgbWindow(not mdai.rgbWindowVisible())

    def toggleDepthWindow(a):
        mdai.showDepthWindow(not mdai.depthWindowVisible())

    def toggleCamera(a):
        global _cameraIndex
        _cameraIndex = not _cameraIndex
        mdai.changeCamera(_cameras[_cameraIndex])

    def shutdown(a):
        mdai.shutdown()
        my_depthai_thread.join()
        exit()
    
    keyboard.on_press_key('r', toggleRgbWindow)
    keyboard.on_press_key('d', toggleDepthWindow)
    keyboard.on_press_key('t', toggleCamera)
    keyboard.on_press_key('q', shutdown)
    
    try:
        while True:
            time.sleep(0.1)
    except KeyboardInterrupt:
        mdai.shutdown()
        my_depthai_thread.join()