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

MAX_ATTEMPTS = 3
TOP_MOUNTED_OAK_D_ID = "14442C103147C2D200"

'''
Spatial Tiny-yolo example
  Performs inference on RGB camera and retrieves spatial location coordinates: x,y,z relative to the center of depth map.
  Can be used for tiny-yolo-v3 or tiny-yolo-v4 networks
'''

# Tiny yolo v3/4 label texts
labelMap = [
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

use_tracker = True
syncNN = True

# Get argument first
nnBlobPath = str((Path(__file__).parent / Path('models/yolo-v4-tiny-tf_openvino_2021.4_6shave.blob')).resolve().absolute())
if len(sys.argv) > 1:
    nnBlobPath = sys.argv[1]

if not Path(nnBlobPath).exists():
    import sys
    raise FileNotFoundError(f'Required file/s not found, please run "{sys.executable} install_requirements.py"')

# Start defining a pipeline
pipeline = dai.Pipeline()

# Define a source - color camera
colorCam = pipeline.createColorCamera()
colorCam.initialControl.setManualFocus(130)
spatialDetectionNetwork = pipeline.createYoloSpatialDetectionNetwork()
monoLeft = pipeline.createMonoCamera()
monoRight = pipeline.createMonoCamera()
stereo = pipeline.createStereoDepth()

xoutRgb = pipeline.createXLinkOut()
xoutNN = pipeline.createXLinkOut()
#xoutBoundingBoxDepthMapping = pipeline.createXLinkOut()
xoutDepth = pipeline.createXLinkOut()

xoutRgb.setStreamName("rgb")
xoutNN.setStreamName("detections")
#xoutBoundingBoxDepthMapping.setStreamName("boundingBoxDepthMapping")
xoutDepth.setStreamName("depth")

colorCam.setPreviewSize(416, 416)
colorCam.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
colorCam.setInterleaved(False)
colorCam.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)

monoLeft.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
monoLeft.setBoardSocket(dai.CameraBoardSocket.LEFT)
monoRight.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
monoRight.setBoardSocket(dai.CameraBoardSocket.RIGHT)

# setting node configs
stereo.setConfidenceThreshold(255)

#stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY)

spatialDetectionNetwork.setBlobPath(nnBlobPath)
spatialDetectionNetwork.setConfidenceThreshold(0.5)
spatialDetectionNetwork.input.setBlocking(False)
spatialDetectionNetwork.setBoundingBoxScaleFactor(0.5)
spatialDetectionNetwork.setDepthLowerThreshold(100)
spatialDetectionNetwork.setDepthUpperThreshold(5000)
# Yolo specific parameters
spatialDetectionNetwork.setNumClasses(80)
spatialDetectionNetwork.setCoordinateSize(4)
spatialDetectionNetwork.setAnchors(np.array([10,14, 23,27, 37,58, 81,82, 135,169, 344,319]))
spatialDetectionNetwork.setAnchorMasks({ "side26": np.array([1,2,3]), "side13": np.array([3,4,5]) })
spatialDetectionNetwork.setIouThreshold(0.5)

# Create outputs

monoLeft.out.link(stereo.left)
monoRight.out.link(stereo.right)

colorCam.preview.link(spatialDetectionNetwork.input)

#spatialDetectionNetwork.boundingBoxMapping.link(xoutBoundingBoxDepthMapping.input)

stereo.depth.link(spatialDetectionNetwork.inputDepth)
spatialDetectionNetwork.passthroughDepth.link(xoutDepth.input)

if use_tracker:
    # Create object tracker
    objectTracker = pipeline.createObjectTracker()
    # track only person
    objectTracker.setDetectionLabelsToTrack([0])
    # possible tracking types: ZERO_TERM_COLOR_HISTOGRAM, ZERO_TERM_IMAGELESS, SHORT_TERM_IMAGELESS, SHORT_TERM_KCF
    objectTracker.setTrackerType(dai.TrackerType.ZERO_TERM_COLOR_HISTOGRAM)
    # take the smallest ID when new object is tracked, possible options: SMALLEST_ID, UNIQUE_ID
    objectTracker.setTrackerIdAssignmentPolicy(dai.TrackerIdAssignmentPolicy.UNIQUE_ID)
    # rgb
    if syncNN:
        objectTracker.passthroughTrackerFrame.link(xoutRgb.input)
    else:
        colorCam.preview.link(xoutRgb.input)
    # Its input
    spatialDetectionNetwork.passthrough.link(objectTracker.inputTrackerFrame)
    spatialDetectionNetwork.passthrough.link(objectTracker.inputDetectionFrame)
    spatialDetectionNetwork.out.link(objectTracker.inputDetections)
    objectTracker.out.link(xoutNN.input)
else:
    if syncNN:
        spatialDetectionNetwork.passthrough.link(xoutRgb.input)
    else:
        colorCam.preview.link(xoutRgb.input)
    spatialDetectionNetwork.out.link(xoutNN.input)
    
detection_lock = Lock()

detection_lock.acquire()
_personDetections = []
_objectDetections = []
detection_lock.release()

class MyDetection(object):
    def __init__(self, label, x, y, z, bboxCtr, confidence, id, status):
        if label == "sports ball":
            self.label = "baseball"
        else:
            self.label = label
        self.x = x / 1000.0
        self.y = y / 1000.0
        self.z = z / 1000.0
        self.theta = math.degrees(-math.asin(x/z) if z != 0.0 else 0)
        self.bboxCtr = bboxCtr
        self.confidence = confidence
        self.id = id
        self.status = status
    
def shutdown():
    global _run_flag
    _run_flag = False

def getPersonDetections():
    with detection_lock:
        return deepcopy(_personDetections)

def getObjectDetections():
    with detection_lock:
        return deepcopy(_objectDetections)
    
def startUp(showRgbWindow=False, showDepthWindow=False):
    global _run_flag, _personDetections, _objectDetections
    # Connect and start the pipeline
    _run_flag = True
    
    found, device_info = dai.Device.getDeviceByMxId(TOP_MOUNTED_OAK_D_ID)

    if not found:
        raise RuntimeError("Top mounted Oak-D device not found!")

    while _run_flag:
        try:
            if showRgbWindow:
                cv2.namedWindow('rgb', cv2.WINDOW_NORMAL)
                cv2.resizeWindow('rgb', 832, 832)
            with dai.Device(pipeline, device_info) as device:
            
                # Output queues will be used to get the rgb frames and nn data from the outputs defined above
                if showRgbWindow:
                    previewQueue = device.getOutputQueue(name="rgb", maxSize=4, blocking=False)
                detectionNNQueue = device.getOutputQueue(name="detections", maxSize=4, blocking=False)
                #xoutBoundingBoxDepthMapping = device.getOutputQueue(name="boundingBoxDepthMapping", maxSize=4, blocking=False)
                if showDepthWindow:
                    depthQueue = device.getOutputQueue(name="depth", maxSize=4, blocking=False)
            
                frame = None
                tracklets = []
            
                startTime = time.monotonic()
                counter = 0
                fps = 0
                color = (255, 255, 255)
            
                while _run_flag:
                    if showRgbWindow:
                        inPreview = previewQueue.get()
                    inNN = detectionNNQueue.get()
                    if showDepthWindow:
                        depth = depthQueue.get()
            
                    counter+=1
                    current_time = time.monotonic()
                    if (current_time - startTime) > 1 :
                        fps = counter / (current_time - startTime)
                        counter = 0
                        startTime = current_time
                                
                    tracklets = inNN.tracklets
                    
                    if showDepthWindow:
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
                    if showRgbWindow:
                        frame = inPreview.getCvFrame()                    
                        height = frame.shape[0]
                        width  = frame.shape[1]
                        
                    with detection_lock:
                        _personDetections = []
                        _objectDetections = []
                        for tracklet in tracklets:                
                            try:
                                label = labelMap[tracklet.label]
                            except:
                                label = tracklet.label
                                
                            str_label = str(label)
                            if str_label == "person":
                                _personDetections.append(MyDetection(str_label,
                                                                     tracklet.spatialCoordinates.x, 
                                                                     tracklet.spatialCoordinates.y,
                                                                     tracklet.spatialCoordinates.z,
                                                                     [(tracklet.srcImgDetection.xmax + tracklet.srcImgDetection.xmin) / 2.0,
                                                                      (tracklet.srcImgDetection.ymax + tracklet.srcImgDetection.ymin) / 2.0],
                                                                     tracklet.srcImgDetection.confidence,
                                                                     tracklet.id,
                                                                     tracklet.status))
                                #if showRgbWindow:
                                    #theta = -math.degrees(math.asin(tracklet.spatialCoordinates.x/tracklet.spatialCoordinates.z))
                                    #cv2.putText(frame, "theta = {:.2f}".format(theta), (2, frame.shape[0] - 15), cv2.FONT_HERSHEY_TRIPLEX, 0.4, color)
        
                            else:
                                _objectDetections.append(MyDetection(str_label,
                                                                     tracklet.spatialCoordinates.x, 
                                                                     tracklet.spatialCoordinates.y,
                                                                     tracklet.spatialCoordinates.z,
                                                                     [(tracklet.srcImgDetection.xmax + tracklet.srcImgDetection.xmin) / 2.0,
                                                                      (tracklet.srcImgDetection.ymax + tracklet.srcImgDetection.ymin) / 2.0],
                                                                     tracklet.srcImgDetection.confidence,
                                                                     tracklet.id,
                                                                     tracklet.status))
                            if label == "sports ball":
                                l = "baseball"
                            else:
                                l = label
                            
                            if showRgbWindow:
                                # Denormalize bounding box
                                x1 = int(tracklet.srcImgDetection.xmin * width)
                                x2 = int(tracklet.srcImgDetection.xmax * width)
                                y1 = int(tracklet.srcImgDetection.ymin * height)
                                y2 = int(tracklet.srcImgDetection.ymax * height)
                                cv2.putText(frame, str(label), (x1 + 10, y1 + 20), cv2.FONT_HERSHEY_TRIPLEX, 0.5, color)
                                cv2.putText(frame, "{:.2f}".format(tracklet.srcImgDetection.confidence*100), (x1 + 10, y1 + 35), cv2.FONT_HERSHEY_TRIPLEX, 0.5, color)
                                cv2.putText(frame, f"X: {int(tracklet.spatialCoordinates.x)} mm", (x1 + 10, y1 + 50), cv2.FONT_HERSHEY_TRIPLEX, 0.5, color)
                                cv2.putText(frame, f"Y: {int(tracklet.spatialCoordinates.y)} mm", (x1 + 10, y1 + 65), cv2.FONT_HERSHEY_TRIPLEX, 0.5, color)
                                cv2.putText(frame, f"Z: {int(tracklet.spatialCoordinates.z)} mm", (x1 + 10, y1 + 80), cv2.FONT_HERSHEY_TRIPLEX, 0.5, color)
                                cv2.putText(frame, f"ID: {[tracklet.id]}", (x1 + 10, y1 + 95), cv2.FONT_HERSHEY_TRIPLEX, 0.5, 255)
                                cv2.putText(frame, tracklet.status.name, (x1 + 10, y1 + 110), cv2.FONT_HERSHEY_TRIPLEX, 0.5, 255)                            
                                cv2.rectangle(frame, (x1, y1), (x2, y2), color, cv2.FONT_HERSHEY_SIMPLEX)
                    
                    if showRgbWindow:                    
                        cv2.putText(frame, "NN fps: {:.2f}".format(fps), (2, frame.shape[0] - 4), cv2.FONT_HERSHEY_TRIPLEX, 0.4, color)
                        cv2.imshow("rgb", frame)
                    if showDepthWindow:
                        cv2.imshow("depth", depthFrameColor)
                    cv2.waitKey(50)
                cv2.destroyAllWindows()
        except Exception as e:
            print(repr(e))
            time.sleep(1)
            