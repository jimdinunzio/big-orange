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

# Fraction of a detection box kept when asking for its depth.  The ROI is
# shrunk about its centre so the patch averaged is the object rather than
# the floor around it -- the same 0.5 the spatial detection network used by
# Oak-D on camera algs applies to its own boxes (setBoundingBoxScaleFactor below).
#
# It assumes the middle of the box IS the object, which holds for a can, a
# person's torso, a lamp -- and not for something you can see through, like a
# chair or a doorway, where the middle is the wall behind.  A caller with a
# hollow object should pass a shrink nearer 1.0 and let the median sort it.
ROI_SHRINK = 0.5

# Smallest patch worth asking about, in depth pixels per axis.  A distant
# object's box can be only a few pixels across before any shrink.
ROI_MIN_PIXELS = 6

# The depth window opens at this fraction of the depth frame's own size.
DEPTH_WINDOW_SCALE = 0.5

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

        self.bboxCtr = [ (self.xmin + self.xmax) / 2.0, (self.ymin + self.ymax) / 2.0]
        
class MyDepthAI:
    def __init__(
        self,
        model = "yolo8nano",
        use_tracker = False,
        syncNN = True,
        subpixel = True,
    ):
        self.model = model
        self.use_tracker = use_tracker
        self.syncNN = syncNN
        # Fractional disparity, which is what resolves depth finer than one
        # whole disparity step -- about 17 mm at 0.75 m, growing as the square
        # of range.  Fixed when the pipeline is built: the firmware refuses to
        # switch it on a running device, and refuses a config message at all
        # while depth is aligned to a camera.
        self.subpixel = subpixel
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
        self._show_yolo_boxes = True
        self._loc = "TOP"
        self._get_picture_cb = None
        self._closePictures = False
        self.window_size = [832, 832]
        
        # Text overlay state: {line: (text, size, expire_time)}
        self._text_overlay = {}
        self._text_overlay_lock = Lock()

        # ROI overlay: list of ((x1,y1,x2,y2), text) entries
        self._roi_rects = []
        self._roi_last_draw_time = 0

        # The same ROIs as handed to the spatial calculator, normalized on the
        # depth frame, so the depth window shows the pixels actually averaged
        self._depth_roi_rects = []
        self._depth_size = None

        # Preview-to-depth geometry, measured off the camera in createPipeline
        self._isp_size = None
        self._preview_crop = (1.0, 1.0)

        self._depth_win_sized = False

        # Latest frame storage for external consumers (e.g. NanoOwlManager)
        self._frame_lock = Lock()
        self._latest_preview = None   # numpy BGR
        self._frame_seq = 0

        if self.model == "mobileNet":
            # Mobilenet ssd labels
            self.labelMap = ["background", "aeroplane", "bicycle", "bird", "boat", "bottle", "bus", "car", "cat", "chair", "cow",
                        "diningtable", "dog", "horse", "motorbike", "person", "pottedplant", "sheep", "sofa", "train", "tvmonitor"]
            self.nnBlobPath = str((Path(__file__).parent / Path('models/mobilenet-ssd_openvino_2021.4_5shave.blob')).resolve().absolute())
        elif self.model == "tinyYolo" or self.model == "yolo8nano":
            # Tiny yolo v3/4 and yolo8 nano label texts
            self.labelMap = [
                "person",         "bicycle",    "car",           "motorbike",     "aeroplane",   "bus",           "train", # 6
                "truck",          "boat",       "traffic light", "fire hydrant",  "stop sign",   "parking meter", "bench", # 13
                "bird",           "cat",        "dog",           "horse",         "sheep",       "cow",           "elephant", # 20
                "bear",           "zebra",      "giraffe",       "backpack",      "umbrella",    "handbag",       "tie", # 27
                "suitcase",       "frisbee",    "skis",          "snowboard",     "sports ball", "kite",          "baseball bat", # 34
                "baseball glove", "skateboard", "surfboard",     "tennis racket", "bottle",      "wine glass",    "cup", # 41
                "fork",           "knife",      "spoon",         "bowl",          "banana",      "apple",         "sandwich", # 48
                "orange",         "broccoli",   "carrot",        "hot dog",       "pizza",       "donut",         "cake", # 55
                "chair",          "sofa",       "pottedplant",   "bed",           "diningtable", "toilet",        "tvmonitor", # 62
                "laptop",         "mouse",      "remote",        "keyboard",      "cell phone",  "microwave",     "oven", # 69
                "toaster",        "sink",       "refrigerator",  "book",          "clock",       "vase",          "scissors", # 76
                "teddy bear",     "hair drier", "toothbrush"
            ]
        if self.model == "yolo8nano":
            self.nnBlobPath = str((Path(__file__).parent / Path('models/yolov8n_coco_640x352.blob')).resolve().absolute())
        elif self.model == "tinyYolo":
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
        elif self.model == "tinyYolo" or self.model == "yolo8nano":
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
        if self.subpixel:
            stereo.setSubpixel(True)

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
            self.window_size = [832, 832]
        elif self.model == "yolo8nano":
            # Yolo8 nano specific parameters
            colorCam.setPreviewSize(640, 352)
            spatialDetectionNetwork.setNumClasses(80)
            spatialDetectionNetwork.setCoordinateSize(4)
            spatialDetectionNetwork.setIouThreshold(0.5)
            self.window_size = [1280, 736]
        elif self.model == "mobileNet":
            colorCam.setPreviewSize(300, 300)
            self.window_size = [832, 832]

        # How much of the colour frame the preview actually shows.  With
        # keepAspectRatio -- the default -- a preview shaped differently from
        # the sensor is a centre CROP, so a square preview off a 16:9 sensor
        # spans only about 56% of its width.  Depth is aligned to the whole
        # colour frame, so preview coordinates have to be mapped before they
        # can be used as a depth ROI.  See previewRoiToDepth.
        isp_w, isp_h = colorCam.getIspSize()
        if not isp_w or not isp_h:
            isp_w, isp_h = colorCam.getResolutionSize()
        prev_w, prev_h = colorCam.getPreviewSize()
        self._isp_size = (isp_w, isp_h)
        if colorCam.getPreviewKeepAspectRatio() and prev_w and prev_h:
            isp_ar = isp_w / float(isp_h)
            prev_ar = prev_w / float(prev_h)
            if prev_ar < isp_ar:        # narrower than the sensor: sides cut
                self._preview_crop = (prev_ar / isp_ar, 1.0)
            else:                       # wider: top and bottom cut
                self._preview_crop = (1.0, isp_ar / prev_ar)
        else:
            self._preview_crop = (1.0, 1.0)

        # Create outputs

        monoLeft.out.link(stereo.left)
        monoRight.out.link(stereo.right)

        colorCam.preview.link(spatialDetectionNetwork.input)

        #spatialDetectionNetwork.boundingBoxMapping.link(xoutBoundingBoxDepthMapping.input)

        stereo.depth.link(spatialDetectionNetwork.inputDepth)
        spatialDetectionNetwork.passthroughDepth.link(xoutDepth.input)

        # SpatialLocationCalculator for on-demand ROI depth queries (used by NanoOwlManager)
        spatialCalc = self.pipeline.create(dai.node.SpatialLocationCalculator)
        spatialCalc.setWaitForConfigInput(True)
        spatialCalc.inputDepth.setBlocking(False)
        spatialCalc.inputDepth.setQueueSize(1)

        xinSpatialCalcConfig = self.pipeline.createXLinkIn()
        xinSpatialCalcConfig.setStreamName("spatialCalcConfig")
        xinSpatialCalcConfig.out.link(spatialCalc.inputConfig)

        stereo.depth.link(spatialCalc.inputDepth)

        xoutSpatialCalc = self.pipeline.createXLinkOut()
        xoutSpatialCalc.setStreamName("spatialCalcData")
        spatialCalc.out.link(xoutSpatialCalc.input)

        if self.use_tracker:
            # Create object tracker
            objectTracker = self.pipeline.createObjectTracker()
            # track only person
            if self.model == "tinyYolo" or self.model == "yolo8nano":
                objectTracker.setDetectionLabelsToTrack([0,41])
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
        
    def closePictures(self):
        self._closePictures = True

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
            self._depth_win_sized = False
            self.inner_run_flag = False

    @property
    def show_yolo_boxes(self):
        return self._show_yolo_boxes

    @show_yolo_boxes.setter
    def show_yolo_boxes(self, value):
        self._show_yolo_boxes = value

    def drawText(self, text, line, size):
        """
        Draw text over the RGB window at the top left in dark yello.
        
        Args:
            text: The string to display
            line: Line number 1-5 for vertical position
            size: Point size for the text
        """
        if line < 1 or line > 5:
            return
        expire_time = time.monotonic() + 5.0
        with self._text_overlay_lock:
            self._text_overlay[line] = (text, size, expire_time)
        
    def getLatestFrame(self):
        """Return (preview_bgr, seq) or (None, 0) if not yet available."""
        with self._frame_lock:
            if self._latest_preview is None:
                return None, 0
            return self._latest_preview.copy(), self._frame_seq

    def getPreviewSize(self):
        """Return (width, height) of the preview frame, or None if not yet available."""
        with self._frame_lock:
            if self._latest_preview is not None:
                h, w = self._latest_preview.shape[:2]
                return w, h
        return None

    def previewRoiToDepth(self, xmin, ymin, xmax, ymax):
        """
        Map a preview-normalized rect onto the aligned depth frame.

        The preview is a centre crop of the colour frame and the depth is
        aligned to the whole of it, so 0-1 spans a different slice on each.
        How different depends on the model's preview shape: a hair in y for
        the 640x352 yolo8nano preview, nearly 2x in x for a square one.

        Args:
            xmin, ymin, xmax, ymax: normalized 0-1 in the preview frame.

        Returns:
            (xmin, ymin, xmax, ymax) normalized 0-1 on the depth frame.
        """
        fx, fy = self._preview_crop
        return (0.5 + (xmin - 0.5) * fx, 0.5 + (ymin - 0.5) * fy,
                0.5 + (xmax - 0.5) * fx, 0.5 + (ymax - 0.5) * fy)

    def getSpatialForROI(self, xmin, ymin, xmax, ymax, draw=False, confidence=0.0,
                         shrink=ROI_SHRINK, stats=False):
        """
        Query the VPU SpatialLocationCalculator for depth at a normalized ROI.

        Args:
            xmin, ymin, xmax, ymax: normalized coordinates (0.0 - 1.0) in the
                preview frame.  Mapped onto the depth frame here.
            draw: if True, draw the sampled patch and x,y,z on the RGB preview,
                and the same patch on the depth window.
            confidence: detection confidence 0-1 to show in the overlay (optional).
            shrink: fraction of the box to keep, about its centre.  Sampling
                the whole box averages the floor in front of an object with
                the object, which reads short.
            stats: if True, also return what the depth patch looked like.

        Returns:
            (x, y, z) in meters, or None if unavailable.  With stats,
            (x, y, z, (min_mm, max_mm, pixels)) -- a depth range far wider
            than the object means the patch caught floor or background.
            x = lateral (positive right), y = vertical, z = depth (forward).
        """
        if not hasattr(self, '_spatialCalcConfigQueue') or self._spatialCalcConfigQueue is None:
            return None

        box = (xmin, ymin, xmax, ymax)
        if shrink and shrink < 1.0:
            cx, cy = (xmin + xmax) / 2.0, (ymin + ymax) / 2.0
            hw, hh = (xmax - xmin) * shrink / 2.0, (ymax - ymin) * shrink / 2.0
            xmin, xmax = cx - hw, cx + hw
            ymin, ymax = cy - hh, cy + hh

        dxmin, dymin, dxmax, dymax = self.previewRoiToDepth(xmin, ymin, xmax, ymax)
        bxmin, bymin, bxmax, bymax = self.previewRoiToDepth(*box)

        # A patch of a couple of pixels comes back empty or all noise, so give
        # back some of the shrink on a small detection rather than all of it.
        # Never past the detection itself: outside that box is not the object.
        dw, dh = self._depth_size or (640, 400)
        if dxmax - dxmin < ROI_MIN_PIXELS / dw:
            c, half = (dxmin + dxmax) / 2.0, ROI_MIN_PIXELS / dw / 2.0
            dxmin, dxmax = max(bxmin, c - half), min(bxmax, c + half)
        if dymax - dymin < ROI_MIN_PIXELS / dh:
            c, half = (dymin + dymax) / 2.0, ROI_MIN_PIXELS / dh / 2.0
            dymin, dymax = max(bymin, c - half), min(bymax, c + half)
        dxmin, dymin = max(0.0, dxmin), max(0.0, dymin)
        dxmax, dymax = min(1.0, dxmax), min(1.0, dymax)

        cfg = dai.SpatialLocationCalculatorConfigData()
        cfg.depthThresholds.lowerThreshold = 100
        cfg.depthThresholds.upperThreshold = 10000
        # MEDIAN rather than the default average: pitched down at the floor
        # the patch spans a real range of depths, and one corner of floor or
        # background drags a mean where a median shrugs it off.
        cfg.calculationAlgorithm = dai.SpatialLocationCalculatorAlgorithm.MEDIAN
        cfg.roi = dai.Rect(dai.Point2f(dxmin, dymin), dai.Point2f(dxmax, dymax))

        spatialCfg = dai.SpatialLocationCalculatorConfig()
        spatialCfg.addROI(cfg)
        if stats:
            # MEDIAN leaves depthMin and depthMax unset, so ask for the same
            # patch a second way in the same message: the median answers, the
            # average alongside it says what the patch was made of.  Both come
            # back in one round trip.
            statCfg = dai.SpatialLocationCalculatorConfigData()
            statCfg.depthThresholds.lowerThreshold = 100
            statCfg.depthThresholds.upperThreshold = 10000
            statCfg.calculationAlgorithm = dai.SpatialLocationCalculatorAlgorithm.AVERAGE
            statCfg.roi = cfg.roi
            spatialCfg.addROI(statCfg)
        self._spatialCalcConfigQueue.send(spatialCfg)

        spatialData = self._spatialCalcQueue.get()
        if spatialData is None:
            return None

        locations = spatialData.getSpatialLocations()
        if len(locations) == 0:
            return None

        loc = locations[0]
        coords = loc.spatialCoordinates
        result = (coords.x / 1000.0, coords.y / 1000.0, coords.z / 1000.0)

        if draw:
            size = self.getPreviewSize()
            if size is not None:
                pw, ph = size
                rect = (int(xmin * pw), int(ymin * ph), int(xmax * pw), int(ymax * ph))
                lines = []
                if confidence > 0:
                    lines.append(f"{confidence:.2f}")
                lines += [f"X:{coords.x:.0f}", f"Y:{coords.y:.0f}", f"Z:{coords.z:.0f}mm"]
                self._roi_rects.append((rect, lines))
                self._roi_last_draw_time = time.monotonic()
            self._depth_roi_rects.append((dxmin, dymin, dxmax, dymax))

        if stats:
            s = locations[1] if len(locations) > 1 else loc
            dmin, dmax = s.depthMin, s.depthMax
            if dmax <= dmin:
                # Left at their sentinels, so this algorithm did not fill them
                dmin = dmax = 0
            return result + ((dmin, dmax, s.depthAveragePixelCount),)
        return result

    def depthResolutionAt(self, z_m):
        """
        How coarsely depth is quantized at this range, in metres.

        One whole disparity step, or a fraction of one with subpixel on.
        The error grows as the square of range, so a reading good to a
        centimetre up close is good to nothing like that across a room.
        """
        # 400p mono, 75 mm baseline.  Focal length in pixels from the 71.9
        # degree horizontal field the OAK-D's mono cameras see.
        f_px = 640 / (2 * math.tan(math.radians(71.9 / 2)))
        step = (z_m * z_m) / (f_px * 0.075)
        # Subpixel splits each step into 2^3 with the default fractional bits
        return step / 8.0 if self.subpixel else step

    def drawROIRect(self, xmin, ymin, xmax, ymax, text=""):
        """Draw the ROI rectangle overlay directly from normalized coordinates.

        Use this when you have a detection bbox but no depth data.
        Args:
            xmin, ymin, xmax, ymax: normalized 0-1 coordinates
            text: optional overlay text (e.g. confidence)
        """
        size = self.getPreviewSize()
        if size is None:
            return
        pw, ph = size
        rect = (int(xmin * pw), int(ymin * ph), int(xmax * pw), int(ymax * ph))
        lines = [text] if text else None
        self._roi_rects.append((rect, lines))
        self._roi_last_draw_time = time.monotonic()

    def clear_roi_rects(self):
        """Clear all ROI overlays (call before processing a new detection batch)."""
        self._roi_rects = []
        self._depth_roi_rects = []

    def stopSpatialForROIDraw(self):
        """Clear the ROI overlay drawn by getSpatialForROI(draw=True)."""
        self._roi_rects = []
        self._depth_roi_rects = []

    def safe_startUp(self, *args, **kwargs):
        try:
            self.startUp(*args, **kwargs)
        except Exception as e:
            print("Exception in depthai thread:", e)
            print("depthai thread exiting.")

    def startUp(self, loc="TOP", showRgbWindow=False, showDepthWindow=False):
        # Connect and start the pipeline
        
        self.showRgbWindow(showRgbWindow)
        self.showDepthWindow(showDepthWindow)

        self.createPipeline()
        self._loc = loc

        self.pictures_shown = []
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
                    self._depth_win_sized = False
                    if self._showRgbWindow:
                        cv2.namedWindow(rgb_win_name, cv2.WINDOW_NORMAL)
                        cv2.resizeWindow(rgb_win_name,self.window_size[0], self.window_size[1])
                    with dai.Device(self.pipeline, device_info) as device:
                    
                        # Output queues will be used to get the rgb frames and nn data from the outputs ffined above
                        previewQueue = device.getOutputQueue(name="rgb", maxSize=4, blocking=False)
                        detectionNNQueue = device.getOutputQueue(name="detections", maxSize=4, blocking=False)
                        #xoutBoundingBoxDepthMapping = device.getOutputQueue(name="boundingBoxDepthMapping", maxSize=4, blocking=False)
                        depthQueue = device.getOutputQueue(name="depth", maxSize=4, blocking=False)
                        self._spatialCalcQueue = device.getOutputQueue(name="spatialCalcData", maxSize=4, blocking=False)
                        self._spatialCalcConfigQueue = device.getInputQueue(name="spatialCalcConfig")

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

                            # Depth ROIs are normalized against this, and it is
                            # not the preview's shape -- see previewRoiToDepth
                            self._depth_size = (depth.getWidth(), depth.getHeight())

                            # Store latest frames for external consumers
                            with self._frame_lock:
                                self._latest_preview = inPreview.getCvFrame()
                                self._frame_seq += 1

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

                                # The patches actually averaged.  Drawn here
                                # rather than on the preview because this is
                                # the frame the ROI coordinates belong to, so
                                # a mapping that is off shows up as a box off
                                # the object.
                                if self._depth_roi_rects and (current_time - self._roi_last_draw_time) > 0.35:
                                    self._depth_roi_rects = []
                                dh_px, dw_px = depthFrameColor.shape[:2]
                                for dx1, dy1, dx2, dy2 in self._depth_roi_rects:
                                    cv2.rectangle(depthFrameColor,
                                                  (int(dx1 * dw_px), int(dy1 * dh_px)),
                                                  (int(dx2 * dw_px), int(dy2 * dh_px)),
                                                  (255, 255, 255), 1)
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
                                    win_name = "snapshot_{:03d}".format(len(self.pictures_shown))
                                    cv2.imshow(win_name, frame)
                                    self.pictures_shown.append(win_name)
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
                
                                    else:
                                        self.objectDetections.append(MyDetection(str_label, self.use_tracker, detection))
                                    
                                    if self._showRgbWindow and self._show_yolo_boxes:
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
                                
                                # Draw text overlay
                                current_time = time.monotonic()
                                with self._text_overlay_lock:
                                    expired_lines = []
                                    for line_num, (text, size, expire_time) in self._text_overlay.items():
                                        if current_time < expire_time:
                                            # Calculate font scale from point size (approximate conversion)
                                            font_scale = size / 30.0
                                            thickness = max(1, int(font_scale * 2))
                                            # Calculate y position based on line number (1-5)
                                            y_pos = int(line_num * size * 1.2) + 10
                                            # Draw dark outline first for contrast, then bright green text
                                            cv2.putText(frame, text, (10, y_pos), cv2.FONT_HERSHEY_SIMPLEX, font_scale, (0, 0, 0), thickness + 1, cv2.LINE_AA)
                                            cv2.putText(frame, text, (10, y_pos), cv2.FONT_HERSHEY_SIMPLEX, font_scale, (50, 255, 50), thickness, cv2.LINE_AA)
                                        else:
                                            expired_lines.append(line_num)
                                    # Remove expired text entries
                                    for line_num in expired_lines:
                                        del self._text_overlay[line_num]
                                
                                # Auto-clear ROI overlays after 0.35s of no draw calls
                                if self._roi_rects and (current_time - self._roi_last_draw_time) > 0.35:
                                    self._roi_rects = []

                                # Draw all ROI overlays
                                _font = cv2.FONT_HERSHEY_SIMPLEX
                                _fscale = 0.4
                                _lh = 16  # line height in pixels
                                _fw = width
                                for roi_rect, roi_lines in self._roi_rects:
                                    rx1, ry1, rx2, ry2 = roi_rect
                                    cv2.rectangle(frame, (rx1, ry1), (rx2, ry2), (0, 255, 0), 2)
                                    if roi_lines:
                                        # Measure widest line to decide side
                                        max_w = max(cv2.getTextSize(l, _font, _fscale, 1)[0][0] for l in roi_lines)
                                        right_x = rx2 + 5
                                        left_x  = rx1 - max_w - 5
                                        tx = right_x if (right_x + max_w) < _fw else left_x
                                        ty = (ry1 + ry2) // 2 - (_lh * (len(roi_lines) - 1)) // 2
                                        for line in roi_lines:
                                            cv2.putText(frame, line, (tx, ty), _font, _fscale, (0, 0, 0), 2, cv2.LINE_AA)
                                            cv2.putText(frame, line, (tx, ty), _font, _fscale, (0, 255, 0), 1, cv2.LINE_AA)
                                            ty += _lh

                                cv2.imshow(rgb_win_name, frame)

                            if self._showDepthWindow:
                                if not self._depth_win_sized:
                                    dh_win, dw_win = depthFrameColor.shape[:2]
                                    cv2.namedWindow(depth_win_name, cv2.WINDOW_NORMAL)
                                    cv2.resizeWindow(depth_win_name,
                                                     int(dw_win * DEPTH_WINDOW_SCALE),
                                                     int(dh_win * DEPTH_WINDOW_SCALE))
                                    self._depth_win_sized = True
                                cv2.imshow(depth_win_name, depthFrameColor)
                            
                            if self._closePictures:
                                for win in self.pictures_shown:
                                    cv2.destroyWindow(win)
                                self.pictures_shown.clear()
                                self._closePictures = False
                                
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
    mdai = MyDepthAI(model="yolo8nano", use_tracker=False)
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
    
    _roi_test_on = False
    def toggleRoiTest(a):
        global _roi_test_on
        _roi_test_on = not _roi_test_on
        if _roi_test_on:
            if not mdai.rgbWindowVisible():
                mdai.showRgbWindow(True)
            print("Spatial ROI test ON (20x20 center box)")
        else:
            mdai.clear_roi_rects()
            print("Spatial ROI test OFF")

    keyboard.on_press_key('r', toggleRgbWindow)
    keyboard.on_press_key('d', toggleDepthWindow)
    keyboard.on_press_key('t', toggleCamera)
    keyboard.on_press_key('s', toggleRoiTest)
    keyboard.on_press_key('q', shutdown)

    print("Keys: r=rgb, d=depth, t=camera, s=spatial ROI test, q=quit")

    try:
        while True:
            if _roi_test_on:
                # Query center 20x20 ROI with drawing enabled
                size = mdai.getPreviewSize()
                if size is not None:
                    pw, ph = size
                    half = 10
                    mdai.getSpatialForROI(
                        (pw // 2 - half) / pw, (ph // 2 - half) / ph,
                        (pw // 2 + half) / pw, (ph // 2 + half) / ph, draw=True)
            time.sleep(0.1)
    except KeyboardInterrupt:
        mdai.shutdown()
        my_depthai_thread.join()
