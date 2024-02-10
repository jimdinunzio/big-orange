# coding=utf-8
import os
from pathlib import Path
from queue import Queue
import argparse
import time
from copy import deepcopy

import cv2
import depthai
import numpy as np
from imutils.video import FPS
from threading import Lock
from math import asin, degrees

TOP_MOUNTED_OAK_D_ID = "14442C103147C2D200"

# parser = argparse.ArgumentParser()
# parser.add_argument(
#     "-nd", "--no-debug", action="store_true", help="prevent debug output"
# )
# parser.add_argument(
#     "-cam",
#     "--camera",
#     action="store_true",
#     help="Use DepthAI 4K RGB camera for inference (conflicts with -vid)",
# )

# parser.add_argument(
#     "-vid",
#     "--video",
#     type=str,
#     help="The path of the video file used for inference (conflicts with -cam)",
# )

# parser.add_argument(
#     "-db",
#     "--databases",
#     action="store_true",
#     help="Save data (only used when running recognition network)",
# )

# parser.add_argument(
#     "-n",
#     "--name",
#     type=str,
#     default="",
#     help="Data name (used with -db) [Optional]",
# )

# args = parser.parse_args()

# debug = not args.no_debug

# is_db = args.databases

# if args.camera and args.video:
#     raise ValueError(
#         'Command line parameter error! "-Cam" cannot be used together with "-vid"!'
#     )
# elif args.camera is False and args.video is None:
#     raise ValueError(
#         'Missing inference source! Use "-cam" to run on DepthAI cameras, or use "-vid <path>" to run on video files'
#     )


def to_planar(arr: np.ndarray, shape: tuple):
    return cv2.resize(arr, shape).transpose((2, 0, 1)).flatten()


def to_nn_result(nn_data):
    return np.array(nn_data.getFirstLayerFp16())


def run_nn(x_in, x_out, in_dict):
    nn_data = depthai.NNData()
    for key in in_dict:
        nn_data.setLayer(key, in_dict[key])
    x_in.send(nn_data)
    return x_out.tryGet()


def frame_norm(frame, *xy_vals):
    return (
        np.clip(np.array(xy_vals), 0, 1) * np.array(frame * (len(xy_vals) // 2))[::-1]
    ).astype(int)


def correction(frame, angle=None, invert=False):
    h, w = frame.shape[:2]
    center = (w // 2, h // 2)
    mat = cv2.getRotationMatrix2D(center, angle, 1)
    affine = cv2.invertAffineTransform(mat).astype("float32")
    corr = cv2.warpAffine(
        frame,
        mat,
        (w, h),
        flags=cv2.INTER_CUBIC,
        borderMode=cv2.BORDER_CONSTANT,
    )
    if invert:
        return corr, affine
    return corr


def cosine_distance(a, b):
    if a.shape != b.shape:
        raise RuntimeError("array {} shape not match {}".format(a.shape, b.shape))
    a_norm = np.linalg.norm(a)
    b_norm = np.linalg.norm(b)
    similarity = np.dot(a, b.T) / (a_norm * b_norm)

    return similarity


databases = "databases"

if not os.path.exists(databases):
    os.mkdir(databases)


def create_db(face_fame, results, name):
    font = cv2.FONT_HERSHEY_PLAIN
    font_scale = 1
    font_color = (0, 255, 0)
    line_type = 1
    try:
        with np.load(f"{databases}/{name}.npz") as db:
            db_ = [db[j] for j in db.files][:]
    except Exception as e:
        db_ = []
    db_.append(np.array(results))
    np.savez_compressed(f"{databases}/{name}", *db_)


def read_db(labels):
    for file in os.listdir(databases):
        filename = os.path.splitext(file)
        if filename[1] == ".npz":
            label = filename[0]
            labels.add(label)
    db_dic = {}
    for label in list(labels):
        with np.load(f"{databases}/{label}.npz") as db:
            db_dic[label] = [db[j] for j in db.files]
    return db_dic

def get_known_faces():
    labels = set()
    for file in os.listdir(databases):
        filename = os.path.splitext(file)
        if filename[1] == ".npz":
            label = filename[0].strip()
            labels.add(label)
    return labels

class DepthAI:
    def __init__(
        self,
        file=None,
        camera=True,
        compute_spatial=False,
        debug=True
    ):
        print("Loading pipeline...")
        self.file = file
        self.camera = camera
        self.compute_spatial = compute_spatial
        self.fps_cam = FPS()
        self.fps_nn = FPS()
        self.create_pipeline()
        self.fontScale = 1 if self.camera else 2
        self.lineType = 0 if self.camera else 3
        self.debug = debug
        self.run_flag = False

    def shutdown(self):
        self.run_flag = False

    def create_pipeline(self):
        print("Creating pipeline...")
        self.pipeline = depthai.Pipeline()

        if self.camera:
            if self.compute_spatial:
                monoLeft = self.pipeline.create(depthai.node.MonoCamera)
                monoRight = self.pipeline.create(depthai.node.MonoCamera)
                stereo = self.pipeline.create(depthai.node.StereoDepth)
                spatialLocationCalculator = self.pipeline.create(depthai.node.SpatialLocationCalculator)

                xoutDepth = self.pipeline.create(depthai.node.XLinkOut)
                xoutSpatialData = self.pipeline.create(depthai.node.XLinkOut)
                xinSpatialCalcConfig = self.pipeline.create(depthai.node.XLinkIn)

                xoutDepth.setStreamName("depth")
                xoutSpatialData.setStreamName("spatialData")
                xinSpatialCalcConfig.setStreamName("spatialCalcConfig")

                # Properties
                monoLeft.setResolution(depthai.MonoCameraProperties.SensorResolution.THE_400_P)
                monoLeft.setCamera("left")
                monoRight.setResolution(depthai.MonoCameraProperties.SensorResolution.THE_400_P)
                monoRight.setCamera("right")

                stereo.setDefaultProfilePreset(depthai.node.StereoDepth.PresetMode.HIGH_DENSITY)
                stereo.setLeftRightCheck(False)
                stereo.setSubpixel(False)
                spatialLocationCalculator.inputConfig.setWaitForMessage(False)

                # Config
                topLeft = depthai.Point2f(0.4, 0.4)
                bottomRight = depthai.Point2f(0.6, 0.6)

                config = depthai.SpatialLocationCalculatorConfigData()
                config.depthThresholds.lowerThreshold = 100
                config.depthThresholds.upperThreshold = 10000
                config.roi = depthai.Rect(topLeft, bottomRight)

                spatialLocationCalculator.initialConfig.addROI(config)

                # Linking
                monoLeft.out.link(stereo.left)
                monoRight.out.link(stereo.right)

                spatialLocationCalculator.passthroughDepth.link(xoutDepth.input)
                stereo.depth.link(spatialLocationCalculator.inputDepth)

                spatialLocationCalculator.out.link(xoutSpatialData.input)
                xinSpatialCalcConfig.out.link(spatialLocationCalculator.inputConfig)

            # ColorCamera
            print("Creating Color Camera...")
            self.cam = self.pipeline.createColorCamera()
            self.cam.setPreviewSize(self._cam_size[1], self._cam_size[0])
            self.cam.setResolution(
                depthai.ColorCameraProperties.SensorResolution.THE_4_K
            )
            self.cam.setInterleaved(False)
            self.cam.setBoardSocket(depthai.CameraBoardSocket.RGB)
            self.cam.setColorOrder(depthai.ColorCameraProperties.ColorOrder.BGR)

            self.cam_xout = self.pipeline.createXLinkOut()
            self.cam_xout.setStreamName("preview")
            self.cam.preview.link(self.cam_xout.input)
            self.cam.setPreviewKeepAspectRatio(True) #is default, but show explicitly for doc.

        self.create_nns()

        print("Pipeline created.")

    def create_nns(self):
        pass

    def create_nn(self, model_path: str, model_name: str, first: bool = False):
        """

        :param model_path: model path
        :param model_name: model abbreviation
        :param first: Is it the first model
        :return:
        """
        # NeuralNetwork
        print(f"Creating {model_path} Neural Network...")
        model_nn = self.pipeline.createNeuralNetwork()
        model_nn.setBlobPath(str(Path(f"{model_path}").resolve().absolute()))
        model_nn.input.setBlocking(False)
        if first and self.camera:
            print("linked cam.preview to model_nn.input")
            self.cam.preview.link(model_nn.input)
        else:
            model_in = self.pipeline.createXLinkIn()
            model_in.setStreamName(f"{model_name}_in")
            model_in.out.link(model_nn.input)

        model_nn_xout = self.pipeline.createXLinkOut()
        model_nn_xout.setStreamName(f"{model_name}_nn")
        model_nn.out.link(model_nn_xout.input)

    def create_mobilenet_nn(
        self,
        model_path: str,
        model_name: str,
        conf: float = 0.5,
        first: bool = False,
    ):
        """

        :param model_path: model name
        :param model_name: model abbreviation
        :param conf: confidence threshold
        :param first: Is it the first model
        :return:
        """
        # NeuralNetwork
        print(f"Creating {model_path} Neural Network...")
        model_nn = self.pipeline.createMobileNetDetectionNetwork()
        model_nn.setBlobPath(str(Path(f"{model_path}").resolve().absolute()))
        model_nn.setConfidenceThreshold(conf)
        model_nn.input.setBlocking(False)

        if first and self.camera:
            self.cam.preview.link(model_nn.input)
        else:
            model_in = self.pipeline.createXLinkIn()
            model_in.setStreamName(f"{model_name}_in")
            model_in.out.link(model_nn.input)

        model_nn_xout = self.pipeline.createXLinkOut()
        model_nn_xout.setStreamName(f"{model_name}_nn")
        model_nn.out.link(model_nn_xout.input)

    def start_pipeline(self):
        found, device_info = depthai.Device.getDeviceByMxId(TOP_MOUNTED_OAK_D_ID)
        if not found:
            raise RuntimeError("Top mounted Oak-D device not found!")

        self.device = depthai.Device(self.pipeline, device_info)
        print("Starting pipeline...")

        if self.compute_spatial:
        	self.start_spatial()

        self.start_nns()

        if self.camera:
            self.preview = self.device.getOutputQueue(
                name="preview", maxSize=4, blocking=False
            )

    def start_nns(self):
        pass

    def start_spatial(self):
        pass
    
    def put_text(self, text, dot, color=(0, 0, 255), font_scale=None, line_type=None):
        if not self.debug:
            return
        font_scale = font_scale if font_scale else self.fontScale
        line_type = line_type if line_type else self.lineType
        dot = tuple(dot[:2])
        cv2.putText(
            img=self.debug_frame,
            text=text,
            org=dot,
            fontFace=cv2.FONT_HERSHEY_COMPLEX,
            fontScale=font_scale,
            color=color,
            lineType=line_type,
        )

    def draw_bbox(self, bbox, color):
        if not self.debug:
            return
        cv2.rectangle(
            img=self.debug_frame,
            pt1=(bbox[0], bbox[1]),
            pt2=(bbox[2], bbox[3]),
            color=color,
            thickness=2,
        )

    def parse(self):
        if self.debug:
            self.debug_frame = self.frame.copy()

        s = self.parse_fun()
        # if s :
        #     raise StopIteration()
        if self.debug:
            cv2.imshow(
                "Camera_view",
                self.debug_frame,
            )
            self.fps_cam.update()

    def parse_fun(self):
        pass

    def compute_spatial_coords(self):
        pass

    def run_video(self):
        self.run_flag = True
        cap = cv2.VideoCapture(str(Path(self.file).resolve().absolute()))
        while cap.isOpened() and self.run_flag:
            read_correctly, self.frame = cap.read()
            if not read_correctly:
                break

            try:
                self.parse()
            except StopIteration:
                print("exception: StopIteration")
                break
            cv2.waitKey(50)
        cap.release()
        cv2.destroyAllWindows()
        self.fps_cam.stop()
        self.fps_nn.stop()

    def run_camera(self):
        self.run_flag = True
        while self.run_flag:
            in_rgb = self.preview.tryGet()
            if in_rgb is not None:
                shape = (3, in_rgb.getHeight(), in_rgb.getWidth())
                self.frame = (
                    in_rgb.getData().reshape(shape).transpose(1, 2, 0).astype(np.uint8)
                )
                self.frame = np.ascontiguousarray(self.frame)
                try:
                    self.parse()
                except StopIteration:
                    print("exception: StopIteration")
                    break
                if self.compute_spatial:
                    self.compute_spatial_coords()
            cv2.waitKey(50)

        cv2.destroyAllWindows()
        self.fps_cam.stop()
        self.fps_nn.stop()
        print(
            f"FPS_CAMERA: {self.fps_cam.fps():.2f} , FPS_NN: {self.fps_nn.fps():.2f}"
        )

    @property
    def cam_size(self):
        return self._cam_size

    @cam_size.setter
    def cam_size(self, v):
        self._cam_size = v

    @property
    def run_flag(self):
        return self._run_flag
    
    @run_flag.setter
    def run_flag(self, v):
        self._run_flag = v

    def run(self):
        try_count = 3
        while try_count > 0:
            try:
                self.start_pipeline()
                break
            except:
                if try_count > 1:
                    time.sleep(2)
                    try_count -= 1
                else:
                    return
        self.fps_cam.start()
        self.fps_nn.start()
        if self.file is not None:
            self.run_video()
        else:
            self.run_camera()
        del self.device

class FaceDetection:
    def __init__(self, name : str, coords_3d : tuple = (0,0,0)):
        self.name = name
        self.x = coords_3d[0] / 1000.0
        self.y = coords_3d[1] / 1000.0
        self.z = coords_3d[2] / 1000.0
        self.theta = degrees(-asin(self.x/self.z) if self.z != 0.0 else 0)
    def __repr__(self):
        return "{{name = {}, x = {}, y = {}, z = {}, theta = {}}}".format(self.name, self.x, self.y, self.z, self.theta)

class FacialRecognize(DepthAI):
    def __init__(self, getPitch=None, offsetPitch=None, getYaw=None, offsetYaw=None, compute_spatial=False,
    			 file=None, camera=True, debug=True, add_face=False, new_name=""):
        self.cam_size = (300, 300)
        super(FacialRecognize, self).__init__(file, camera, compute_spatial, debug)
        self.face_frame_corr = Queue()
        self.face_frame = Queue()
        self.face_coords = Queue()
        self.labels = set()
        self.add_face = add_face
        self.new_name = new_name
        self.detection_lock = Lock()
        self.raw_detections_list = [[],[]]
        self.raw_detections_ready =[False, False]
        if not add_face:
            self.db_dic = read_db(self.labels)
        self.getPitch=getPitch
        self.offsetPitch=offsetPitch
        self.getYaw=getYaw
        self.offsetYaw=offsetYaw
        self.search_dir=-1
        self.detections_idx = 0
        self.face_added = False

    def was_face_added(self):
        return self.face_added
    
    def other_detections_idx(self):
        return int(not self.detections_idx)

    def rgb_to_depthview(self, x, y):
        # depth frame is 640x400, rgb preview is 300x300
        # for rgb preview 4k camera is 3840x2160 scaled down to 533x300 then cropped
        # based on calculations this should be 640/533 = 1.2 factor with (533-300)/2 = 116 crop offset.
        # y seems to be just cropped 
        # but it doesn't line up
        out_x = 1.12 * (x + 117)
        out_y = y + 50
        return depthai.Point2f(out_x / 640.0, out_y / 400.0)

    def compute_spatial_coords(self):
        #print("------------------------------------------------------------")

        cfg = depthai.SpatialLocationCalculatorConfig()
        num_detections = len(self.raw_detections_list[self.detections_idx])
        idx = 0
        for n in self.raw_detections_list[self.detections_idx]:
            config = depthai.SpatialLocationCalculatorConfigData()
            config.depthThresholds.lowerThreshold = 100
            config.depthThresholds.upperThreshold = 10000        
            face_coords = n["face_coords"]
            topLeft = self.rgb_to_depthview(face_coords[0], face_coords[1])
            bottomRight = self.rgb_to_depthview(face_coords[2], face_coords[3])
            config.roi = depthai.Rect(topLeft, bottomRight)
            config.calculationAlgorithm = depthai.SpatialLocationCalculatorAlgorithm.MEDIAN
            cfg.addROI(config)
            idx += 1
    
        if num_detections == 0:
            #add one dummy ROI to avoid errors
            config = depthai.SpatialLocationCalculatorConfigData()
            config.roi = depthai.Rect(depthai.Point2f(0,0), depthai.Point2f(0,0))
            cfg.addROI(config)

        self.spatialCalcConfigInQueue.send(cfg)

        inDepth = self.depthQueue.get() # Blocking call, will wait until a new data has arrived

        # Code to display the depth frame is commented out below. Used for debugging and visualization

        # depthFrame = inDepth.getFrame()
        # depthFrameColor = cv2.normalize(depthFrame, None, 255, 0, cv2.NORM_INF, cv2.CV_8UC1)
        # depthFrameColor = cv2.equalizeHist(depthFrameColor)
        # depthFrameColor = cv2.applyColorMap(depthFrameColor, cv2.COLORMAP_HOT)
        
        # we submit spatial calculation above, but it will not come back until next frame,
        # the depthData retrieved below is for the previous frame. We use a buffer of 2 and swap between them.
        # and then the spatial data and face data will be for the same frame.
        with self.detection_lock:
            self.detections_idx = self.other_detections_idx()

            num_detections = len(self.raw_detections_list[self.detections_idx])
            if num_detections > 0:
                spatialData = self.spatialCalcQueue.get().getSpatialLocations()
                nameIdx = 0
                for depthData in spatialData:
                    #roi = depthData.config.roi
                    #n = self.raw_detections_list[self.detections_idx][nameIdx]
                    #face_coords = n["face_coords"]
                    #topLeft = self.rgb_to_depthview(face_coords[0], face_coords[1])
                    #bottomRight = self.rgb_to_depthview(face_coords[2], face_coords[3])
                    #match_roi = depthai.Rect(topLeft, bottomRight)                
                    #print("raw detectio #{}: roi = {}, {}, {}, {}".format(nameIdx, match_roi.x, match_roi.y, match_roi.width, match_roi.height))
                    #print("spatial data #{}: roi = {}, {}, {}, {}".format(nameIdx, roi.x, roi.y, roi.width, roi.height))
                    # roi = roi.denormalize(width=depthFrameColor.shape[1], height=depthFrameColor.shape[0])
                    # xmin = int(roi.topLeft().x)
                    # ymin = int(roi.topLeft().y)
                    # xmax = int(roi.bottomRight().x)
                    # ymax = int(roi.bottomRight().y)

                    # fontType = cv2.FONT_HERSHEY_TRIPLEX
                    # cv2.rectangle(depthFrameColor, (xmin, ymin), (xmax, ymax), self.color,  cv2.FONT_HERSHEY_SCRIPT_SIMPLEX)
                    # cv2.putText(depthFrameColor, f"X: {int(depthData.spatialCoordinates.x)} mm", (xmin + 10, ymin + 20), fontType, 0.5, self.color)
                    # cv2.putText(depthFrameColor, f"Y: {int(depthData.spatialCoordinates.y)} mm", (xmin + 10, ymin + 35), fontType, 0.5, self.color)
                    # cv2.putText(depthFrameColor, f"Z: {int(depthData.spatialCoordinates.z)} mm", (xmin + 10, ymin + 50), fontType, 0.5, self.color)
                    if nameIdx < num_detections:
                        self.raw_detections_list[self.detections_idx][nameIdx]["3d_coords"]= \
                        (depthData.spatialCoordinates.x,
                            depthData.spatialCoordinates.y,
                            depthData.spatialCoordinates.z)
                    #print("spatial coords = {}, {}, {}".format(depthData.spatialCoordinates.x, depthData.spatialCoordinates.y, depthData.spatialCoordinates.z))
                    nameIdx += 1
                self.raw_detections_ready[self.detections_idx] = True
        # Show the frame
        #cv2.imshow("depth", depthFrameColor)

    def start_spatial(self):
        # Output queue will be used to get the depth frames from the outputs defined above
        self.depthQueue = self.device.getOutputQueue(name="depth", maxSize=4, blocking=False)
        self.spatialCalcQueue = self.device.getOutputQueue(name="spatialData", maxSize=4, blocking=False)
        self.spatialCalcConfigInQueue = self.device.getInputQueue("spatialCalcConfig")

        self.color = (255, 255, 255)

    def create_nns(self):
        self.create_mobilenet_nn(
            "models/face-detection-retail-0004_openvino_2021.2_4shave.blob",
            "mfd",
            first=self.camera,
        )

        self.create_nn(
            "models/head-pose-estimation-adas-0001_openvino_2021.2_4shave.blob",
            "head_pose",
        )
        self.create_nn(
            "models/face-recognition-mobilefacenet-arcface_2021.2_4shave.blob",
            "arcface",
        )

    def start_nns(self):
        if not self.camera:
            self.mfd_in = self.device.getInputQueue("mfd_in")
        self.mfd_nn = self.device.getOutputQueue("mfd_nn", 4, False)
        self.head_pose_in = self.device.getInputQueue("head_pose_in", 4, False)
        self.head_pose_nn = self.device.getOutputQueue("head_pose_nn", 4, False)
        self.arcface_in = self.device.getInputQueue("arcface_in", 4, False)
        self.arcface_nn = self.device.getOutputQueue("arcface_nn", 4, False)

    def run_face_mn(self):
        if not self.camera:
            nn_data = run_nn(
                self.mfd_in, self.mfd_nn, {"data": to_planar(self.frame, (300, 300))}
            )
        else:
            nn_data = self.mfd_nn.tryGet()
        if nn_data is None:
            return False

        bboxes = nn_data.detections
        if len(bboxes) == 0:
            # try moving camera up and down until a face is detected
            if self.getPitch and self.offsetPitch:
                self.offsetPitch(self.search_dir)
                pitch = self.getPitch()
                if pitch <= -60:
                    self.search_dir = 1
                elif pitch >= -10:
                    self.search_dir = -1 
        count = 0
        for bbox in bboxes:
            face_coord = frame_norm(
                self.frame.shape[:2], *[bbox.xmin, bbox.ymin, bbox.xmax, bbox.ymax]
            )
            # center camera on the first face
            if self.offsetPitch and self.offsetYaw and count == 0:
                x_ctr = (face_coord[0] + face_coord[2]) / (2.0 * self.cam_size[0])
                y_ctr = (face_coord[1] + face_coord[3]) / (2.0 * self.cam_size[1])

                y_diff = y_ctr - 0.5
                adj = y_diff * 6
                if abs(adj) < 1.0:
                    adj = 0.0                
                self.offsetPitch(adj)

                x_diff = 0.5 - x_ctr
                adj = x_diff * 6
                if abs(adj) < 1.0:
                    adj = 0.0
                self.offsetYaw(adj)

            self.face_frame.put(
                self.frame[face_coord[1] : face_coord[3], face_coord[0] : face_coord[2]]
            )
            self.face_coords.put(face_coord)
            self.draw_bbox(face_coord, (10, 245, 10))
            count += 1
        return True

    def run_head_pose(self):
        while self.face_frame.qsize():
            face_frame = self.face_frame.get()
            nn_data = run_nn(
                self.head_pose_in,
                self.head_pose_nn,
                {"data": to_planar(face_frame, (60, 60))},
            )
            if nn_data is None:
                return False

            out = np.array(nn_data.getLayerFp16("angle_r_fc"))
            self.face_frame_corr.put(correction(face_frame, -out[0]))

        return True

    def run_arcface(self):
        with self.detection_lock:
            raw_detections = []        
            while self.face_frame_corr.qsize():
                face_coords = self.face_coords.get()
                face_fame = self.face_frame_corr.get()

                nn_data = run_nn(
                    self.arcface_in,
                    self.arcface_nn,
                    {"data": to_planar(face_fame, (112, 112))},
                )

                if nn_data is None:
                    return False
                self.fps_nn.update()
                results = to_nn_result(nn_data)

                if self.add_face:
                    create_db(face_fame, results, self.new_name)
                    self.face_added = True
                else:
                    conf = []
                    max_ = 0
                    label_ = None
                    for label in list(self.labels):
                        for j in self.db_dic.get(label):
                            conf_ = cosine_distance(j, results)
                            if conf_ > max_:
                                max_ = conf_
                                label_ = label
                    conf.append((max_, label_))
                    if conf[0][0] >= 0.5:
                        detected_name = conf[0]
                        raw_detections.append({"name": conf[0][1], "face_coords": face_coords})
                    else:
                        detected_name = (1 - conf[0][0], "UNKNOWN")
                        raw_detections.append({"name": detected_name, "face_coords": face_coords})
                    self.put_text(
                        f"name:{detected_name[1]}",
                        (face_coords[0], face_coords[1] - 35),
                        (244, 0, 255),
                    )
                    self.put_text(
                        f"conf:{detected_name[0] * 100:.2f}%",
                        (face_coords[0], face_coords[1] - 10),
                        (244, 0, 255),
                    )
            self.raw_detections_ready[self.detections_idx] = False
            self.raw_detections_list[self.detections_idx] = deepcopy(raw_detections)
        return True

    def parse_fun(self):
        if self.run_face_mn():
            if self.run_head_pose():
                if self.run_arcface():
                    return True

    def get_detections(self) -> []:
        detections = []
        if self.compute_spatial:
            i = 0
            while len(self.raw_detections_list[0]) > 0 or len(self.raw_detections_list[1]) > 0:
                i = 0
                with self.detection_lock:
                    if self.raw_detections_ready[i] == True:
                        break
                i += 1
                with self.detection_lock:
                    if self.raw_detections_ready[i] == True:
                        break
                time.sleep(0.05)

            with self.detection_lock:
                i = 0
                if self.raw_detections_ready[i] == False:
                    i += 1
                    if self.raw_detections_ready[i] == False:
                        return detections

                for rd in self.raw_detections_list[i]:
                    detections.append(FaceDetection(rd["name"], rd.get("3d_coords", (0,0,0))))
        else: # not self.compute_spatial
            with self.detection_lock:
                for rd in self.raw_detections_list[self.detections_idx]:
                    detections.append(FaceDetection(rd["name"]))
        return detections

if __name__ == "__main__":

    #add a new face to database with camera input

    # import facial_recognize as fr
    # from threading import Thread
    # f = fr.FacialRecognize(add_face=True, new_name="Jim")
    # t=Thread(target=f.run).start()

    #test facial recognition with camera input

    import time
    import facial_recognize as fr
    from threading import Thread
    from latte_panda_arduino import LattePandaArduino
    from move_oak_d import MoveOakD

    def getPitch():
        return m.getPitch()
    def offsetPitch(val):
        m.offsetPitch(val)
    def getYaw():
        return m.getYaw()
    def offsetYaw(val):
        m.offsetYaw(val)

    _lpArduino = LattePandaArduino()
    _lpArduino.initialize()
    m = MoveOakD()
    m.initialize(_lpArduino.board)

    f = FacialRecognize(getPitch, offsetPitch, getYaw, offsetYaw, compute_spatial=True)
    t=Thread(target=f.run).start()
    try:
        while True:
            detections = f.get_detections()
            for detection in detections:
                if detection.x != 0 or detection.y != 0 or detection.z != 0:
                    print("{} is seen at location x = {}, y = {}, z = {}".format(detection.name, 
                                                                                 detection.x,
                                                                                 detection.y,
                                                                                 detection.z))
                else:
                    print("{} is seen.".format(detection.name))
            time.sleep(1)
    except KeyboardInterrupt:
        f.shutdown()
        m.shutdown()
        _lpArduino.shutdown()
