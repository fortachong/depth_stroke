import cv2
import numpy as np
import depthai as dai
from datetime import datetime
import queue as Queue
import threading

# Depth Map using OAKD camera and depthai library
class DepthMap:
    def __init__(self, fps=10, depth_stream_name='depth', cam_resolution='400'):
        # Camera options = 400, 720, 800
        cam_res = {
            '400': (
                    dai.MonoCameraProperties.SensorResolution.THE_400_P,
                    640,
                    400
                ),
            '720': (
                    dai.MonoCameraProperties.SensorResolution.THE_720_P,
                    1280,
                    720
                ),
            '800': (
                    dai.MonoCameraProperties.SensorResolution.THE_800_P,
                    1280,
                    800
                )
        }
        self.mono_resolution_left = cam_res[cam_resolution][0]
        self.mono_resolution_right = cam_res[cam_resolution][0]
        self.res_w = cam_res[cam_resolution][1]
        self.res_h = cam_res[cam_resolution][2]
        self.fps = fps
        self.depth_stream_name = depth_stream_name
        self.capture = False
        self.depth_frame = None
        self.show_display = True

    def initialize_display(self):
        if self.show_display:
            self.depth_img = np.zeros((self.res_h, self.res_w , 3), np.uint8)        
            cv2.imshow(self.depth_stream_name, self.depth_img)
        
    # Pipeline
    def create_pipeline(self):
        print("[{}]: Creating Pipeline...".format(
            datetime.now().strftime("%d/%m/%Y %H:%M:%S")
            )
        )
        # Pipeline
        self.pipeline = dai.Pipeline()
        print("[{}]: Setting up Depth...".format(
            datetime.now().strftime("%d/%m/%Y %H:%M:%S")
            )
        )
        # Mono Left Camera
        mono_l = self.pipeline.createMonoCamera()
        # Mono Right Camera
        mono_r = self.pipeline.createMonoCamera()
        # Mono Camera Settings
        mono_l.setResolution(self.mono_resolution_left)
        mono_l.setBoardSocket(dai.CameraBoardSocket.LEFT)
        mono_r.setResolution(self.mono_resolution_right)
        mono_r.setBoardSocket(dai.CameraBoardSocket.RIGHT)
        mono_l.setFps(self.fps)
        mono_r.setFps(self.fps)
        # Depth and Output
        stereo = self.pipeline.createStereoDepth()
        xout_depth = self.pipeline.createXLinkOut()
        # Stream Names
        xout_depth.setStreamName(self.depth_stream_name)
        # Stereo Depth parameters 
        output_depth = True
        output_rectified = False
        lr_check = False
        subpixel = False
        stereo.setOutputDepth(output_depth)
        stereo.setOutputRectified(output_rectified)
        stereo.setConfidenceThreshold(255)
        stereo.setLeftRightCheck(lr_check)
        stereo.setSubpixel(subpixel)
        # Mono L / R -> Stereo L / R
        mono_l.out.link(stereo.left)
        mono_r.out.link(stereo.right)
        # Stereo Depth -> Out
        stereo.depth.link(xout_depth.input)
        
    # Capture (loop), queue is a blocking threadsafe queue
    def start_capture(self, queue):
        self.initialize_display()
        self.create_pipeline()
        with dai.Device(self.pipeline) as device:
            pip = device.startPipeline()
            print("Pipeline started: {}".format(pip))
            
            # Dai queue
            # 1. Out: Depth
            q_d = device.getOutputQueue(name=self.depth_stream_name, maxSize=4, blocking=False)
            self.capture = True
            while self.capture:
                in_depth = q_d.get()
                self.depth_frame = in_depth.getFrame()
                self.show(queue)
                # Produce a map and copy to queue
                message = {
                    'command': 'depth',
                    'data': self.depth_frame
                }
                queue.put(message)

    # Show display
    def show(self, queue):
        key = cv2.waitKey(1) 
        if key == ord('q') or key == 27:
            queue.put({'command': 'stop'})
            self.stop_capture()
            cv2.destroyAllWindows()

        if self.depth_frame is not None:
            dframe = self.depth_frame.copy()
            depth_frame_color = cv2.normalize(dframe, None, 255, 0, cv2.NORM_INF, cv2.CV_8UC1)
            depth_frame_color = cv2.equalizeHist(depth_frame_color)
            depth_frame_color = cv2.applyColorMap(depth_frame_color, cv2.COLORMAP_SPRING)    
            cv2.imshow(self.depth_stream_name, depth_frame_color)

    def stop_capture(self):
        self.capture = False


class MessageProcessor(threading.Thread):
    def __init__(
            self,
            queue
        ):
        threading.Thread.__init__(self)
        self.active = True
        self.queue = queue

    # Process message
    def process(self, message):
        # Normalize depth map
        # print(message)
        pass
           
    # Run thread
    def run(self):
        while self.active:
            message = self.queue.get()
            if 'command' in message:
                if message['command'] == 'stop':
                    self.active = False
                else:
                    self.process(message)

if __name__ == "__main__":
    queue = Queue.Queue()
    # Process Thread
    mp = MessageProcessor(queue)
    mp.start()
    # Depth Map
    depth = DepthMap()
    depth.start_capture(queue)
    queue.put({'command': 'stop'})