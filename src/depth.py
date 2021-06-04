import cv2
import depthai as dai
from datetime import datetime
import queue as Queue
import threading

class DepthMap:
    def __init__(self, fps=30, depth_stream_name='depth'):
        self.mono_resolution_left = dai.MonoCameraProperties.SensorResolution.THE_800_P
        self.mono_resolution_right = dai.MonoCameraProperties.SensorResolution.THE_800_P
        self.fps = fps
        self.depth_stream_name = depth_stream_name
        self.capture = False
        
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
                depth_frame = in_depth.getFrame()
                # Produce a map and copy to queue
                message = {
                    'command': 'depth',
                    'data': depth_frame
                }
                queue.put(message)

    def stop_capture(self):
        self.capture = False


class MessageProcessor(threading.Thread):
    def __init__(
            self,
            queue
        ):
        threading.Thread.__init__(self)
        self.queue = queue

    # Process message
    def process(self, message):        
        pass
    
    # Run thread
    def run(self):
        global STOP
        while self.active:
            message = self.queue.get()
            if 'command' in message:
                if message['command'] == 'stop':
                    self.active = False
                else:
                    self.process(message)

if __name__ == "__main__":
    STOP = False
    queue = Queue()
    # Process Thread
    mp = MessageProcessor(queue)
    mp.start()
    # Depth Map
    depth = DepthMap()
    depth.start_capture(queue)

    while not STOP:
        key = cv2.waitKey(1) 
        if key == ord('q') or key == 27:
            depth.stop_capture()
            queue.put({'command': 'stop'})
            STOP = True
