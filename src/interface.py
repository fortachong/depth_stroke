import cv2
import numpy as np
import depthai as dai
from datetime import datetime
import queue as Queue
import threading
import depth
import grid

# Grid Interface
class CVGridInterface(depth.DepthMap):
    def __init__(
            self, 
            fps=20, 
            fun='mean', 
            grid_size=16, 
            square_size=10,
            spacing=2,
            depth_stream_name='depth', 
            cam_resolution='400'
        ):
        super(CVGridInterface, self).__init__(
            fps=fps, 
            depth_stream_name=depth_stream_name, 
            cam_resolution=cam_resolution
        )
        self.depth_resolution = 2**16
        # Types of aggregaton function mean, median, min, max
        self.fun = fun
        self.grid_size = grid_size
        self.screen_width = self.res_w
        self.screen_height = self.res_h
        self.square_size = square_size
        self.spacing = spacing
        self.grid = grid.Grid(
            size=self.grid_size, 
            screen_width=self.screen_width, 
            screen_heigth=self.screen_height,
            square_size=self.square_size,
            spacing=self.spacing
        )
        self.spos, self.epos = self.grid.generate()

    def initialize_display(self):
        if self.show_display:
            self.depth_img = np.zeros((self.res_h, self.res_w , 3), np.uint8)        
            cv2.imshow(self.depth_stream_name, self.depth_img)

    def show_grid(self):
        grid = np.zeros((self.screen_height,self.screen_width,3), np.uint8)
        color_bgr = (81,111,231)
        for (spos_x, spos_y), (epos_x, epos_y) in zip(self.spos, self.epos):
            cv2.rectangle(grid, (spos_x, spos_y), (epos_x, epos_y), color_bgr, -1)
        cv2.imshow('grid_configuration', grid)

    # Use the grid to average depth
    def discretize_over_grid(self, depth_map):
        dm = depth_map.copy()
        cells = []
        for (spos_x, spos_y), (epos_x, epos_y) in zip(self.spos, self.epos):
            cell_data = depth_map[spos_y:epos_y+1, spos_x:epos_x+1]
            # print(cell_data)
            cell_agg = np.mean(cell_data)
            if self.fun == 'median':
                cell_agg = np.median(cell_data)
            if self.fun == 'min':
                cell_agg = np.min(cell_data)
            if self.fun == 'max':
                cell_agg = np.max(cell_data)
            
            cells.append(cell_agg)
            dm[spos_y:epos_y+1, spos_x:epos_x+1] = cell_agg

        # re-arrange cells list into a 2d map
        cells_map = np.reshape(np.array(cells), (-1, self.grid_size)).T
        # Lists of cells, nparray 2d of cells, depth_map modified
        return cells, cells_map, dm

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
                self.cells, self.cells_map, self.dm = self.discretize_over_grid(self.depth_frame)

                self.show(queue)
                # Produce a map and copy to queue
                message = {
                    'command': 'depth',
                    'data': self.depth_frame,
                    'spos': self.spos,
                    'epos': self.epos,
                    'cells': self.cells
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
            depth_frame_color = cv2.applyColorMap(depth_frame_color, cv2.COLORMAP_HSV)    
            cv2.imshow(self.depth_stream_name, depth_frame_color)
            # Show the discretized depth_map
            dm_ = self.dm.copy()
            grid_frame_color = cv2.normalize(dm_, None, 255, 0, cv2.NORM_INF, cv2.CV_8UC1)
            grid_frame_color = cv2.equalizeHist(grid_frame_color)
            grid_frame_color = cv2.applyColorMap(grid_frame_color, cv2.COLORMAP_OCEAN)  
            cv2.imshow("Interface", grid_frame_color)
            

    def stop_capture(self):
        self.capture = False                

# Thread that process messages through a thread safe queue
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

TYPE = 'GRID'
# TYPE = 'DEPTH'
if __name__ == "__main__":
    # Just to show the Grid using a normal camera
    if TYPE == 'GRID':
        cam = cv2.VideoCapture(0)
        gridder = CVGridInterface(
            square_size=12,
            spacing=10,
            fun='mean'
        )

        if not cam.isOpened():
            print("Error in Camera")
            exit()

        gridder.show_grid()        
        while True:
            ret, frame = cam.read()
            # if frame is read correctly ret is True
            if not ret:
                print("Cannot read frame")
                break
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            cells, cells_map, img = gridder.discretize_over_grid(gray)
            # display
            cv2.imshow('original', gray)   
            # display grid
            cv2.imshow('grid', img)
            if cv2.waitKey(1) == ord('q'):
                break
            
        cam.release()
        cv2.destroyAllWindows()

    # Show the grid using the OAKD
    if TYPE == 'DEPTH':
        queue = Queue.Queue()
        # Process Thread
        mp = MessageProcessor(queue)
        mp.start()
        # Depth Map
        depth = CVGridInterface(
            square_size=24,
            fun='mean'
        )
        depth.start_capture(queue)
        queue.put({'command': 'stop'})
