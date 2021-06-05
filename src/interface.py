import cv2
import numpy as np
import depthai as dai
from datetime import datetime
import queue as Queue
import threading
import depth
import grid

class CVGridInterface(depth.DepthMap):
    def __init__(self, fps=30, fun='mean', grid_size=16):
        super(CVGridInterface, self).__init__(fps=fps)
        # Types of aggregaton function mean, median, min, max
        self.fun = fun
        self.grid_size = grid_size
        self.grid = grid.Grid(size=self.grid_size)
        self.spos, self.epos = self.grid.generate()

    # Use the grid to average depth and returns array
    def discretize_over_grid(self, depth_map):
        cells = []
        for spos_x, spos_y, epos_x, epos_y in zip(self.spos, self.epos):
            cell_data = depth_map[spos_y:epos_y+1, spos_x:epos_x+1]
            cell_agg = np.mean(cell_data)
            if self.fun == 'median':
                cell_agg = np.median(cell_data)
            if self.fun == 'min':
                cell_agg = np.min(cell_data)
            if self.fun == 'max':
                cell_agg = np.max(cell_data)
            
            cells.append(cell_agg)

        # re-arrange cells list into a 2d map
        cells_map = np.reshape(np.array(cells), (-1, self.grid_size)).T
        return cells_map

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
                cells = self.discretize_over_grid(depth_frame)
                message = {
                    'command': 'depth',
                    'data': depth_frame,
                    'cells': cells
                }
                queue.put(message)

    
