import cv2
import numpy as np
import depthai as dai
from datetime import datetime
import queue as Queue
import threading
import depth
import grid

class CVGridInterface(depth.DepthMap):
    def __init__(
            self, 
            fps=20, 
            fun='mean', 
            grid_size=16, 
            screen_width=800, 
            screen_height=600,
            square_size=10
        ):
        super(CVGridInterface, self).__init__(fps=fps)
        # Types of aggregaton function mean, median, min, max
        self.fun = fun
        self.grid_size = grid_size
        self.screen_width = screen_width
        self.screen_height = screen_height
        self.square_size = square_size
        self.grid = grid.Grid(
            size=self.grid_size, 
            screen_width=self.screen_width, 
            screen_heigth=self.screen_height,
            square_size=self.square_size
        )
        self.spos, self.epos = self.grid.generate()

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
        return cells, cells_map, dm

    # Capture (loop), queue is a blocking threadsafe queue
    def start_capture(self, queue):
        pass


if __name__ == "__main__":
    cam = cv2.VideoCapture(0)
    gridder = CVGridInterface(
        screen_width=640, 
        screen_height=480, 
        square_size=24,
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
