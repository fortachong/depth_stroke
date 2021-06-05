import cv2
import numpy as np

class Grid:
    def __init__(
        self,
        size=64,
        spacing=2,
        square_size=5,
        screen_width=800,
        screen_heigth=600,
        center=True,
        starting_x=0,
        starting_y=0
    ):
        # Number of squares
        self.size = size
        # Spacing between squares
        self.spacing = spacing
        # Square size
        self.square_size = square_size
        # screen size
        self.screen_width = screen_width
        self.screen_height = screen_heigth
        # Center
        self.center = center
        # Starting position in x and y
        self.starting_x = starting_x
        self.starting_y = starting_y

    def generate(self):
        total_size = self.size * self.square_size + (self.size-1) * self.spacing
        if self.center:
            self.starting_x = self.screen_width//2 - total_size//2
            self.starting_y = self.screen_height//2 - total_size//2
        
        starting_pos = []
        ending_pos = []
        offset_x = self.starting_x
        for _ in range(self.size):
            pos_x = offset_x
            offset_y = self.starting_y
            for _ in range(self.size):
                pos_y = offset_y
                starting_pos.append((pos_x, pos_y))
                ending_pos.append((pos_x+self.square_size, pos_y+self.square_size))
                offset_y += self.square_size + self.spacing
            offset_x += self.square_size + self.spacing
        return starting_pos, ending_pos
        

if __name__ == "__main__":
    screen_width = 800
    screen_height = 600
    grid = Grid(size=16, square_size=24, screen_width=screen_width, screen_heigth=screen_height)
    spos, epos = grid.generate()
    
    screen = np.zeros((screen_height,screen_width,3), np.uint8)
    color_bgr = (81,111,231)
    for (spos_x, spos_y), (epos_x, epos_y) in zip(spos, epos):
        cv2.rectangle(screen, (spos_x, spos_y), (epos_x, epos_y), color_bgr, -1)
        
    while True:
        cv2.imshow("grid", screen)
        key = cv2.waitKey(1) 
        if key == ord('q') or key == 27:
            break
        elif key == 32:
            # Pause on space bar
            cv2.waitKey(0)