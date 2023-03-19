import numpy as np
import cv2 as cv

import pyclipper
from obstacle_model import *

class environment:
    """Class contains all the funcitonality to create and manipulate environemnt and its visualization
    """

    
    def inflate_polygon(self, vertices, radius):
        pco = pyclipper.PyclipperOffset()
        pco.AddPath(vertices, pyclipper.PT_CLIP, pyclipper.ET_CLOSEDPOLYGON)
        vertices_inflated = pco.Execute(radius)
        vertices_inflated = [tuple(x) for x in vertices_inflated[0]]
        return vertices_inflated

    def __init__(self, height, width, inflation_radius) -> None:
        """Initialize environment parameters

        Args:
            height (int): height of the map
            width (int): width of the map
        """
        self.height = height
        self.width = width
        self.inflation_radius = inflation_radius
        #create a map fo given dimentions. 3 channels for opencv BGR
        self.map = np.ones((height, width, 3))

        

        #create obstacle models for all the objects in the environment

        #create original boundary obstacle model
        self.boundary_model = obstacle_model([
            [(600,0), (600,250), (0,250), (0,0)]
            ])
        
        #create inflated boundary obstacle model
        self.inflated_boundary_model = obstacle_model([
            self.inflate_polygon([(600,0), (600,250), (0,250), (0,0)],  -1*self.inflation_radius),
            ])
        
        #create original polygon objects obstacle model
        self.original_obstacle_model = obstacle_model([
            [(150,0), (150,100), (100,100), (100,0)],                           # Polygon corresponding to botom rectangular pillar
            [(150,150), (150,250), (100,250), (100,150)],                       # Polygon corresponding to Top rectangular pillar
            [(360,87), (360,163), (300,200), (240,163), (240,87), (300,50)],    # Polygon corresponding to hexgon 
            [(510,125), (460,225), (460,25) ],                                  # Polygon corresponding to Triangle 
        ])      


        #create inflated polygon objects obstacle model
        self.inflated_obstacle_model = obstacle_model([
            self.inflate_polygon([(150,0), (150,100), (100,100), (100,0)],  self.inflation_radius),
            self.inflate_polygon([(150,150), (150,250), (100,250), (100,150)],  self.inflation_radius),
            self.inflate_polygon([(360,87), (360,163), (300,200), (240,163), (240,87), (300,50)],  self.inflation_radius),
            self.inflate_polygon([(510,125), (460,225), (460,25) ],  self.inflation_radius),  
        ])  


    
    def create_map(self):
        """Idnetify obstacles and free space in the map uisng the obstacle models
        """
        #Iterate through all the states in the enviornement
        for i in range(self.map.shape[0]):
            for j in range(self.map.shape[1]):
                #Checks if state present inside the non-inflated obstacle
                if self.original_obstacle_model.is_inside_obstacle((j,i)):
                    self.map[i,j] = [0x80, 0x80, 0xff]
                
                #Checks if state present inside an nflated obstacle
                elif self.inflated_obstacle_model.is_inside_obstacle((j,i)):
                    self.map[i,j] = [0x7a, 0x70, 0x52]

                #Checks if state present outside the inflated boundary
                elif not self.inflated_boundary_model.is_inside_obstacle((j,i)):
                    self.map[i,j] = [0x7a, 0x70, 0x52]

                #Identify as stae belongs to the free space
                else:
                    self.map[i,j] = [0xc2, 0xba, 0xa3]

    def is_valid_position(self, position):
        """Checks if a given position belongs to free space or obstacle space

        Args:
            position (tuple): state of the agent to be verified

        Returns:
            bool: True if pixel belongs to free space else False
        """
        #Check if a state belongs to free psace by comparing the color of the respective pixel in the environment
        if  self.map[position[0], position[1]][0] == 0xc2 and \
            self.map[position[0], position[1]][1] == 0xba and \
            self.map[position[0], position[1]][2] == 0xa3:
            return True
        
        #return false of state is in obstacle space
        return False


    def refresh_map(self):
        """Refreshes map with updated image map
        """
        #Flip the map to satisfy the environment direction
        image = cv.flip(self.map.astype('uint8'), 0)
        cv.imshow("map", image)

    def update_map(self, explored_node):
        """Update map with explored nodes colour

        Args:
            explored_node (tuple): state that has been visited
        """
        i, j, theta = explored_node
        self.map[i, j] = [255, 255, 255]
        self.refresh_map()


    def save_image(self, file_path):
        """saves current state of the environment in the file location as image 

        Args:
            file_path (string): absolute path of the file where the image needs to be saved
        """
        #Flip the map to satisfy the environment direction
        image = cv.flip(self.map.astype('uint8'), 0)
        cv.imwrite(file_path, image)

    def highlight_state(self, position):
        """Draws a circle at the given location in the environment

        Args:
            position (tuple): pixel location
        """
        self.map =  cv.circle(self.map, (position[1],position[0]), 2, (255, 0, 0), 2)
        self.refresh_map()

    def highlight_point(self, position):
        """Highlights a point in the environment at the given locaiton

        Args:
            position (tuple): pixel location
        """
        i, j = position
        self.map[i, j] = [255, 0, 0]


    #primitives to save video of the jplanning environment-----------------------
    def begin_video_writer(self):
         self.writer= cv.VideoWriter('Animation Video.mp4', cv.VideoWriter_fourcc(*'DIVX'), 10, (self.width, self.height))
    
    def write_video_frame(self):
        image = cv.flip(self.map.astype('uint8'), 0)
        self.writer.write(image)

    def close_video_writer(self):
        self.writer.release()
    #--------------------------------------------------------------------------------
    

if __name__ == "__main__":
    _map_viz = environment(250, 600, 10)
    _map_viz.create_map()
    _map_viz.refresh_map()
    cv.waitKey(0)
