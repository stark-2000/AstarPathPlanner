# Implementation-of-A*-Path-Planning-Algorithm

## Team Members:
- Tej (119197066 - itej89)
- Arshad Shaik (118438832 - arshad22)
- Dhinesh Rajasekaran (119400241 - dhinesh)

## Google Drive Link:
- Below link has the Animation Video output:
    - https://drive.google.com/file/d/1Xx3JgBlFZ-htePhxba8Yi7xv2LvUGVkv/view

## Github Link:
- https://github.com/stark-2000/AstarPathPlanner

## Instructions:
- Clone the repository to your local machine using the following command:
    ``` 
    git clone https://github.com/stark-2000/AstarPathPlanner.git
    ```
    cd into the cloned repository
    ```
    cd AstarPathPlanner
    ```

- Alternatively, you can download the zip file of the repository and extract it to your local machine:
    ```
    cd AstarPathPlanner-master
    ```

- Open Terminal and Run the following commands & test with the following test cases: (make sure visulization.py file is in the same folder)
    ```
    python3 a_star_arshad_dhinesh_tej.py
    ```
- Now for Test Case 1, Enter the following values when prompted or any random values of your choice:
    - Enter Radius of the robot as "5"
    - Enter the clearance as "5"
    - Enter step size for robot movement as "5"
    - Enter the start node as "30,30,30"
    - Enter the goal node as "418,120,30"

- Repeat the above step for Test Case 2 with the following values or any random values of your choice:
    - Enter Radius of the robot as "5"
    - Enter the clearance as "5"
    - Enter step size for robot movement as "10"
    - Enter the start node as "30,30,30"
    - Enter the goal node as "120,120,30"

- Note: 
    - To find the optimal path for goal node placed on the right side of hexagon, it takes approx 6 to 7mins depending on your cpu speed. So, please be patient.
    - The threshold used for node closeness comparison is 2.5 units (0.5 * radius of robot), so please provide the step size above 2.5 units for finding the path. It changes dynamicallyy based on the radius of the robot provided by user. Make sure the step size is above 0.5 * radius of the robot.


## Obstacle Space - Map:
- Adobe Photoshop generated:  

![Obstacle_Space_Map](https://user-images.githubusercontent.com/112987383/230250741-1ed85db1-178c-4c36-87fe-f6015a5a2674.png)

- Desmos Visualization:
   
![Canvas_Desmos](https://user-images.githubusercontent.com/78305300/226239180-21341f99-308c-4c84-a3f6-ec63ddbf447d.png)

## Demo Video:
 - Video shows exploration of nodes and finding the goal node for a circular robot given start node, goal node, robot radius and clerance from obstacles. Once the shortest path is found, it is backtracked and visualized using openCV.
   
https://user-images.githubusercontent.com/78305300/226238687-4ec5a996-c4e9-45fa-b35a-5d853e70b8a6.mp4


## Dependencies:
 - Other py file (map creation, visualization script, action generation, node checking)
    - For these various operations, multiple py scripts are created and included in the same repository.
    - Make sure you are running a_star_tej_arshad_dhinesh.py script from the same folder.

 - Pyclipper Dependency:
    - Pyclipper is a python wrapper for the C++ library clipper. It is used to perform boolean operations on polygons. It gives us the inflated polygon vertices provided the original vertices of the polygon and the inflation radius.
    - Pyclipper works in latest verisons of python in Linux. But for some reason, in windows, you need python 3.7 to run pyclipper. So, if you are using windows, make sure you have python 3.7 installed.
    - Link: https://pypi.org/project/pyclipper/

 - All the installation instructions work both for Linux and Windows. For windows, make sure you have python interpreter installed. If not, download and install python 3.7 from the following link:
    - https://www.python.org/downloads/release/python-370/

    - Python 3.7 Installation for Linux:
        - (if req open terminal and run the following commands)
        ``` 
        sudo apt install python3.7
        ```

 - pyclipper library
    - (if req open terminal and run the following commands)
    ``` 
    sudo pip install pyclipper
    ```

 - Numpy library
    - (if req open terminal and run the following commands)
    ```
    sudo pip install numpy
    ```

- openCV library
    - (if req open terminal and run the following commands)
    ```
    sudo pip install opencv-contrib-python
    ```

- time library
    - (if req open terminal and run the following commands)
    ```
    sudo pip install time
    ```

- heapq library
    - (if req open terminal and run the following commands)
    ```
    sudo pip install heapq
    ```

- math library
    - (if req open terminal and run the following commands)
    ```
    sudo pip install math
    ```

- matplotlib library
    - (if req open terminal and run the following commands)
    ```
    sudo pip install matplotlib
    ```
