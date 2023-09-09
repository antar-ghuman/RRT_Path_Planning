# RRT_Path_Planning

This project implements the Rapidly-exploring Random Tree (RRT) algorithm for pathfinding in autonomous driving. The algorithm efficiently computes a collision-free path from a given starting point to a specified goal point on a map image, considering obstacles.

**Dependencies**
The project relies on the following Python libraries:

OpenCV (cv2): For image processing and visualization

NumPy: For numerical operations

Python version > 3.8

matplotlib


**How to Use**
To use this project, follow the steps below:

Prepare a map binary image where you want to find the path or use the one inside the maps directory. The map image should represent the environment with obstacles, and it should be in a common image format like PNG or JPEG and only contain pixel values as 255 (obstacle) and 0 (no-obstacle).

Run the main.py file with the required arguments in the following format:

```bash
python main.py start_x start_y goal_x goal_y map_image
```

start_x and start_y are the x and y coordinates of the starting point, respectively.
goal_x and goal_y are the x and y coordinates of the goal point, respectively.
map_image is the path to the map image file.
The algorithm will compute the path, avoiding obstacles, and draw it on the map image.

The resulting map image with the path will be saved in the home directory.

