#!/usr/bin/env python3


# This assignment implements Dijkstra's shortest path on a graph, finding an unvisited node in a graph,

#   picking which one to visit, and taking a path in the map and generating waypoints along that path

#

# Given to you:

#   Priority queue

#   Image handling

#   Eight connected neighbors

#

# Slides https://docs.google.com/presentation/d/1XBPw2B2Bac-LcXH5kYN4hQLLLl_AMIgoowlrmPpTinA/edit?usp=sharing


# The ever-present numpy

import numpy as np

# Our priority queue
import heapq

# Using imageio to read in the image
import imageio

# -------------- Showing start and end and path ---------------

def plot_with_path(im, im_threshhold, zoom=1.0, robot_loc=None, goal_loc=None, path=None):

    """Show the map plus, optionally, the robot location and goal location and proposed path

    @param im - the image of the SLAM map

    @param im_threshhold - the image of the SLAM map

    @param zoom - how much to zoom into the map (value between 0 and 1)

    @param robot_loc - the location of the robot in pixel coordinates

    @param goal_loc - the location of the goal in pixel coordinates

    @param path - the proposed path in pixel coordinates"""


    # Putting this in here to avoid messing up ROS

    import matplotlib.pyplot as plt


    fig, axs = plt.subplots(1, 2)

    axs[0].imshow(im, origin='lower', cmap="gist_gray")

    axs[0].set_title("original image")

    axs[1].imshow(im_threshhold, origin='lower', cmap="gist_gray")

    axs[1].set_title("threshold image")

    """

    # Used to double check that the is_xxx routines work correctly

    for i in range(0, im_threshhold.shape[1]-1, 10):

        for j in range(0, im_threshhold.shape[0]-1, 10):

            if is_wall(im_thresh, (i, j)):

                axs[1].plot(i, j, '.b')

    """


    # Double checking lower left corner

    axs[1].plot(10, 5, 'xy', markersize=5)


    # Show original and thresholded image

    for i in range(0, 2):

        if robot_loc is not None:

            axs[i].plot(robot_loc[0], robot_loc[1], '+r', markersize=10)

        if goal_loc is not None:

            axs[i].plot(goal_loc[0], goal_loc[1], '*g', markersize=10)

        if path is not None:

            for p, q in zip(path[0:-1], path[1:]):

                axs[i].plot([p[0], q[0]], [p[1], q[1]], '-y', markersize=2)

                axs[i].plot(p[0], p[1], '.y', markersize=2)

        axs[i].axis('equal')


    for i in range(0, 2):

        # Implements a zoom - set zoom to 1.0 if no zoom
        width = im.shape[1]

        height = im.shape[0]

        axs[i].set_xlim(width / 2 - zoom * width / 2, width / 2 + zoom * width / 2)

        axs[i].set_ylim(height / 2 - zoom * height / 2, height / 2 + zoom * height / 2)


# -------------- Thresholded image True/False ---------------

def is_wall(im, pix):

    """ Is the pixel a wall pixel?

    @param im - the image

    @param pix - the pixel i,j"""

    if im[pix[1], pix[0]] == 0:

        return True
    return False


def is_unseen(im, pix):

    """ Is the pixel one we've seen?

    @param im - the image

    @param pix - the pixel i,j"""

    if im[pix[1], pix[0]] == 128:

        return True

    return False



def is_free(im, pix):

    """ Is the pixel empty?

    @param im - the image

    @param pix - the pixel i,j"""

    if im[pix[1], pix[0]] == 0:

        return True

    return False



def convert_image(im, wall_threshold, free_threshold):

    """ Convert the image to a thresholded image with not seen pixels marked

    @param im - WXHX ?? image (depends on input)

    @param wall_threshold - number between 0 and 1 to indicate wall

    @param free_threshold - number between 0 and 1 to indicate free space

    @return an image of the same WXH but with 0 (free) 255 (wall) 128 (unseen)"""


    # Assume all is unseen

    im_ret = np.zeros((im.shape[0], im.shape[1]), dtype='uint8') + 128


    im_avg = im

    if len(im.shape) == 3:

        # RGB image - convert to gray scale

        im_avg = np.mean(im, axis=2)

    # Force into 0,1

    im_avg = im_avg / np.max(im_avg)

    # threshold

    #   in our example image, black is walls, white is free

    im_ret[im_avg < wall_threshold] = 0

    im_ret[im_avg > free_threshold] = 255

    return im_ret



# -------------- Getting 4 or 8 neighbors ---------------

def four_connected(pix):

    """ Generator function for 4 neighbors

    @param im - the image

    @param pix - the i, j location to iterate around"""

    for i in [-1, 1]:

        ret = pix[0] + i, pix[1]

        yield ret

    for i in [-1, 1]:

        ret = pix[0], pix[1] + i

        yield ret



def eight_connected(pix):

    """ Generator function for 8 neighbors

    @param im - the image

    @param pix - the i, j location to iterate around"""

    for i in range(-1, 2):

        for j in range(-1, 2):

            if i == 0 and j == 0:

                pass

            ret = pix[0] + i, pix[1] + j

            yield ret


def dijkstra(im, robot_loc, goal_loc):

    if not is_free(im, robot_loc):
        raise ValueError(f"Start location {robot_loc} is not in the free space of the map")
    if not is_free(im, goal_loc):
        raise ValueError(f"Goal location {goal_loc} is not in the free space of the map")


    # qyeye to add nodes we will explore

    priority_queue = []

    heapq.heappush(priority_queue, (0, robot_loc))


    # set for visteed nodes

    visited = {}

    visited[robot_loc] = (0, None) 

    # const vals

    straightC = 1

    diagonalC = 1.4


    # iterate until nothing left in the queue (means we poped all the nodes)

    while priority_queue:

        current_cost, current_node = heapq.heappop(priority_queue)


        # done when we get to the goal

        if current_node == goal_loc:

            break


        for neighbor in eight_connected(current_node):

            x, y = neighbor

            if 0 <= x < im.shape[1] and 0 <= y < im.shape[0] and is_free(im, neighbor):

                move_cost = diagonalC if abs(current_node[0] - x) + abs(current_node[1] - y) == 2 else straightC

                newC = current_cost + move_cost

                if neighbor not in visited or newC < visited[neighbor][0]:

                    # add to visted so we dont come back

                    visited[neighbor] = (newC, current_node)

                    heapq.heappush(priority_queue, (newC, neighbor))

    # revuild the path and costs 

    path = []

    current = goal_loc

    while current is not None:

        path.append(current)

        current = visited[current][1] if current in visited else None


    return path[::-1]

def heuristic(a, b):
    # Manhattan distance
    return abs(a[0] - b[0]) + abs(a[1] - b[1])

def a_star(grid, start, goal):
    open_list = []  # Priority queue (min-heap)
    closed_list = set()  # Visited nodes

    start_node = Node(start)
    goal_node = Node(goal)

    heapq.heappush(open_list, start_node)

    while open_list:
        # Get the node with the lowest f score
        current_node = heapq.heappop(open_list)
        closed_list.add(current_node.position)

        # Check if we reached the goal
        if current_node.position == goal_node.position:
            path = []
            while current_node:
                path.append(current_node.position)
                current_node = current_node.parent
            return path[::-1]  # Return reversed path

        # Generate neighbors (up, down, left, right)
        neighbors = [
            (0, -1),  # Up
            (0, 1),   # Down
            (-1, 0),  # Left
            (1, 0)    # Right
        ]

        for offset in neighbors:
            neighbor_pos = (current_node.position[0] + offset[0],
                            current_node.position[1] + offset[1])

            # Check if the neighbor is within bounds and walkable
            if (0 <= neighbor_pos[0] < len(grid) and
                0 <= neighbor_pos[1] < len(grid[0]) and
                grid[neighbor_pos[0]][neighbor_pos[1]] == 0):

                if neighbor_pos in closed_list:
                    continue

                neighbor_node = Node(neighbor_pos, current_node)
                neighbor_node.g = current_node.g + 1
                neighbor_node.h = heuristic(neighbor_pos, goal_node.position)
                neighbor_node.f = neighbor_node.g + neighbor_node.h

                # Check if this path to the neighbor is better
                if any(neighbor.position == neighbor_node.position and neighbor.f <= neighbor_node.f for neighbor in open_list):
                    continue

                heapq.heappush(open_list, neighbor_node)

    return None  # No path found
    
def open_image(im_name):

    """ A helper function to open up the image and the yaml file and threshold

    @param im_name - name of image in Data directory

    @returns image anbd thresholded image"""


    # Needed for reading in map info

    from os import open
    
    im = imageio.imread("Data/" + im_name)


    wall_threshold = 0.7

    free_threshold = 0.9

    try:

        yaml_name = "Data/" + im_name[0:-3] + "yaml"

        with open(yaml_name, "r") as f:

            dict = yaml.load_all(f)

            wall_threshold = dict["occupied_thresh"]

            free_threshold = dict["free_thresh"]

    except:

        pass


    im_thresh = convert_image(im, wall_threshold, free_threshold)

    return im, im_thresh



if __name__ == '__main__':

    # Putting this here because in JN it's yaml
    import yaml_1 as yaml

    # Use one of these

    """ Values for SLAM map

    im, im_thresh = open_image("SLAM_map.png")

    robot_start_loc = (200, 150)

    # Closer one to try

    # robot_goal_loc = (315, 250)

    robot_goal_loc = (615, 850)

    zoom = 0.8

    """

    """ Values for map.pgm"""

    im, im_thresh = open_image("map.pgm")

    robot_start_loc = (1940, 1953)

    robot_goal_loc = (2135, 2045)

    zoom = 0.1


    """

    print(f"Image shape {im_thresh.shape}")

    for i in range(0, im_thresh.shape[1]-1):

        for j in range(0, im_thresh.shape[0]-1):

            if is_free(im_thresh, (i, j)):

                print(f"Free {i} {j}")

    """

    path = dijkstra(im_thresh, robot_start_loc, robot_goal_loc)
    plot_with_path(im, im_thresh, zoom=zoom, robot_loc=robot_start_loc, goal_loc=robot_goal_loc, path=path)

    # Depending on if your mac, windows, linux, and if interactive is true, you may need to call this to get the plt
    # windows to show
    # plt.show()

    print("Done")
