#!/usr/bin/env python3

# This assignment lets you both define a strategy for picking the next point to explore and determine how you
#  want to chop up a full path into way points. You'll need path_planning.py as well (for calculating the paths)
#
# Note that there isn't a "right" answer for either of these. This is (mostly) a light-weight way to check
#  your code for obvious problems before trying it in ROS. It's set up to make it easy to download a map and
#  try some robot starting/ending points
#
# Given to you:
#   Image handling
#   plotting
#   Some structure for keeping/changing waypoints and converting to/from the map to the robot's coordinate space
#
# Slides

# The ever-present numpy
import numpy as np

# Your path planning code
import path_planning as pp
# Our priority queue
import heapq

# Using imageio to read in the image
import imageio


# -------------- Showing start and end and path ---------------
def plot_with_explore_points(im_threshhold, zoom=1.0, robot_loc=None, explore_points=None, best_pt=None):
    """Show the map plus, optionally, the robot location and points marked as ones to explore/use as end-points
    @param im - the image of the SLAM map
    @param im_threshhold - the image of the SLAM map
    @param robot_loc - the location of the robot in pixel coordinates
    @param best_pt - The best explore point (tuple, i,j)
    @param explore_points - the proposed places to explore, as a list"""

    # Putting this in here to avoid messing up ROS
    import matplotlib.pyplot as plt

    fig, axs = plt.subplots(1, 2)
    axs[0].imshow(im_threshhold, origin='lower', cmap="gist_gray")
    axs[0].set_title("original image")
    axs[1].imshow(im_threshhold, origin='lower', cmap="gist_gray")
    axs[1].set_title("threshold image")
    """
    # Used to double check that the is_xxx routines work correctly
    for i in range(0, im_threshhold.shape[1]-1, 10):
        for j in range(0, im_threshhold.shape[0]-1, 2):
            if is_reachable(im_thresh, (i, j)):
                axs[1].plot(i, j, '.b')
    """

    # Show original and thresholded image
    if explore_points is not None:
        for p in explore_points:
            axs[1].plot(p[0], p[1], '.b', markersize=2)

    for i in range(0, 2):
        if robot_loc is not None:
            axs[i].plot(robot_loc[0], robot_loc[1], '+r', markersize=10)
        if best_pt is not None:
            axs[i].plot(best_pt[0], best_pt[1], '*y', markersize=10)
        axs[i].axis('equal')

    for i in range(0, 2):
        # Implements a zoom - set zoom to 1.0 if no zoom
        width = im_threshhold.shape[1]
        height = im_threshhold.shape[0]

        axs[i].set_xlim(width / 2 - zoom * width / 2, width / 2 + zoom * width / 2)
        axs[i].set_ylim(height / 2 - zoom * height / 2, height / 2 + zoom * height / 2)


# -------------- For converting to the map and back ---------------
def convert_pix_to_x_y(im_size, pix, size_pix):
    """Convert a pixel location [0..W-1, 0..H-1] to a map location (see slides)
    Note: Checks if pix is valid (in map)
    @param im_size - width, height of image
    @param pix - tuple with i, j in [0..W-1, 0..H-1]
    @param size_pix - size of pixel in meters
    @return x,y """
    if not (0 <= pix[0] <= im_size[1]) or not (0 <= pix[1] <= im_size[0]):
        raise ValueError(f"Pixel {pix} not in image, image size {im_size}")

    return [size_pix * pix[i] / im_size[i] for i in range(0, 2)]


def convert_x_y_to_pix(im_size, x_y, size_pix):
    """Convert a map location to a pixel location [0..W-1, 0..H-1] in the image/map
    Note: Checks if x_y is valid (in map)
    @param im_size - width, height of image
    @param x_y - tuple with x,y in meters
    @param size_pix - size of pixel in meters
    @return i, j (integers) """
    pix = [int(x_y[i] * im_size[1-i] / size_pix) for i in range(0, 2)]

    if not (0 <= pix[0] <= im_size[1]) or not (0 <= pix[1] <= im_size[0]):
        raise ValueError(f"Loc {x_y} not in image, image size {im_size}")
    return pix

def is_reachable(im, pix): #rewrote this so that it checks if the point is next to an unseen point
    """ Is the pixel reachable, i.e., has a neighbor that is free?
    Used for
    @param im - the image
    @param pix - the pixel i,j"""

    # Returns True (the pixel is adjacent to a pixel that is free)
    #  False otherwise
    # You can use four or eight connected - eight will return more points
    # YOUR CODE HERE

    # Define 8-connected neighbors (include diagonals)
    neighbors = [
        (-1, 0), (1, 0), (0, -1), (0, 1),  # 4-connected (up, down, left, right)
    ]

    # Get image dimensions
    width, height = im.shape

    # Check each neighbor
    for dx, dy in neighbors:
        neighbor_x = pix[0] + dx
        neighbor_y = pix[1] + dy

        # Ensure is within image bounds
        if 0 <= neighbor_x < width and 0 <= neighbor_y < height:
            # Check if the neighboring pixel is a wall (value 0)
            # if pp.is_wall(im, [neighbor_x, neighbor_y]):
            #     return False
            # Check if the neighboring pixel is unseen (value 128)
            if pp.is_unseen(im, [neighbor_x, neighbor_y]):
                return True

    # No free neighbors found
    return False


def find_all_possible_goals(im):
    pixels = im
    # Define labels
    FREE = 0
    UNKNOWN = -1

    # Get dimensions of the array
    height, width = pixels.shape

    # Initialize a boolean array for the result
    result = np.zeros_like(pixels, dtype=bool)

    # Compare with neighbors using slicing
    if height > 1:
        result[1:, :] |= (pixels[1:, :] == FREE) & (pixels[:-1, :] == UNKNOWN)  # Above neighbor
        result[:-1, :] |= (pixels[:-1, :] == FREE) & (pixels[1:, :] == UNKNOWN)  # Below neighbor
    if width > 1:
        result[:, 1:] |= (pixels[:, 1:] == FREE) & (pixels[:, :-1] == UNKNOWN)  # Left neighbor
        result[:, :-1] |= (pixels[:, :-1] == FREE) & (pixels[:, 1:] == UNKNOWN)  # Right neighbor

    # Get the coordinates of matching pixels
    coordinates = np.argwhere(result)

    # Return as a single list
    return [(x, y) for y, x in coordinates]

def find_best_point(im, possible_points, robot_loc, previous_points, radius = 40):
    """ Pick one of the unseen points to go to
    @param im - thresholded image
    @param possible_points - possible points to chose from
    @param robot_loc - location of the robot (in case you want to factor that in)
    """
    # YOUR CODE HERE

    if not possible_points:
        # No possible points to explore
        return None

    # Initialize the best point and the minimum distance
    best_point = None
    min_distance = 0.0
    # Iterate through all possible points

    def distance(point1, point2):
        return np.sqrt((point1[0] - point2[0])**2 + (point1[1] - point2[1])**2)
    
    filtered_locations = []
    
    for loc in possible_points:
        if all(distance(loc, goal) >= radius for goal in previous_points):
            filtered_locations.append(loc)


    for point in filtered_locations:
        # Calculate Euclidean distance from robot location
        Currrent_distance = distance(robot_loc, point)

        # Update the best point if a closer point is found
        if Currrent_distance > min_distance:
            best_point = point
            min_distance = Currrent_distance 

    return best_point

def find_waypoints(im, path):
    """ Place waypoints along the path
    @param im - the thresholded image
    @param path - the initial path
    @ return - a new path"""

    # Again, no right answer here
    # YOUR CODE HERE
    # Parameters for waypoint selection
    max_distance = 10  # Maximum distance between waypoints
    waypoints = []

    # Add the first point as the starting waypoint
    if not path:
        return waypoints  # Return an empty list if no path is given
    waypoints.append(path[0])

    # Iterate through the path to place waypoints
    for i in range(1, len(path)):
        # Calculate the Euclidean distance from the last waypoint
        last_waypoint = waypoints[-1]
        current_point = path[i]
        distance = ((current_point[0] - last_waypoint[0])**2 + (current_point[1] - last_waypoint[1])**2)**0.5

        # Place a waypoint if the distance exceeds the threshold
        if distance >= max_distance or i == len(path) - 1:
            waypoints.append(current_point)

    return waypoints

if __name__ == '__main__':
    # Doing this here because it is a different yaml than JN
    import yaml_1 as yaml

    im, im_thresh = path_planning.open_image("map.pgm")

    robot_start_loc = (1940, 1953)

    all_unseen = find_all_possible_goals(im_thresh)
    best_unseen = find_best_point(im_thresh, all_unseen, robot_loc=robot_start_loc)

    plot_with_explore_points(im_thresh, zoom=0.1, robot_loc=robot_start_loc, explore_points=all_unseen, best_pt=best_unseen)

    path = path_planning.dijkstra(im_thresh, robot_start_loc, best_unseen)
    waypoints = find_waypoints(im_thresh, path)
    path_planning.plot_with_path(im, im_thresh, zoom=0.1, robot_loc=robot_start_loc, goal_loc=best_unseen, path=waypoints)

    # Depending on if your mac, windows, linux, and if interactive is true, you may need to call this to get the plt
    # windows to show
    # plt.show()

    print("Done")
