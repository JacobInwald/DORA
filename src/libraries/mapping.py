import numpy as np
from matplotlib import pyplot as plt
import os 
from collections import deque


# ! Library Methods

def man_fuzz(grid: np.ndarray) -> np.ndarray:
    for i in range(grid.shape[0]):
        for j in range(grid.shape[1]):
            try:
                grid[i, j] = np.mean([grid[i, j], grid[i - 1, j], grid[i + 1, j], grid[i, j - 1], grid[i, j + 1]])
            except IndexError:
                pass
    return grid


def bresenham(start: np.ndarray, end: np.ndarray, res=1) -> np.ndarray[np.float64]:
    """
    Generate a Bresenham line between two points in a grid.

    Args:
        start (np.ndarray): The starting point of the line.
        end (np.ndarray): The ending point of the line.
        res (float, optional): The resolution of the grid. Defaults to 1.

    Returns:
        np.ndarray: An array of points representing the Bresenham line.
    """
    
    direction = end - start
    length = int(np.linalg.norm(direction) / res) + 1 
    result = np.zeros((length + 1, 2))

    for i in range(0, length + 1):
        result[i] = (start + (i / length) * direction).round(int(-np.log10(res)) if res < 1 else 0) // res
    
    return result


def floodfill(start: np.ndarray, occupancy_map: np.ndarray, res=1) -> np.ndarray[np.float64]:
    """
    Perform flood fill algorithm on an occupancy grid starting from a given point.

    Parameters:
    start (np.ndarray): The starting point for the flood fill algorithm.
    occupancy_grid (np.ndarray): The occupancy grid on which the flood fill algorithm is performed.
    res (int, optional): The resolution of the occupancy grid. Defaults to 1.

    Returns:
    np.ndarray[np.float64]: The result of the flood fill algorithm.
    """
    
    q = deque()
    q.append(start)
    while q:
        current = q.pop()
        occupancy_map[current[0], current[1]] = 0
        for (i, j) in [(1, 0), (-1, 0), (0, 1), (0, -1)]:
                try:
                    if occupancy_map[current[0] + i, current[1] + j] == 0.5:
                        q.append(np.array([current[0] + i, current[1] + j]))
                except IndexError:
                    pass

    return occupancy_map


def init_occ_map(ox, oy, xy_resolution=0.02, EXTEND_AREA=1.0):
    min_x = round(min(ox) - EXTEND_AREA / 2.0)
    min_y = round(min(oy) - EXTEND_AREA / 2.0)
    max_x = round(max(ox) + EXTEND_AREA / 2.0)
    max_y = round(max(oy) + EXTEND_AREA / 2.0)
    x_w = int(round((max_x - min_x) / xy_resolution))
    y_w = int(round((max_y - min_y) / xy_resolution))
    occupancy_map = np.ones((x_w + 1, y_w + 1)) * 0.5
    center_x = int(round(-min_x / xy_resolution))  # center x coordinate of the grid map
    center_y = int(round(-min_y / xy_resolution))  # center y coordinate of the grid map
    return occupancy_map, min_x, max_x, min_y, max_y, x_w, y_w, center_x, center_y


def occupancy_grid(ox, oy, xy_resolution=0.02, EXTEND_AREA=1.0, fuzz=False):
    """
    Generates an occupancy map based on the given obstacle coordinates.

    Parameters:
    - ox (list): List of x-coordinates of the obstacles.
    - oy (list): List of y-coordinates of the obstacles.
    - xy_resolution (float): Resolution of the occupancy map in the x and y directions. Default is 0.02.
    - EXTEND_AREA (float): Extension factor for the occupied area. Default is 1.0.

    Returns:
    - occupancy_map (numpy.ndarray): Occupancy map representing the environment, where 0.0 represents free area and 1.0 represents occupied area.
    """
    
    occupancy_map, min_x, max_x, min_y, max_y, x_w, y_w, center_x, center_y = \
        init_occ_map(ox, oy, xy_resolution, EXTEND_AREA)
    prev_ix, prev_iy = center_x - 1, center_y
    
    # Find the bounding box of the occupied area
    for (x, y) in zip(ox, oy):
        # x, y coordinates of the the occupied area
        ix = int(round((x - min_x) / xy_resolution))
        iy = int(round((y - min_y) / xy_resolution))
        
        free_area = bresenham(np.array([prev_ix, prev_iy]), np.array([ix, iy]))

        for fa in free_area:
            occupancy_map[int(fa[0])][int(fa[1])] = 0  # free area 0.0
        
        prev_ix = ix
        prev_iy = iy
    
    # Flood Fill
    occupancy_map = floodfill(np.array([center_x, center_y]), occupancy_map)
    occupancy_map = man_fuzz(occupancy_map) if fuzz else occupancy_map
    # Draw on Obstacles
    for (x, y) in zip(ox, oy):
            ix = int(round((x - min_x) / xy_resolution))
            iy = int(round((y - min_y) / xy_resolution))
            occupancy_map[ix][iy] = 1.0  # occupied area 1.0
            # occupancy_map[ix + 1][iy] = 1.0  # extend the occupied area
            # occupancy_map[ix][iy + 1] = 1.0  # extend the occupied area
            # occupancy_map[ix + 1][iy + 1] = 1.0  # extend the occupied area
    
    occupancy_map = man_fuzz(occupancy_map) if fuzz else occupancy_map
    
    return occupancy_map

# ! Simulation Methods


def isBetween(a, b, c):
    return np.linalg.norm(a - c) + np.linalg.norm(b - c) == np.linalg.norm(a - b)


def pointIntersect(p1, d1, p2, d2):
    
    if d2[0] == 0:
        t = (p2[0] - p1[0]) / d1[0]
    elif d2[1] == 0:
        t = (p2[1] - p1[1]) / d1[1]
    else:
        t = ((p2[1] - p1[1]) + (d2[1]/d2[0])*(p1[0] - p2[0])) / (d1[1] - (d2[1]*d1[0]) / d2[0])
    p = p1 + t * d1
    
    if any(np.isnan(p)) or any(np.isinf(p)):
        print("Error: pointIntersect()")
        print(p1, d1, p2, d2)
        return np.array([np.nan, np.nan])

    return p

def lidarRay(start, angle, bregion: np.ndarray):
    dir_lidar = np.array([np.sin(angle), np.cos(angle)])
    dir_lidar = np.round(dir_lidar, 3)
    min_dist = np.inf
    for i in range(len(bregion)):
        p1 = bregion[i]
        dir = bregion[i+1] - p1 if i < len(bregion) - 1 else p1 - bregion[0]
        dir = np.round(dir, 3)
        p = pointIntersect(start, dir_lidar, p1, dir)
        
        if np.isnan(p).any() or  \
            np.dot(p - start, dir_lidar) < 0 or \
            not isBetween(p1, p1+dir, p) or \
            np.linalg.norm(p - start) > min_dist:
            continue
        
        min_dist = np.linalg.norm(p - start)
        
    return min_dist if min_dist != np.inf else np.nan


def lidarCircleScan(start: np.ndarray, bregion: np.ndarray, res: int =1):
    scan = []
    angles = [np.deg2rad(i) for i in np.arange(0, 360, res)]
    for angle in [a for a in angles]:
        p = np.array([angle, lidarRay(start, angle, bregion)])
        if np.isnan(p).any():
            continue
        scan.append(p)
    return np.array(scan)



region = np.array([[-2, 4], [3, 4], [2,2], [4, 3], [4, 0], [4, 0], [2, -1], [-2, 0]])
region2 = np.array([[-2, 4], [3, 4], [2,2], [4, 3], [4, 0], [4, 0], [2, -1], [-2, 0], [-2, 4]])

scan = lidarCircleScan(np.array([0, 0]), region, res= 1)
ox = scan[:,1] * np.sin(scan[:,0])
oy = scan[:,1] * np.cos(scan[:,0])

plt.figure(1, figsize=(10, 10))
plt.plot(region2[:,0], region2[:,1])

plt.plot([0], [0], 'bo')

plt.plot(ox, oy, 'ro')
plt.show()
xyresolution = 0.1
occupancy_map = occupancy_grid(ox, oy, xy_resolution=xyresolution)
xy_res = np.array(occupancy_map).shape

plt.figure(1, figsize=(10, 4))
plt.subplot(122)
plt.imshow(occupancy_map, cmap="PiYG_r")
# cmap = "binary" "PiYG_r" "PiYG_r" "bone" "bone_r" "RdYlGn_r"
plt.clim(-0.4, 1.4)
plt.gca().set_xticks(np.arange(-.5, xy_res[1], 1), minor=True)
plt.gca().set_yticks(np.arange(-.5, xy_res[0], 1), minor=True)
plt.grid(True, which="minor", color="w", linewidth=0.6, alpha=0.5)
plt.colorbar()
plt.subplot(121)
plt.plot([oy, np.zeros(np.size(oy))], [ox, np.zeros(np.size(oy))], "ro-")
plt.axis("equal")
plt.plot(0.0, 0.0, "ob")
plt.gca().set_aspect("equal", "box")
bottom, top = plt.ylim()  # return the current y-lim
plt.ylim((top, bottom))  # rescale y axis, to match the grid orientation
plt.grid(True)
plt.show()


# ! Test Code



# if __name__ == "__main__":

#     with open('data/lidar01.csv') as f:
#         data = f.readlines()
#         data = [line.strip() for line in data]
#         data = [line.split(',') for line in data]
#         data = np.array(data).astype(np.double)
#         angles = data[:, 0]
#         distances = data[:, 1]
        

#     ox = distances * np.sin(angles)
#     oy = distances * np.cos(angles)
