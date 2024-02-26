import numpy as np
from point_cloud import PointCloud
from occupancy_map import OccupancyMap
from dora_msgs.msg import Toy


def man_fuzz(grid: np.ndarray) -> np.ndarray:
    """
    Applies the Manhattan Fuzz algorithm to the given grid.

    The Manhattan Fuzz algorithm calculates the mean value of each cell in the grid
    by considering its neighboring cells in the Manhattan distance.

    Params:
        grid (np.ndarray): The input grid.

    Returns:
        np.ndarray: The grid with the Manhattan Fuzz algorithm applied.
    """
    for i in range(grid.shape[0]):
        for j in range(grid.shape[1]):
            if grid[i, j] >= 0.95 or grid[i, j] == 0:
                continue
            try:
                grid[i, j] = np.mean([
                    grid[i + x, j + y] for x in range(-1, 2)
                    for y in range(-1, 2)
                ])
            except IndexError:
                pass
    return grid


def bresenham(start: np.ndarray,
              end: np.ndarray,
              res: float = 1) -> np.ndarray[np.float64]:
    """
    Generate a Bresenham line between two points in a grid.

    Params:
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
        result[i] = (start + (i / length) * direction
                     ).round(int(-np.log10(res)) if res < 1 else 0) // res

    return result



class Router:
    """
    The Router class is responsible for calculating routes.
    (the original Router class but without the navigation part)

    TODO: Implement class (Keming)
    """

    def __init__(self):
        pass

    def route(self, start, end, map: OccupancyMap) -> np.ndarray:
        """
        This function calculates the route from the current position to the end point.

        Parameters:
            end (np.ndarray): The end point coordinates on the map.
            map (OccupancyMap): The occupancy map object representing the environment.

        Returns:
            np.ndarray: The calculated route as an array of coordinates.
        """
        pass

    def next_mapping_pt(self, map: OccupancyMap) -> np.ndarray:
        pass

    def next_retrieve_pt(self, map: OccupancyMap, toys: [Toy]) -> (np.ndarray, Toy):
        pass

    def next_unload_pt(self, map: OccupancyMap, toy: Toy) -> np.ndarray:
        pass
