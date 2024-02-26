import numpy as np
from .point_cloud import PointCloud
from .occupancy_map import OccupancyMap
from dora_msgs.msg import Toy
from queue import PriorityQueue


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
        self.unload_points = None

    def route(self, start, end, map: OccupancyMap) -> np.ndarray:
        """
        This function calculates the route from the current position to the end point.

        Parameters:
            end (np.ndarray): The end point coordinates on the map.
            map (OccupancyMap): The occupancy map object representing the environment.

        Returns:
            np.ndarray: The calculated route as an array of coordinates.
        """
        move_dist = map.resolution
        directions = np.array([(x, y) for x in [-1, 0, 1] for y in [-1, 0, 1]
                               if (x, y) != (0, 0)])

        g = {}
        g[str(start)] = 0
        alpha = 0.5 # the penalty parameter for being close to obstacles
        f = lambda p,a: 3* np.linalg.norm(p - end) + g[str(p)] + a * alpha
        q = PriorityQueue()
        q.put((f(start,0), start))
        parent = {}
        found = False

        # A* Algorithm
        while q.empty() is False:
            _, cur = q.get()
            cur = np.array(cur)
            strCur = str(cur)

            if np.linalg.norm(cur - end) < move_dist:
                found = True
                break

            for d in directions:
                n = cur + d * move_dist
                strNext = str(n)
            
                
                if map.sampleCoord(n) > 0.5:
                    g[strNext] = g[strCur] + np.inf
                    continue
                
                # penalize for obstacles in 3 move_dist
                penalty = map.sampleCoord(n, mean=True)
                
                try:
                    if g[strCur] + self.controller.move_dist < g[strNext]:
                        g[strNext] = g[strCur] + move_dist
                        parent[strNext] = cur
                except KeyError:
                    g[strNext] = g[strCur] + move_dist
                    q.put((f(n,penalty), tuple(n)))
                    parent[strNext] = cur

        if found:
            route = [cur]
            cur = str(cur)
            while cur in parent:
                route.append(parent[cur])
                cur = str(parent[cur])
            route.reverse()
            return np.array(route)

        return [start]

    def next_mapping_pt(self, map: OccupancyMap) -> np.ndarray:
        """
        Finds the next mapping point in the occupancy map.

        Parameters:
        - map: OccupancyMap object representing the map.

        Returns:
        - numpy array representing the next mapping point.
        """

        map.normalise()
        clouds = map.pointclouds

        # BFS
        for cloud in clouds:
            for p in cloud.emptyCloud:
                d = ((p - cloud.origin) / np.linalg.norm(p - cloud.origin) *
                     map.resolution)
                if (any(np.linalg.norm(p - c.origin) < 0.75 for c in clouds)
                        or map.sampleCoord(p + d, yx=True, mean=True) <= 1e-3
                        or map.sampleCoord(p + d, yx=True, mean=True) >= 0.55):
                    continue
                return p - d

        return clouds[-1].origin

    def next_retrieve_pt(self, map: OccupancyMap, toys: [Toy], cur: np.ndarray) -> (np.ndarray, Toy):
        """
        Finds the next point to retrieve toy in the occupancy map. (nearest toy from start position)

        Parameters:
        - map: OccupancyMap object representing the map.
        - toys: A list of Toy objects.
        - cur: Current coordinate of np.array([x,y]).

        Returns:
        - A tuple of coordinate and the Toy object.
        
        """
        closest_point = None
        closest_toy = None
        shortest_dist = None
        for toy in toys:
            this_dist = np.linalg.norm([toy.position.x, toy.position.y], cur)
            if closest_point == None:
                shortest_dist = this_dist
                closest_point = np.array([toy.position.x, toy.position.y])
                closest_toy = toy
            if this_dist < shortest_dist:
                shortest_dist = this_dist
                closest_point = np.array([toy.position.x, toy.position.y])
                closest_toy = toy
                
        return (closest_point, closest_toy)

    def next_unload_pt(self, map: OccupancyMap, toy: Toy, cur: np.ndarray) -> np.ndarray:
        """
        Finds the next point to unload toy in the occupancy map. (Nearest unload point from start position)

        Parameters:
        - map: OccupancyMap object representing the map.
        - toy: The toy to unload.
        - cur: Current coordinate of np.array([x,y]).

        Returns:
        - The coordinate of the unload point
        
        """
        
        closest_unload_point = None
        shortest_dist = None
        
        for unload_point in self.unload_points:
            this_dist = np.linalg.norm([unload_point.x, unload_point.y], cur)
            if closest_unload_point == None:
                shortest_dist = this_dist
                closest_unload_point = unload_point
            if this_dist < shortest_dist:
                shortest_dist = this_dist
                closest_unload_point = unload_point
                
        return closest_unload_point
            
