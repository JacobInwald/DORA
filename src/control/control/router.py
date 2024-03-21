import numpy as np
from point_cloud import PointCloud
from occupancy_map import OccupancyMap
# from dora_msgs.msg import Toy, Toys
from utils import bresenham

from queue import PriorityQueue

class Router:
    """
    The Router class is responsible for calculating routes.
    (the original Router class but without the navigation part)

    Parameters:
        unload_points: Key-value pair to store coodinate and type of the unload points.
                       In form of (x, y): toy class

    TODO: Implement class (Keming)
    """

    def __init__(self):
        self.unload_points = {}

    def route(self, start, end, map: OccupancyMap) -> np.ndarray:
        """
        This function calculates the route from the current position to the end point.

        Parameters:
            end (np.ndarray): The end point coordinates on the map.
            map (OccupancyMap): The occupancy map object representing the environment.

        Returns:
            np.ndarray: The calculated route as an array of coordinates.
        """
        move_dist = 0.05
        map.change_res(move_dist)
        directions = np.array([(x, y) for x in [-1, 0, 1] for y in [-1, 0, 1]
                               if (x, y) != (0, 0)])
        directions = np.array([(1, 0), (0,1), (-1,0), (0,-1)])
        g = {}
        g[str(start)] = 0
        alpha = 2 # the penalty parameter for being close to obstacles
        def f(p, a): return 3 * np.linalg.norm(p - end) + g[str(p)] + a * alpha
        q = PriorityQueue()
        q.put((f(start, 0), (start, 0)))
        parent = {}
        found = False

        # A* Algorithm
        while q.empty() is False:
            _, (cur, dir) = q.get()
            cur = np.array(cur)
            strCur = str(cur)

            if np.linalg.norm(cur - end) < move_dist:
                found = True
                break
            n_d = 0
            for d in directions:
                n_d += 1
                n = cur + d * move_dist
                
                strNext = str(n)

                if map.sample_coord(n) > 0.5:
                    g[strNext] = g[strCur] + np.inf
                    continue
                

                # penalize for obstacles in 3 move_dist
                
                penalty = map.sample_coord(n, mean=True, n=3)
                if penalty != 1:
                    penalty = 0
                penalty *= 1000
                penalty += int(n_d != dir)
                

                try:
                    if g[strCur] + move_dist < g[strNext]:
                        g[strNext] = g[strCur] + move_dist
                        parent[strNext] = cur
                except KeyError:
                    g[strNext] = g[strCur] + move_dist
                    q.put((f(n, penalty), (tuple(n), n_d)))
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
                        or map.sample_coord(p + d, yx=True, mean=True) <= 1e-3
                        or map.sample_coord(p + d, yx=True, mean=True) >= 0.55):
                    continue
                return p - d

        return clouds[-1].origin

    # def next_retrieve_pt(self, map: OccupancyMap, toys: Toys, cur: np.ndarray) -> tuple[np.ndarray, Toy]:
    #     """
    #     Finds the next point to retrieve toy in the occupancy map. (nearest toy from start position)

    #     Parameters:
    #     - map: OccupancyMap object representing the map.
    #     - toys: A list of Toy objects.
    #     - cur: The start point of DORA.

    #     Returns:
    #     - A tuple of coordinate and the Toy object.

    #     """
    #     closest_point = None
    #     closest_toy = None
    #     shortest_dist = None
    #     for toy in toys:
    #         this_dist = np.linalg.norm([toy.position.x, toy.position.y], cur)
    #         if closest_point == None:
    #             shortest_dist = this_dist
    #             closest_point = np.array([toy.position.x, toy.position.y])
    #             closest_toy = toy
    #         if this_dist < shortest_dist:
    #             shortest_dist = this_dist
    #             closest_point = np.array([toy.position.x, toy.position.y])
    #             closest_toy = toy

    #     return (closest_point, closest_toy)

    # def next_unload_pt(self, map: OccupancyMap, toy: Toy) -> np.ndarray:
    #     """
    #     Finds the closest unload point to the toy in the occupancy map. (Nearest unload point from start position)

    #     Parameters:
    #     - map: OccupancyMap object representing the map.
    #     - toy: The toy to unload.

    #     Returns:
    #     - The coordinate of the unload point

    #     """

    #     closest_unload_point = None
    #     shortest_dist = None

    #     for _, (unload_point, unload_point_class) in enumerate(self.unload_points):
    #         if unload_point_class != Toy.cls:
    #             continue
    #         this_dist = np.linalg.norm([unload_point[0], unload_point[1]], [
    #                                    Toy.position.x, Toy.position.y])
    #         if closest_unload_point == None:
    #             shortest_dist = this_dist
    #             closest_unload_point = unload_point
    #         if this_dist < shortest_dist:
    #             shortest_dist = this_dist
    #             closest_unload_point = unload_point

    #     return closest_unload_point
    
    
def test():
    import matplotlib.pyplot as plt
    occ = OccupancyMap.load('/Users/linkeming/Documents/GitHub/DORA/data/maps/reference_map.npz')
    occ.fuzz_map(5)
    router = Router()
    route = router.route(np.array([1, -0.5]), np.array([1.5, -1.5]), occ)
    print(route)
    for pt in route:
        pt = occ.translate(pt)
        occ.map[pt[1], pt[0]] = 2
    plt.imshow(occ.map)
    plt.show()
    
    critical_route = get_turning_points(route)
        
    for pt in critical_route:
        pt = occ.translate(pt)
        occ.map[pt[1], pt[0]] = 5
    print('ccccc', critical_route)
    
    plt.imshow(occ.map)
    plt.show()
    
    for pt in route:
        pt = occ.translate(pt)
        occ.map[pt[1], pt[0]] = 0.1
    
    pts = reduce_turning_points(critical_route, occ)
    print(pts)
    for pt in pts:
        pt = occ.translate(pt)
        occ.map[pt[1], pt[0]] = 1
    plt.imshow(occ.map)
    plt.show()
    
def reduce_turning_points(turning_points, map):
    """
        Reduce number of turning points if there is no objects in between.
        This is for making less turns for navigation.

        Parameters:
        turning_points (np.ndarray): The calculated route as an array of coordinates.
        map (OccupancyMap): The reference map with detected obstacles.
        
        Returns:
        - numpy array representing remaining turning points.
    """
    i = 0
    remain_pts = []
    remain_pts.append(turning_points[0])
    print(turning_points)
    while True:
        for j in range(i + 1, len(turning_points)):
            if line_close_to_obs(turning_points[i], turning_points[j], map):
                print('bad line')
                print('i:' ,i)
                print('j:' ,j)
                i = j - 1
                remain_pts.append(turning_points[i])
                break
            else:
                print('good line')
                print('i:' ,i)
                print('j:' ,j)
                if j == len(turning_points) - 1:
                    i = j
                    break
            if i == len(turning_points) - 1:
                remain_pts.append(turning_points[i])
                return remain_pts
        if i == len(turning_points) - 1:
            remain_pts.append(turning_points[i])
            break
    remain_pts.append(turning_points[-1])
    return remain_pts
        
def get_turning_points(route) -> np.ndarray:
    """
        Finds all the turning points from the route from A* algorithm.

        Parameters:
        route (np.ndarray): The calculated route as an array of coordinates.

        Returns:
        - numpy array representing coordinates of turning points.
    """
    result = [x - y for x, y in zip(route[1:-1], route[0:-2])]
    turning_points = []
    for i in range(1, len(result), 1):
        if not (np.all(result[i] == result[i-1])):
            turning_points.append(route[i])
    turning_points.insert(0, route[0])
    turning_points.append(route[-1])
    return np.array(turning_points)
            
def line_close_to_obs(p1,p2,map: OccupancyMap):
    """
        Detect if any points on the line created by two coordinates are too 
        close to obstacles.

        Parameters:
        p1 (np.array): first coordinate.
        p2 (np.array): second coordinate.
        map (OccupancyMap): The reference map with detected obstacles.

        Returns:
        - Boolean indicating if there is obstacle nearby (True means there is).
    """
    line = bresenham(p1,p2, 0.01)
    line = line * 0.01
    for pt in line:
        if map.sample_coord(pt, mean=True, n=3) == 1:
            return True
    return False
        
test()