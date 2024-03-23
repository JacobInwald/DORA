import numpy as np
import cv2
from dora_msgs.msg import Toy, Toys
from queue import PriorityQueue
from .occupancy_map import OccupancyMap


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

    def route(self, start, end, occ: OccupancyMap) -> np.ndarray:
        """
        This function calculates the route from the current position to the end point.

        Parameters:
            end (np.ndarray): The end point coordinates on the map.
            map (OccupancyMap): The occupancy map object representing the environment.

        Returns:
            np.ndarray: The calculated route as an array of coordinates.
        """
        move_dist = 0.05
        occ = occ.change_res(move_dist)
        directions = np.array([[0, 1], [1, 0], [0, -1], [-1, 0]])

        g = {}
        g[str(start)] = 0
        alpha = 2  # the penalty parameter for being close to obstacles
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

                if occ.sample_coord(n) > 0.5:
                    g[strNext] = g[strCur] + np.inf
                    continue

                # penalize for obstacles in 3 move_dist
                penalty = occ.sample_coord(n, mean=True, n=3)
                if penalty != 1:
                    penalty = 0
                penalty *= 2
                penalty += int(dir != n_d)

                try:
                    if g[strCur] + move_dist < g[strNext]:
                        g[strNext] = g[strCur] + move_dist
                        parent[strNext] = cur if n_d != dir else parent[strCur]
                except KeyError:
                    g[strNext] = g[strCur] + move_dist
                    q.put((f(n, penalty), (tuple(n), n_d)))
                    parent[strNext] = cur if n_d != dir else parent[strCur]

        # Retrieve path
        path = [start]
        if found:
            route = [cur]
            cur = str(cur)
            while cur in parent:
                route.append(parent[cur])
                cur = str(parent[cur])
            route.reverse()
            path = np.array(route)

        # Reduce points in path
        cur_ = path[0]
        path_ = []
        for pt_ in path[1:]:
            mask = np.zeros(occ.map.shape)
            cv2.line(mask, occ.translate(cur_), occ.translate(pt_), 1, 3)
            score = np.max(mask * occ.map)
            if score > 0.5:
                path_.append(last)
                cur_ = last
            last = pt_
        path_.append(path[-1])

        return path_

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

    def next_retrieve_pt(self, map: OccupancyMap, toys: Toys, cur: np.ndarray) -> tuple[np.ndarray, Toy]:
        """
        Finds the next point to retrieve toy in the occupancy map. (nearest toy from start position)

        Parameters:
        - map: OccupancyMap object representing the map.
        - toys: A list of Toy objects.
        - cur: The start point of DORA.

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

    def next_unload_pt(self, map: OccupancyMap, toy: Toy) -> np.ndarray:
        """
        Finds the closest unload point to the toy in the occupancy map. (Nearest unload point from start position)

        Parameters:
        - map: OccupancyMap object representing the map.
        - toy: The toy to unload.

        Returns:
        - The coordinate of the unload point

        """

        closest_unload_point = None
        shortest_dist = None

        for _, (unload_point, unload_point_class) in enumerate(self.unload_points):
            if unload_point_class != Toy.cls:
                continue
            this_dist = np.linalg.norm([unload_point[0], unload_point[1]], [
                                       Toy.position.x, Toy.position.y])
            if closest_unload_point == None:
                shortest_dist = this_dist
                closest_unload_point = unload_point
            if this_dist < shortest_dist:
                shortest_dist = this_dist
                closest_unload_point = unload_point

        return closest_unload_point


# def test():
#     occ = OccupancyMap.load('reference_map.npz')
#     router = Router()
#     start = np.array([2.5, -0.5])
#     end = np.array([2.5, -1.5])
#     route = router.route_(start, end, occ)
#     pt_ = route[0]
#     for pt in route[1:]:
#         cv2.line(occ.map, occ.translate(pt_), occ.translate(pt), 2, 1)
#         pt_ = pt
#     plt.imshow(occ.map)
#     plt.show()

# test()
