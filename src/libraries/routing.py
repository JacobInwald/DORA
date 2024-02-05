import numpy as np
from matplotlib import pyplot as plt
import simulator as simulate
import os, shutil
from queue import PriorityQueue


# ! Library Methods

def manFuzz(grid: np.ndarray) -> np.ndarray:
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
                grid[i, j] = np.mean([grid[i+x, j+y] for x in range(-1,2) for y in range(-1,2)])
            except IndexError:
                pass
    return grid


def bresenham(start: np.ndarray, end: np.ndarray, res: float=1) -> np.ndarray[np.float64]:
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
        result[i] = (start + (i / length) * direction).round(int(-np.log10(res)) if res < 1 else 0) // res
    
    return result


class GPS:
    """
    Represents a GPS sensor.
    
    Attributes:
    - pos: numpy.ndarray - The current position of the robot.
    TODO: Implement the GPS sensor class (Wilfredo)
    """
    
    def __init__(self):
        pass
    
    
    def locate(self, controller: "simulate.Controller") -> np.ndarray, float:
        """
        Updates the position of the robot based on the GPS sensor.
        """
        img = controller.getOverheadImage()
        return self.getPosInImage(img), self.getRotInImage(img)
    
    
    def getPosInImage(self, img: np.ndarray) -> np.ndarray:
        """
        Returns the position of the robot in the given image.

        Params:
            img (np.ndarray): The image to search for the robot.

        Returns:
            np.ndarray: The position of the robot in the image.
            
        TODO: Implement the logic to find the position of the robot in the image. (Wilfredo)
        """
        pass
    
    
    def getRotInImage(self, img: np.ndarray) -> float:
        """
        Returns the position of the robot in the given image.

        Params:
            img (np.ndarray): The image to search for the robot.

        Returns:
            np.ndarray: The position of the robot in the image.
            
        TODO: Implement the logic to find the position of the robot in the image. (Wilfredo)
        """
        pass
    

class Router:
    """
    The Router class is responsible for calculating routes and navigating the robot on the map.
    TODO: Finish it (Keming)
    """

    def __init__(self, controller: "simulate.Controller"):
        self.controller = controller


    def route(self, end: np.ndarray, map: "OccupancyMap") -> np.ndarray:
        """
        This function calculates the route from the current position to the given end point on the map.
        
        Parameters:
            controller (simulate.Controller): The controller object used for simulation.
            end (np.ndarray): The end point coordinates on the map.
            map (OccupancyMap): The occupancy map object representing the environment.
        
        Returns:
            np.ndarray: The calculated route as an array of coordinates.
        """
        start = self.controller.pos
        moveDist = map.resolution
        
        g = {}
        g[str(start)] = 0
        f = lambda p: np.linalg.norm(p - end) + g[str(p)]
        q = PriorityQueue()
        q.put((f(start), start))
        parent = {}
        found = False
        
        # A* Algorithm
        while q.empty() == False:
            _, cur = q.get()
            cur = np.array(cur)
            strCur = str(cur)
            
            if np.linalg.norm(cur - end) < moveDist:
                found = True
                break
            
            for d in np.array([(x, y) for x in [-1, 0, 1] for y in [-1, 0, 1] if (x,y) != (0,0)]):
                
                next = cur + d * moveDist
                strNext = str(next)
                
                # TODO: Penalize being near obstacles
                if map.sampleCoord(next) > 0.2:
                    continue
                
                try:
                    if g[strCur] + self.controller.move_dist < g[strNext]:
                        g[strNext] = g[strCur] + moveDist
                        parent[strNext] = cur
                except KeyError:
                    g[strNext] = g[strCur] + moveDist
                    q.put((f(next), tuple(next)))
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


    def nextMappingPoint(map: "OccupancyMap") -> np.ndarray:
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
                d = (p - cloud.origin) / np.linalg.norm(p - cloud.origin) * map.resolution
                if any(np.linalg.norm(p - c.origin) < 0.5 for c in clouds) or \
                    map.sampleCoord(p + d, yx=True, mean=True) <= 1e-4 or \
                    map.sampleCoord(p + d, yx=True, mean=True) >= 0.55:
                    continue
                return np.roll(p - d, 1)
        
            
        return clouds[-1].origin
     
     
    def nextTidyingPoint(map: "OccupancyMap") -> np.ndarray:
        """
        Finds the next tidying point in the occupancy map.

        Parameters:
        - map: OccupancyMap object representing the map.

        Returns:
        - numpy array representing the next tidying point.
        
        TODO: Implement the logic to find the next tidying point (Keming)
        TODO: Feel free to change the structure as well, just document it in the PR
        """
        pass
    
    
    def toPoint(self, end: np.ndarray) -> None:
        """
        Move the robot to the given end point on the map.
        
        Parameters:
            controller (simulate.Controller): The controller object used for simulation.
            end (np.ndarray): The end point coordinates on the map.
        TODO: Probably should be moved to the Controller class (fine for now)
        """
        dist = np.linalg.norm(end - self.controller.pos)
        angle = np.arctan2(end[1] - self.controller.pos[1], end[0] - self.controller.pos[0]) - self.controller.rot
        self.controller.turn(angle)
        self.controller.forward(dist)


    def followRoute(self, route: np.ndarray) -> None:
        """
        Follow the given route on the map.
        
        Parameters:
            controller (simulate.Controller): The controller object used for simulation.
            route (np.ndarray): The calculated route as an array of coordinates.
        TODO: Probratebly should be moved to the Controller class (fine for now)
        """
        for i in range(1, len(route)):
            self.toPoint(route[i])
    

class PointCloud:
    """
    Represents a point cloud generated from lidar data.
    
    Attributes:
    - lidar: numpy.ndarray - The lidar data.
    - origin: numpy.ndarray - The origin of the point cloud.
    - maxScanDist: float - The maximum scan distance.
    - objectCloud: list - The cloud containing points representing objects.
    - emptyCloud: list - The cloud containing points representing empty space.
    - min: numpy.ndarray - The minimum values of the point cloud.
    - max: numpy.ndarray - The maximum values of the point cloud.
    - center: numpy.ndarray - The center point of the point cloud.
    """
    
    def __init__(self, lidar: np.ndarray, origin: np.ndarray, maxScanDist: float):
        self.lidar = lidar
        self.origin = np.roll(origin,1)
        self.maxScanDist = maxScanDist

        self.initClouds(lidar)
        self.initMinMax()


    def initMinMax(self) -> None:
        """
        Initializes the minimum and maximum values of the point cloud.
        """
        emin = np.array([np.inf, np.inf])
        emax = np.array([-np.inf, -np.inf])
        omin = np.array([np.inf, np.inf])
        omax = np.array([-np.inf, -np.inf])
        
        if len(self.objectCloud) > 0:
            omin = np.array([self.objectCloud[:,0].min(), self.objectCloud[:,1].min()])
            omax = np.array([self.objectCloud[:,0].max(), self.objectCloud[:,1].max()])
        if len(self.emptyCloud) > 0:
            emin = np.array([self.emptyCloud[:,0].min(), self.emptyCloud[:,1].min()])
            emax = np.array([self.emptyCloud[:,0].max(), self.emptyCloud[:,1].max()])
        
        self.min = np.array([min(omin[0], emin[0]), min(omin[1], emin[1])])
        self.max = np.array([max(omax[0], emax[0]), max(omax[1], emax[1])])
        
        if np.isinf(self.min).any() or np.isinf(self.max).any():
            self.min = np.array([0, 0])
            self.max = np.array([0, 0])
        
        self.min -= 0.5
        self.max += 0.5
        self.center = np.round((self.max - self.min) / 2, 3)
        
        
    def initClouds(self, lidar) -> np.ndarray:
        """
        Initializes the object cloud and empty cloud based on the lidar data.
        TODO: use x, y not y, x
        """
        oScan = np.array([p for p in lidar if np.isfinite(p).all()])
        eScan = np.array([p for p in lidar if not np.isfinite(p).all()])
        
        try: 
            ox = oScan[:,1] * np.cos(oScan[:,0])
            oy = oScan[:,1] * np.sin(oScan[:,0])
            self.objectCloud = np.array([ox, oy]).T
        except Exception:
            self.objectCloud = np.array([])
        
        try:
            ex = (self.maxScanDist-0.2) * np.cos(eScan[:,0])
            ey = (self.maxScanDist-0.2) * np.sin(eScan[:,0])
            self.emptyCloud = np.array([ex, ey]).T
        except Exception:
            self.emptyCloud = np.array([])


    def cloud(self):
        """
        Returns the combined point cloud.
        """
        if len(self.objectCloud) == 0:
            return self.emptyCloud
        elif len(self.emptyCloud) == 0:
            return self.objectCloud
        else:
            return np.append(self.objectCloud, self.emptyCloud, axis=0)
    
    
    def removeDuplicates(self, clouds: ["PointCloud"]) -> None:
        """
        Removes duplicate points from the object cloud and empty cloud based on the given list of clouds.
        
        Parameters:
        - clouds: list - The list of PointCloud objects to compare with.
        """
        # TODO: find efficient method of removing duplicates
        f = lambda p, cloud: any((np.abs(c - p) < 0.001).any() for c in cloud)
        
        for cloud in clouds:
            for p in self.objectCloud:
                if f(p, cloud.objectCloud):
                    self.objectCloud = np.delete(self.objectCloud, np.where((self.objectCloud == p).all(axis=1)), axis=0)
            for p in self.emptyCloud:
                if f(p, cloud.emptyCloud):
                    self.emptyCloud = np.delete(self.emptyCloud, np.where((self.emptyCloud == p).all(axis=1)), axis=0)
        self.initMinMax()


    def transform(self, offset: np.ndarray) -> None:
        """
        Transforms the object cloud and empty cloud by adding the given offset.
        
        Parameters:
        - offset: numpy.ndarray - The offset to be added to the point cloud.
        """
        if len(self.objectCloud) > 0:
            self.objectCloud += offset
        if len(self.emptyCloud) > 0:
            self.emptyCloud += offset
        self.initMinMax()
        
        
    def isEmpty(self, n=5):
        """
        Checks if the point cloud is empty.
        
        Parameters:
        - n: int - The threshold value for considering the point cloud as empty.
        
        Returns:
        - bool - True if the point cloud is empty, False otherwise.
        """
        return len(self.objectCloud) + len(self.emptyCloud) <= n
    


class OccupancyMap:  
    """
    Represents an occupancy map based on obstacle coordinates and resolution. 
    nb: The coords are in the form [x, y].T or (y, x)
    
    Attributes:
    - offset: numpy.ndarray - The offset of the occupancy map.
    - pointclouds: list - The coordinates of the obstacles.
    - resolution: float - The resolution of the occupancy map.
    """
    
    def __init__(self, offset: np.ndarray, pointclouds: list['PointCloud'], resolution: float=0.05):
        """
        Initializes an occupancy map based on the given obstacle coordinates and resolution.

        Params:
            offset (np.ndarray): The offset of the occupancy map, given (x, y).
            pointclouds (list[np.ndarray]): The coordinates of the obstacles, given [ox, oy].T.
            resolution (float, optional): Resolution of the occupancy map. Defaults to 0.02.

        Returns:
            None
        """
        # explanatory
        self.offset = np.roll(offset,1)
        self.resolution = resolution 
        self.pointclouds = pointclouds if not isinstance(pointclouds, PointCloud) else [pointclouds]

        # Gets the minimum and maximum x and y coordinates of the obstacles
        mins = np.array([c.min for c in self.pointclouds])
        maxs = np.array([c.max for c in self.pointclouds])

        self.min = np.array([min(mins[:,0]), min(mins[:,1])])
        self.max = np.array([max(maxs[:,0]), max(maxs[:,1])])

        # Calculate the shape of the occupancy map
        self.shape = np.round((self.max - self.min) / resolution).astype(int)

        # Generate the occupancy map with probability 0.5 in each cell
        self.map = np.ones(self.shape) * 0.5


    
    def generate(self, fuzz: bool = True) -> "OccupancyMap":
        """
        Generates an occupancy map based on the given obstacle coordinates.

        Params:
            fuzz (bool): Whether to apply a fuzzy filter to the occupancy map. Default is True.
            
        Returns:
            map (numpy.ndarray): Occupancy map representing the environment, where 0.0 represents free area and 1.0 represents occupied area.
        """

        # Draw Empty Space
        for cloud in self.pointclouds:
            o = self.translate(cloud.origin, True)    # Normalise the origin
            for p in cloud.cloud():
                i = self.translate(p, True)   # Normalise the point
                # Draw ray between origin and point
                line = bresenham(o, i)
                for pl in line:
                    try:
                        self.map[int(pl[0])][int(pl[1])] = 0  # free area 0.0
                    except IndexError:
                        pass
        
        # Draw on Obstacles
        wallThickness = 3
        for cloud in self.pointclouds:
                for p in cloud.objectCloud:
                    i = self.translate(p, True)
                    
                    for w in range(wallThickness):
                        try:
                            prob = 1
                            self.map[i[0] + w][i[1]] = prob  # extend the occupied area
                            self.map[i[0]][i[1] + w] = prob  # extend the occupied area
                            self.map[i[0] + w][i[1] + w] = prob  # extend the occupied area
                        except IndexError:
                            pass
            
        # Apply Fuzzy Filter
        self.map = manFuzz(self.map) if fuzz else self.map

        return self


    def merge(self, others: "OccupancyMap") -> "OccupancyMap":
        """
        Merges the current OccupancyMap with another OccupancyMap.

        Params:
            other (OccupancyMap): The OccupancyMap to merge with.

        Returns:
            OccupancyMap: The merged OccupancyMap.
        """
        if isinstance(others, OccupancyMap):
            others = [others]
        
        for other in others:
            
            newPointcloud = self.pointclouds

            # Normalise the other pointclouds
            for cloud in other.pointclouds:

                cloud.transform(cloud.origin - self.offset)
                # TODO: find efficient method of removing duplicates
                # cloud.removeDuplicates(newPointcloud)
                
                if cloud.isEmpty():
                    continue
                newPointcloud.append(cloud)
        
        # rerolls the offset so it doesn't get rolled twice
        self.__init__(np.roll(self.offset,1), newPointcloud)
        return self
    
    
    def sampleCoord(self, coord: np.ndarray, yx=False, mean=False, n=3) -> np.ndarray:
        """
        Sample the occupancy map at a given coordinate.

        Params:
            coord (np.ndarray): The coordinate to sample.
            yx (bool, optional): If True, the coordinate is treated as (y, x) instead of (x, y). Defaults to False.
                
        Returns:
            np.ndarray: The sampled value, nan if the coordinate is out of bounds.
        """
        try:
            coord = self.translate(coord, yx)
            if not mean:
                return self.map[coord[0], coord[1]]
            else:
                return 1 if 1 in self.map[coord[0]-n:coord[0]+n, coord[1]-n:coord[1]+n] \
                            else np.mean(self.map[coord[0]-n:coord[0]+n, coord[1]-n:coord[1]+n])
        except IndexError:
            return 1
    
    
    def translate(self, coord: np.ndarray, yx=False) -> np.ndarray:
        """
        Translates a coordinate to a grid index.

        Params:
            coord (float): The coordinate value to be translated.
            yx (bool, optional): If True, the coordinate is treated as (y, x) instead of (x, y). Defaults to False.

        Returns:
            int: The grid index corresponding to the translated coordinate.
        """
        if yx:
            return np.round(((coord - self.offset) - self.min) / self.resolution).astype(int)
        else:
            return np.round(((np.roll(coord,1) - self.offset) - self.min) / self.resolution).astype(int)
    
    
    def normalise(self) -> None:
        """
        Normalizes the pointclouds by applying the offset.
        """
        for c in self.pointclouds:
            c.transform(self.offset)
        self.offset = np.array([0, 0])
            
    
    def show(self, raycast: bool=False, region: np.ndarray=None, save: bool = False, path: str = '') -> None:
        """
        Display the occupancy map using matplotlib.

        Params:
            raycast (bool, optional): If True, displays the occupancy map with raycast visualization. Defaults to False.
            region (np.ndarray, optional): The region to be plotted in the raycast visualization. Defaults to None.
        """   
        plt.figure(1, figsize=(4, 4))
        if not raycast:
            plt.imshow(self.map, cmap="PiYG_r")
            plt.clim(0, 1)
            plt.gca().set_xticks(np.arange(-.5, self.shape[1], 1), minor=True)
            plt.gca().set_yticks(np.arange(-.5, self.shape[0], 1), minor=True)
            plt.grid(True, which="minor", color="w", linewidth=0.6, alpha=0.5)
            plt.colorbar()
        else:
            if not region is None:
                plt.plot(region[:,0], region[:,1], "bo-")
            
            for cloud in self.pointclouds:
                if len(cloud.objectCloud) == 0:
                    continue

                c = cloud.objectCloud + self.offset
                plt.plot([c[:, 1], self.offset[1] + np.zeros(np.size(c[:, 1]))], [c[:, 0], self.offset[0] + np.zeros(np.size(c[:, 0]))], "ro-")
            plt.axis("equal")
            plt.plot(self.offset[1], self.offset[0], "ob")
            plt.gca().set_aspect("equal", "box")
            bottom, top = plt.ylim()  # return the current y-lim
            plt.ylim((top, bottom))  # rescale y axis, to match the grid orientation
            plt.grid(True)
        if save:
            plt.savefig(path)
            plt.close()
        else:
            plt.show()



    
# ! TEST CODE

def test_1():
    # Initialise Bounds
    region = [np.array([[-2, 4], [3, 4], [2,2], [4, 3], [4, 0], [4, 0], [2, -1], [-2, 0]]), \
                    np.array([[-1,3],[-1,2.5],[-1.5,3]])]
    res = 10    # Resolution of LIDAR scans
    max_dist = 1.5 # max distance of LIDAR scans
    # Initialise Controller
    controller = simulate.Controller(np.array([0, 0]), 0, region, res, max_scan_dist=max_dist)
    # Initialise PointCloud with a LIDAR scan of the environment
    cloud = PointCloud(controller.getLiDARScan(), controller.pos, controller.max_scan_dist)
    # Initialise a new OccupancyMap with the PointCloud
    m = OccupancyMap(controller.pos, [cloud])
    # Initiliase a router
    router = Router(controller)
    # Moves the controller 33 times
    for i in range(0, 33):
        # Create a new scan
        cloud = PointCloud(controller.getLiDARScan(), controller.pos, controller.max_scan_dist)
        # Merge the new Occupancy Map with the previous one
        m.merge(OccupancyMap(controller.pos, [cloud]))
        # Generate it and show it
        m.generate()
        m.show()
        
        # Move to the next mapping point
        next = router.nextMappingPoint(m, max_dist)
        path = router.route(controller, next, m)
        next = path[-1]
        
        # Move and trace path on the map
        for p in path:
            router.toPoint(controller, p)
            p = m.translate(p)
            m.map[p[0], p[1]] = 1
        
        m.show()


def test_2():
    
    if os.path.exists("move"):
        shutil.rmtree("move")
    os.makedirs("move")
    
    if os.path.exists("map"):
        shutil.rmtree("map")
    os.makedirs("map")
    
    region = [np.array([[-2, 4], [3, 4], [2,2], [4, 3], [4, 0], [4, 0], [2, -1], [-2, 0]]), \
                    np.array([[-1,3],[-1,2.5],[-1.5,3]])]
    res = 10
    max_dist = 1.5
    controller = simulate.Controller(np.array([0, 0]), 0, region, res, max_scan_dist=max_dist)
    cloud = PointCloud(controller.getLiDARScan(), controller.pos, controller.max_scan_dist)
    router = Router(controller)
    m = OccupancyMap(controller.pos, [cloud])
    index = 1
    for i in range(0, 45):
        cloud = PointCloud(controller.getLiDARScan(), controller.pos, controller.max_scan_dist)
        m.merge(OccupancyMap(controller.pos, [cloud]))
        m.generate()
        
        next = router.nextMappingPoint(m, max_dist)
        path = router.route(controller, next, m)
        next = path[-1]
        
        m.generate()

        o=0
        for p in path:
            controller.show(save=True, path=f"move/{index+o}.png")
            o+=1
            router.toPoint(controller, p)
        

        index+=o
 
 
def test_3():
    if os.path.exists("combine"):
        shutil.rmtree("combine")
    os.makedirs("combine")
    
    region = [np.array([[-2, 4], [3, 4], [2,2], [4, 3], [4, 0], [4, 0], [2, -1], [-2, 0]]), \
                    np.array([[-1,3],[-1,2.5],[-1.5,3]])]
    res = 10
    max_dist = 1.5
    controller = simulate.Controller(np.array([0, 0]), 0, region, res, max_scan_dist=max_dist)
    cloud = PointCloud(controller.getLiDARScan(), controller.pos, controller.max_scan_dist)
    m = OccupancyMap(controller.pos, [cloud])
    router = Router(controller)
    index = 1
    combine = 1
    compress = 3
    for i in range(0, 40):
        cloud = PointCloud(controller.getLiDARScan(), controller.pos, controller.max_scan_dist)
        m.merge(OccupancyMap(controller.pos, [cloud]))
        m.generate()
        
        next = router.nextMappingPoint(m, max_dist)
        path = router.route(controller, next, m)
        next = path[-1]

        for x in range(len(path) // compress):
            m.show(save=True, path=f"combine/{combine}.png")
            combine += 1

        o=0
        for p in path:
            controller.show(save=True, path=f"combine/{combine}.png")
            if o % compress == 0: 
                combine += 1
            o+=1
            router.toPoint(controller, p)

        index+=o
        
