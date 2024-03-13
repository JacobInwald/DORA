import numpy as np
import matplotlib.pyplot as plt
# from dora_msgs.msg import Map
from point_cloud import PointCloud
from utils import *
import cv2 

class OccupancyMap:
    """
    Represents an occupancy map based on obstacle coordinates and resolution.
    nb: The coords are in the form [x, y].T or (y, x)

    Attributes:
    - offset: numpy.ndarray - The offset of the occupancy map.
    - pointclouds: list - The coordinates of the obstacles.
    - resolution: float - The resolution of the occupancy map.
    """

    def __init__(
        self,
        offset: np.ndarray,
        pointclouds: list["PointCloud"],
        resolution: float = 0.05,
    ):
        """
        Initialises an occupancy map based on the given obstacle coordinates and resolution.

        Params:
            offset (np.ndarray): The offset of the occupancy map, given (x, y).
            pointclouds (list[np.ndarray]): The coordinates of the obstacles, given [ox, oy].T.
            resolution (float, optional): Resolution of the occupancy map. Defaults to 0.02.

        Returns:
            None
        """
        # explanatory
        self.offset = offset
        self.resolution = resolution
        self.pointclouds = (pointclouds
                            if not isinstance(pointclouds, PointCloud) else
                            [pointclouds])
        if len(self.pointclouds) == 0:
            self.map = np.zeros((1, 1))
        else:
            self.map = self.generate()
        

    def generate(self, fuzz: bool = True) -> "OccupancyMap":
        """
        Generates an occupancy map based on the given obstacle coordinates.

        Params:
            fuzz (bool): Whether to apply a fuzzy filter to the occupancy map. Default is True.

        Returns:
            map (numpy.ndarray): Occupancy map representing the environment, where 0.0 represents free area and 1.0 represents occupied area.
        """
        
        pad_size = int(self.pointclouds[0].maxScanDist / self.resolution) * 2
        self.map = np.zeros((int(pad_size/2), int(pad_size/2)))
        img = np.pad(self.map, pad_size, mode='constant', constant_values=0)
        first = True
        
        for cloud in self.pointclouds:
            img = self.merge_cloud_into_map(cloud, fuzz=False, overwrite=first, set_map=False)
            first = False
        
        self.map = man_fuzz(img)
        return self.map      
    
    # Merging

    def merge_cloud_into_map(self, cloud: "PointCloud", fuzz=True, overwrite=False, set_map=True) -> "OccupancyMap":
                
        # Init Vars
        img = np.zeros_like(self.map)
        pad_size = int(cloud.maxScanDist / self.resolution) * 2
        img = np.pad(img, pad_size, mode='constant', constant_values=0)
        
        # Paste previous map
        ox, oy = self.translate(self.offset) + pad_size
        oh, ow = self.map.shape
        img[ox-ow//2:ox+ow//2, oy-oh//2:oy+oh//2] = self.map
        
        # Generate and norm cloud image
        cloud_img = cloud.generate(res=self.resolution)
        h, w = cloud_img.shape
        x, y = self.translate(cloud.origin) + pad_size

        # Init Masks
        img_zeros = img <= 0.05
        cloud_zeros = cloud_img <= 0.05
        
        # Combine the cloud
        subsample = img[x-(w//2):x+(w//2), y-(h//2):y+(w//2)]
        
        img[x-(w//2):x+(w//2), y-(h//2):y+(w//2)] = np.maximum(subsample, cloud_img)

        # Reset 0s
        if not overwrite:
            img[x-(w//2):x+(w//2), y-(h//2):y+(w//2)][cloud_zeros] = 0
            img[img_zeros] = 0
        
        # Crop back to original sizes
        x, y = self.translate(self.offset) + pad_size
        w, h = pad_size, pad_size
        img = img[x-(w//2):x+(w//2), y-(h//2):y+(w//2)]
        img = man_fuzz(img) if fuzz else img
        
        if set_map:
            self.map = img
            
        return img

    # Point Sampling

    def localise_cloud(self, cloud: "PointCloud") -> "PointCloud":
        """
        Localise a pointcloud to the occupancy map.

        Params:
            cloud (PointCloud): The pointcloud to be localised.

        Returns:None
            PointCloud: The localised pointcloud.
        """
        # Normalise reference map
        padding = int(cloud.maxScanDist / 0.05)
        ref = np.copy(self.map).astype(np.float32).T
        ref = np.pad(ref, padding, mode='constant', constant_values=0.5)
        
        # INIT
        min_loss = np.inf
        best_pose = np.array([0, 0, 0])
        template = cloud.generate(rot=0, res=self.resolution).astype(np.float32).T
        #  TODO: increase resolution for low loss values ...
        for r in [np.deg2rad(i) for i in np.arange(0, 360, 1)]:
            # Generate the rotated template
            _template = rotate_image(template, np.rad2deg(r))
            w, h = _template.shape
            
            # Match the Template
            ret = cv2.matchTemplate(ref, _template, cv2.TM_CCOEFF)
            _, _, _, max_loc = cv2.minMaxLoc(ret)
            
            # Get the center of the match
            center = np.array([max_loc[0] + (w / 2), max_loc[1] + (h / 2)])
            center -= padding * np.array([1, 1])
            x, y = self.translate(center, False)
            r = 2*np.pi - r
            pose = np.array([x, y, r])
            
            # Evaluate Match Quality
            top_left = max_loc
            bottom_right = (top_left[0] + w, top_left[1] + h)
            res = np.copy(ref[top_left[1]:bottom_right[1], top_left[0]:bottom_right[0]])
            res_center = np.array([res.shape[0] / 2, res.shape[1] / 2])
            
            # Mask the area outside the scan range
            Y, X = np.ogrid[:h, :w]
            dist_from_center = np.sqrt((X - res_center[0])**2 + (Y-res_center[1])**2)
            mask = dist_from_center <= (cloud.maxScanDist / self.resolution)
            res[~mask] = 0.5
            
            loss = np.sum((res-_template) ** 2) / (res.shape[0] * res.shape[1])
            
            if loss < min_loss:
                min_loss = loss
                best_pose = pose
                best_temp = _template
                best_res = res
                
        # plt.imshow(np.hstack([best_res, best_temp, np.abs(best_res-best_temp)]))
        # plt.show()
        return best_pose

    def sampleCoord(self,
                    coord: np.ndarray,
                    yx=False,
                    mean=False,
                    n=3) -> np.ndarray:
        """
        Sample the occupancy map at a given coordinate.

        Params:
            coord (np.ndarray): The coordinate to sample.
            yx (bool, optional): If True, the coordinate is treated as (y, x) instead of (x, y). Defaults to False.

        Returns:
            np.ndarray: The sampled value, nan if the coordinate is out of bounds.
        """
        try:
            coord = self.translate(coord)
            if not mean:
                return self.map[coord[0], coord[1]]
            else:
                return (1 if 1 in self.map[coord[0] - n:coord[0] + n,
                                           coord[1] - n:coord[1] + n] else
                        np.mean(self.map[coord[0] - n:coord[0] + n,
                                         coord[1] - n:coord[1] + n]))
        except IndexError:
            return 1

    def translate(self, coord: np.ndarray, to_index: bool = True) -> np.ndarray:
        """
        Translates a coordinate to a grid index.

        Params:
            coord (float): The coordinate value to be translated.

        Returns:
            int: The grid index corresponding to the translated coordinate.
        """
        if to_index:
            return np.array(np.round(((coord - self.offset) / self.resolution) + np.array(self.map.shape) / 2)).astype(int)
        else:
            return (coord - np.array(self.map.shape) / 2) * self.resolution + self.offset

    def change_res(self, new_res: float) -> "OccupancyMap":
        """
        Changes the resolution of the occupancy map.

        Params:
            res (float): The new resolution of the occupancy map.

        Returns:
            OccupancyMap: The occupancy map with the new resolution.
        """
        old_res = self.resolution
        self.resolution = new_res
        new_shape = np.array(self.map.shape) * (old_res / new_res)

        self.map = cv2.resize(self.map, (int(new_shape[1]), int(new_shape[0])), cv2.INTER_NEAREST)
        
        return self
    
    # Saving

    def save(self, name: str = None) -> None:
        """        # # Crop back to original sizes
        
        x, y = self.translate(self.offset) + pad_size
        w, h = pad_size, pad_size
        img = img[x-(w//2):x+(w//2), y-(h//2):y+(w//2)]
        Save the occupancy map to a file.

        Params:
            path (str): The path to save the occupancy map to.
        """
        if name is None:
            name = "occupancy_map_" + hash(self)
        np.savez(f'data/maps/{name}.npz', offset=self.offset, resolution=self.resolution, map=self.map)

    @staticmethod
    def load(path: str):
        """
        Save the occupancy map to a file.

        Params:
            path (str): The path to save the occupancy map to.
        """
        occ = OccupancyMap(np.array([0, 0]), [])
        data = np.load(path)
        occ.offset = data['offset']
        occ.resolution = data['resolution']
        occ.map = data['map']
        
        return occ

    # ROS Support
    
    def to_msg(self) -> "Map":
        """
        Converts the OccupancyMap to a ROS message.

        Returns:
            Map: The ROS message representing the OccupancyMap.
        """
        msg = Map()
        msg.offset.x = self.offset[0]
        msg.offset.y = self.offset[1]
        msg.resolution = self.resolution
        msg.map = self.map
        return msg

    @staticmethod
    def from_msg(msg: "Map") -> "OccupancyMap":
        """
        Converts a ROS message to an OccupancyMap.

        Params:
            msg (Map): The ROS message to be converted.

        Returns:
            OccupancyMap: The OccupancyMap representing the ROS message.
        """
        pos = np.array([msg.offset.x, msg.offset.y])
        map = np.array(msg.map)
        res = msg.resolution
        occ =  OccupancyMap(pos, [], res)
        occ.map = map
        return occ

    # Displaying

    def show(
        self,
        raycast: bool = False,
        region: np.ndarray = None,
        save: bool = False,
        path: str = "",
    ) -> None:
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
            plt.gca().set_xticks(np.arange(-0.5, self.shape[1], 1), minor=True)
            plt.gca().set_yticks(np.arange(-0.5, self.shape[0], 1), minor=True)
            plt.grid(True, which="minor", color="w", linewidth=0.6, alpha=0.5)
            plt.colorbar()
        else:
            if not region is None:
                plt.plot(region[:, 0], region[:, 1], "bo-")

            for cloud in self.pointclouds:
                if len(cloud.objectCloud) == 0:
                    continue

                c = cloud.objectCloud + self.offset
                plt.plot(
                    [c[:, 0], self.offset[0] + np.zeros(np.size(c[:, 0]))],
                    [c[:, 1], self.offset[1] + np.zeros(np.size(c[:, 1]))],
                    "ro-",
                )
            plt.axis("equal")
            plt.plot(self.offset[1], self.offset[0], "ob")
            plt.gca().set_aspect("equal", "box")
            bottom, top = plt.ylim()  # return the current y-lim
            # rescale y axis, to match the grid orientation
            plt.ylim((top, bottom))
            plt.grid(True)
        if save:
            plt.savefig(path)
            plt.close()
        else:
            plt.show()


class Map:
    
    def __init__(self, regions) -> None:
        self.regions = regions
    
    
    def line_intersects_region(self, p1, p2):
        """
        Check if a line intersects with any of the regions in the map.
        """
        return lineIntersectPolygon(p1, p2, self.regions)
    
    def cast_ray(self,
        start: np.ndarray,
        angle: float,
        noise=True,
        max_dist=np.inf,
    ) -> float:
        """
        Calculates the minimum distance from a starting point to a boundary region
        along a given angle using LiDAR.

        Params:
            start (np.ndarray): The starting point coordinates.
            angle (float): The angle in radians.
            bounds (np.ndarray): The boundary region coordinates.

        Returns:
            float: The minimum distance from the starting point to the boundary region.
            nb: return inf if there is no intersection
        """

        dir_lidar = np.round(np.array([np.cos(angle), np.sin(angle)]), 3)
        min_dist = np.inf

        for bound in self.regions:
            for i in range(len(bound)):
                # Get a bounding line of the boundary region
                p1 = bound[i]
                dir = np.round(
                    bound[i + 1] - p1 if i < len(bound) - 1 else bound[0] - p1, 3
                )

                #  Get intersection point between the LiDAR ray and the bounding line
                p = pointIntersect(start, dir_lidar, p1, dir)

                # Checks if the intersection point is valid (pointing right way, not nan, not too far away,...)
                if (
                    np.isnan(p).any()
                    or not isBetween(p1, p1 + dir, p)
                    or np.dot(dir_lidar, p - start) < 0
                    or np.linalg.norm(p - start) >= min_dist
                ):
                    continue

                # Update minimum distance
                min_dist = np.linalg.norm(p - start)

        noise = np.random.normal(0, 0.01) if noise else 0
        return min_dist + noise if min_dist < max_dist else np.inf

def isBetween(a: np.ndarray, b: np.ndarray, c: np.ndarray) -> bool:
    """
    Check if point c lies between points a and b.

    Params:
    a (numpy.ndarray): The first point.
    b (numpy.ndarray): The second point.
    c (numpy.ndarray): The point to check.

    Returns:
    bool: True if c lies between a and b, False otherwise.
    """
    return (
        np.linalg.norm(a - c) + np.linalg.norm(b - c) -
        np.linalg.norm(a - b) < 0.0001
    )

def pointIntersect(
    p1: np.ndarray, d1: np.ndarray, p2: np.ndarray, d2: np.ndarray
) -> np.ndarray:
    """
    Calculate the intersection point between two lines defined by a point and a direction vector.

    Params:
    p1 (numpy.ndarray): The starting point of the first line.
    d1 (numpy.ndarray): The direction vector of the first line.
    p2 (numpy.ndarray): The starting point of the second line.
    d2 (numpy.ndarray): The direction vector of the second line.

    Returns:
    numpy.ndarray: The intersection point of the two lines.
    nb: this return nan if the lines are parallel
    """
    if d2[0] == 0:
        t = (p2[0] - p1[0]) / d1[0]
    else:
        t = ((p2[1] - p1[1]) + (d2[1] / d2[0]) * (p1[0] - p2[0])) / (
            d1[1] - (d2[1] * d1[0]) / d2[0]
        )

    p = p1 + t * d1

    if any(np.isnan(p)) or any(np.isinf(p)):
        return np.array([np.nan, np.nan])

    return p

def lineIntersectPolygon(
    p: np.ndarray, d: np.ndarray, bounds: np.ndarray
) -> np.ndarray:
    """
    Calculate the intersection point between a line and a polygon.

    Params:
    p (numpy.ndarray): The starting point of the line.
    d (numpy.ndarray): The direction vector of the line.
    bounds (numpy.ndarray): The polygon to check for intersection.

    Returns:
    numpy.ndarray: The intersection point of the line and the polygon.
    """
    intersects = []

    for bound in bounds:
        for i in range(len(bound)):
            # Get a bounding line of the boundary region
            p_bound = bound[i]
            d_bound = np.round(
                bound[i + 1] -
                p_bound if i < len(bound) - 1 else bound[0] - p_bound, 3
            )

            #  Get intersection point between the LiDAR ray and the bounding line
            intersect = pointIntersect(p, d, p_bound, d_bound)

            # Checks if the intersection point is valid (pointing right way, not nan, not too far away,...)
            if (
                np.isnan(intersect).any()
                or not isBetween(p_bound, p_bound + d_bound, intersect)
                or np.dot(d, intersect - p) < 0
                or np.linalg.norm(intersect - p) > np.linalg.norm(d)
            ):
                continue

            # Update minimum distance
            intersects.append(intersect)

    return intersects

def getLiDARScan(pos, map,  rot=0, noise=0.01, max_scan_dist=1.5, scan_res=1) -> np.ndarray:
        """
        Perform a circular LiDAR scan around a given start point within the specified bounds.

        Returns:
            np.ndarray: An array containing the LiDAR scan results.
        """
        angles = [np.deg2rad(i) for i in np.arange(0, 360, scan_res)]
        scan = [
                [a-rot, map.cast_ray(pos, a, noise, max_scan_dist)]
            for a in angles
        ]
        return scan


def gen_map(res=0.05):
    region = [np.array([[-2, 4], [3, 4], [2,2], [4, 3], [4, 0], [4, 0], [2, -1], [-2, 0]]), \
                    np.array([[-1,3],[-1,2.5],[-1.5,3]])]
    
    m = Map(region)
    r = 0
    max_dist = 5
    
    p = np.array([1.5, 2])
    scan = getLiDARScan(p, m, rot=r, max_scan_dist=max_dist)
    cloud = PointCloud(scan, np.array([1.5, 2]), max_dist, rot=r)
    occ = OccupancyMap(p, cloud, resolution=0.05)
    
    p = np.array([0, 0])
    scan = getLiDARScan(p, m, rot=r, max_scan_dist=max_dist)
    cloud = PointCloud(scan, np.array([0, 0]), max_dist, rot=r)
    occ.merge_cloud_into_map(cloud)
    
    p = np.array([1.5, 3.5])
    scan = getLiDARScan(p, m, rot=r, max_scan_dist=max_dist)
    cloud = PointCloud(scan, np.array([1.5, 3.5]), max_dist, rot=r)
    occ.merge_cloud_into_map(cloud)
    
    occ.change_res(0.05)
    occ.save('test')
    
def test():
    region = [np.array([[-2, 4], [3, 4], [2,2], [4, 3], [4, 0], [4, 0], [2, -1], [-2, 0]]), \
                    np.array([[-1,3],[-1,2.5],[-1.5,3]])]
    
    m = Map(region)
    p = np.array([1, 0])
    r = np.pi / 2
    scan_res = 0.1
    max_dist = 3
    # Error in the scale of the image
    
    scan = getLiDARScan(p, m, rot=r, max_scan_dist=max_dist,scan_res=scan_res)
    cloud = PointCloud(scan, np.array([0, 0]), max_dist, rot=r)
    occ = OccupancyMap.load('data/maps/test.npz')
    
    pose = occ.localise_cloud(cloud)
    
    print(pose)
    
    
# gen_map()
test()