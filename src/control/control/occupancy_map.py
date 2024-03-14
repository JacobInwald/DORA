import numpy as np
import matplotlib.pyplot as plt
# from dora_msgs.msg import Map
from .point_cloud import PointCloud
from .utils import *
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
        self.offset = np.copy(offset)
        self.resolution = resolution
        self.pointclouds = (pointclouds
                            if not isinstance(pointclouds, PointCloud) else
                            [pointclouds])
        if len(self.pointclouds) == 0:
            self.map = np.zeros((1, 1))
        else:
            self.map = self.generate()
     
    # Generation

    def generate(self, fuzz: bool = False) -> "OccupancyMap":
        """
        Generates an occupancy map based on the given obstacle coordinates.

        Params:
            fuzz (bool): Whether to apply a fuzzy filter to the occupancy map. Default is True.

        Returns:
            map (numpy.ndarray): Occupancy map representing the environment, where 0.0 represents free area and 1.0 represents occupied area.
        """
        pad_size = int(self.pointclouds[0].maxScanDist / self.resolution) * 2
        self.map = np.zeros((int(pad_size), int(pad_size)))
        img = np.pad(self.map, pad_size, mode='constant', constant_values=0)
        first = True
        
        for cloud in self.pointclouds:
            img = self.merge_cloud_into_map(cloud, fuzz=fuzz, overwrite=first, set_map=False)
            first = False
        
        self.map = man_fuzz(img, 1) if fuzz else img  
        return self.map      
    
    def merge_cloud_into_map(self, cloud: "PointCloud", fuzz=False, overwrite=False, set_map=True) -> "OccupancyMap":
                
        # Init Vars
        img = np.zeros_like(self.map)
        pad_size = int(cloud.maxScanDist / self.resolution) * 2
        img = np.pad(img, pad_size, mode='constant', constant_values=0)
        
        # Paste previous map
        ox, oy = self.translate(self.offset) + pad_size
        oh, ow = self.map.shape
        img[oy-oh//2:oy+oh//2, ox-ow//2:ox+ow//2] = self.map

        # Generate and norm cloud image
        cloud_img = cloud.generate(res=self.resolution)
        h, w = cloud_img.shape
        x, y = self.translate(cloud.origin) + pad_size
        
        # Init Masks
        img_zeros = img <= 0.05
        cloud_zeros = cloud_img <= 0.05
        
        # Combine the cloud
        subsample = img[y-(h//2):y+(h//2), x-(w//2):x+(w//2)]
        
        # if overwrite:
        #     img[y-(h//2):y+(h//2), x-(w//2):x+(w//2)] = cloud_img
        #     img[img <= 0.6] = 0
        # else:
        #     img[y-(h//2):y+(h//2), x-(w//2):x+(w//2)]  += cloud_img
        #     img[img <= 0.6] = 0
        
        img[y-(h//2):y+(h//2), x-(w//2):x+(w//2)] = np.maximum(subsample, cloud_img)

        # Reset 0s
        if not overwrite:
            
            img_highs = img > 0.65
            img_zeros = np.logical_and(img_zeros, np.logical_not(img_highs))
            
            cloud_highs = cloud_img > 0.65
            cloud_zeros = np.logical_and(cloud_zeros, np.logical_not(cloud_highs))
            img[y-(h//2):y+(h//2), x-(w//2):x+(w//2)][cloud_zeros] = 0
            
            img[img_zeros] = 0
        print(ox-pad_size, oy-pad_size, x-pad_size, y-pad_size)
        plt.imshow(img)
        plt.show()
        # Crop back to original sizes
        ox, oy = self.translate(self.offset) + pad_size
        h, w = self.map.shape
        cx, cy = self.translate(cloud.origin) + pad_size
        w += np.abs(cx - ox)
        h += np.abs(cy - oy)
        if w % 2 != 0:
            w += 1
        if h % 2 != 0:
            h += 1
            
        img = img[oy-(h//2):oy+(h//2), ox-(w//2):ox+(w//2)]
        img = man_fuzz(img, 1) if fuzz else img
        
        if set_map:
            self.map = img
            
        return img

    def localise_cloud(self, cloud: "PointCloud") -> "PointCloud":
        """
        Localise a pointcloud to the occupancy map.

        Params:
            cloud (PointCloud): The pointcloud to be localised.

        Returns:None
            PointCloud: The localised pointcloud.
        """
        # Normalise reference map
        padding = int(cloud.maxScanDist / self.resolution)
        ref = np.copy(self.map).astype(np.float32)
        ref = np.pad(ref, padding, mode='constant', constant_values=0.5)
        
        # INIT
        min_loss = np.inf
        best_pose = np.array([0, 0, 0])
        loss = 1
        next_r = 0
        
        # takes 3.7 secs
        template = cloud.generate(rot=0, res=self.resolution).astype(np.float32)
        
        for i in range(50):
            r=next_r
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
            r = (2*np.pi - r) % (2*np.pi)
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
            
            
            # Gradient descent(ish)
            diff = np.abs(res-_template)
            diff[_template <= 0.55] = 0  # Ignore the 0.5s
            diff[diff > 0.55] = 5 * diff[diff > 0.55] # Penalize the edges
            
            # max_loss = 0.14
            loss = np.sum(diff) / (res.shape[0] * res.shape[1])
            delta_r = np.deg2rad(1*loss*50) 
            next_r = next_r + delta_r
            
            if loss < min_loss:
                min_loss = loss
                best_pose = pose
                best_temp = _template
                best_res = res
                best_diff = diff
                
            if loss < 0.003:
                break
                  
        plt.imshow(np.hstack([best_res, best_temp, best_diff / 5]))
        plt.show()
        return best_pose
    
    #  Helpers

    def sample_coord(self,
                    coord: np.ndarray,
                    mean=False,
                    n=3) -> np.ndarray:
        """
        Sample the occupancy map at a given coordinate.

        Params:
            coord (np.ndarray): The coordinate to sample.
            mean (bool, optional): If True, return the mean value of the surrounding area instead of the sampled value. Defaults to False.
            n (int, optional): The size of the surrounding area to consider when calculating the mean value. Defaults to 3.

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
        if new_shape[0] % 2 != 0:
            new_shape[0] += 1
        if new_shape[1] % 2 != 0:
            new_shape[1] += 1
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


def gen_map(res=0.01):
    region = [np.array([[-2, 4], [3, 4], [2,2], [4, 3], [4, 0], [4, 0], [2, -1], [-2, 0]]), \
                    np.array([[-1,3],[-1,2.5],[-1.5,3]])]
    
    m = Map(region)
    r = 0
    max_dist = 5
    scan_res = 0.5
    
    p = np.array([1.5, 2])
    scan = getLiDARScan(p, m, rot=r, max_scan_dist=max_dist, scan_res=scan_res)
    cloud = PointCloud(scan, np.array([1.5, 2]), max_dist, rot=r)
    occ = OccupancyMap(p, cloud, resolution=res)
    
    p = np.array([0, 0])
    scan = getLiDARScan(p, m, rot=r, max_scan_dist=max_dist, scan_res=scan_res)
    cloud = PointCloud(scan, np.array([0, 0]), max_dist, rot=r)
    occ.merge_cloud_into_map(cloud)
    
    p = np.array([1.5, 3.5])
    scan = getLiDARScan(p, m, rot=r, max_scan_dist=max_dist, scan_res=scan_res)
    cloud = PointCloud(scan, np.array([1.5, 3.5]), max_dist, rot=r)
    occ.merge_cloud_into_map(cloud)
    
    # plt.imshow(occ.map)
    # plt.show()
    
    occ.save('test')
    
def test():
    region = [np.array([[-2, 4], [3, 4], [2,2], [4, 3], [4, 0], [4, 0], [2, -1], [-2, 0]]), \
                    np.array([[-1,3],[-1,2.5],[-1.5,3]])]
    
    m = Map(region)
    p = np.array([2, 3.5])
    r = 1.333 * np.pi
    scan_res = 0.1
    max_dist = 3
    # Error in the scale of the image
    
    scan = getLiDARScan(p, m, rot=r, max_scan_dist=max_dist,scan_res=scan_res)
    cloud = PointCloud(scan, np.array([0, 0]), max_dist, rot=r)
    occ = OccupancyMap.load('data/maps/test.npz')
    
    occ.change_res(0.01)
    plt.imshow(occ.map)
    plt.show()
    plt.imshow(cloud.generate(res=0.05))
    plt.show()
    pose = occ.localise_cloud(cloud)
    print(p, r)
    print(pose)
    
    
# gen_map()
# test()