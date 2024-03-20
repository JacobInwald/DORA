import numpy as np
import cv2
from dora_msgs.msg import Map
from .point_cloud import PointCloud
from .utils import *


class OccupancyMap:
    """
    Represents an occupancy map generated from a list of point clouds.

    Attributes:
        offset (np.ndarray): The offset of the occupancy map.
        pointclouds (list["PointCloud"]): A list of point clouds used to generate the occupancy map.
        resolution (float): The resolution of the occupancy map.

    Methods:
        __init__: Initialize the OccupancyMap object.
        generate: Generates an occupancy map based on the point clouds.
        merge_cloud_into_map: Merges a point cloud into the occupancy map.
        localise_cloud: Localizes a point cloud within the occupancy map.
        sample_coord: Samples the occupancy map at a given coordinate.
        translate: Translates a coordinate to a grid index.
        change_res: Changes the resolution of the occupancy map.
        save: Saves the occupancy map to a file.
        load: Loads the occupancy map from a file.
    """

    def __init__(
        self,
        offset: np.ndarray,
        pointclouds: list["PointCloud"],
        resolution: float = 0.05,
    ):
        """
        Initialize the OccupancyMap object.

        Args:
            offset (np.ndarray): The offset of the occupancy map.
            pointclouds (list["PointCloud"]): A list of point clouds used to generate the occupancy map.
            resolution (float, optional): The resolution of the occupancy map. Defaults to 0.05.
        """
        self.offset = np.copy(offset)
        self.resolution = resolution
        self.pointclouds = (
            pointclouds if not isinstance(
                pointclouds, PointCloud) else [pointclouds]
        )
        if len(self.pointclouds) == 0:
            self.map = np.zeros((1, 1))
        else:
            self.map = self.generate()

    # Generation

    def generate(self) -> "OccupancyMap":
        """
        Generates an occupancy map based on the point clouds.

        Returns:
            OccupancyMap: The generated occupancy map.
        """
        pad_size = int(self.pointclouds[0].maxScanDist / self.resolution) * 2
        self.map = np.ones((int(pad_size), int(pad_size))) * 0.5
        first = True

        for cloud in self.pointclouds:
            self.map = self.merge_cloud_into_map(
                cloud, overwrite=first, set_map=False)
            first = False

        return self.map

    def merge_cloud_into_map(self, cloud: "PointCloud", overwrite: bool = False, set_map: bool = True) -> "OccupancyMap":
        """
        Merges a point cloud into the occupancy map.

        Args:
            cloud (PointCloud): The point cloud to merge into the map.
            overwrite (bool, optional): Whether to overwrite existing map values with the merged values. Defaults to False.
            set_map (bool, optional): Whether to update the map with the merged result. Defaults to True.

        Returns:
            OccupancyMap: The merged occupancy map.
        """

        # Init Vars
        img = np.ones_like(self.map) * 0.5
        pad_size = int(cloud.maxScanDist / self.resolution) * 2
        img = np.pad(img, pad_size, mode='constant', constant_values=0.5)

        # Paste previous map
        ox, oy = self.translate(self.offset) + pad_size
        oh, ow = self.map.shape
        img[oy-oh//2:oy+oh//2, ox-ow//2:ox+ow//2] = self.map

        # Generate and norm cloud image
        cloud_img = cloud.generate(res=self.resolution)
        h, w = cloud_img.shape
        x, y = self.translate(cloud.origin) + pad_size

        # Init Masks
        img_zeros = img <= 0.3
        cloud_zeros = cloud_img <= 0.3

        # Combine the cloud
        subsample = img[y-(h//2):y+(h//2), x-(w//2):x+(w//2)]
        img[y-(h//2):y+(h//2), x-(w//2):x+(w//2)
            ] = np.maximum(subsample, cloud_img)

        # Reset 0s
        if not overwrite:

            img_highs = img > 0.65
            img_zeros = np.logical_and(img_zeros, ~img_highs)

            cloud_highs = np.logical_and(
                cloud_img > 0.65, img_highs[y-(h//2):y+(h//2), x-(w//2):x+(w//2)])
            cloud_zeros = np.logical_and(cloud_zeros, ~cloud_highs)
            img[y-(h//2):y+(h//2), x-(w//2):x+(w//2)][cloud_zeros] = 0

            img[img_zeros] = 0

        # Crop back to original sizes
        w = np.abs(x - ox) + ow
        h = np.abs(y - oy) + oh
        if w % 2 != 0:
            w += 1
        if h % 2 != 0:
            h += 1

        img = img[oy-(h//2):oy+(h//2), ox-(w//2):ox+(w//2)]

        if set_map:
            self.map = img

        return img

    def fuzz_map(self, n: int = 3) -> np.ndarray:
        self.map = cv2.blur(self.map, (n, n))

    def localise_cloud(self, cloud: "PointCloud") -> "PointCloud":
        """
        Localizes the given point cloud within the occupancy map.

        Args:
            cloud (PointCloud): The point cloud to be localized.

        Returns:
            tuple: A tuple containing the best pose estimate and the match score.
                The best pose estimate is a numpy array of shape (3,) representing
                the x, y, and rotation coordinates of the best pose.
                The match score is a float representing the percentage of match
                between the best pose and the template.

        """

        # INIT
        min_loss = np.inf
        loss = 1
        start_minima = None
        best_pose = np.array([0, 0, 0])
        r = 0

        # Normalise reference map
        ref = np.copy(self.map).astype(np.float32)
        template = cloud.generate(
            rot=r, res=self.resolution).astype(np.float32)
        ref[ref < 0.75] = 0
        template[template < 0.75] = 0

        num_perfect_matches = np.sum(template)

        # O(2^n)
        for i in range(25):
            # Generate the rotated template
            _template = rotate_image(template, np.rad2deg(r))
            _template[_template < 0.75] = 0
            h, w = _template.shape

            # Match the Template TODO: improve it takes the longest
            ret = cv2.matchTemplate(ref, _template, cv2.TM_CCOEFF)
            _, _, _, (max_x, max_y) = cv2.minMaxLoc(ret)

            # Crop to the matched area
            res = ref[max_y:max_y + h, max_x:max_x+w]

            # Calculate the percentage of correct matches
            mask1 = np.abs(res - _template) < 0.25
            mask2 = _template != 0
            mask = np.logical_and(mask1, mask2)
            diff = (num_perfect_matches - np.sum(mask)) / num_perfect_matches

            # Calculate the loss, using an exponential function to penalize large differences
            #   the 0.1 means that when the match over 90% accurate, the loss is less than 1
            loss = 2 ** (6*(diff - 0.1))
            loss = min(loss, 30)

            # Prevents moving out of minimas
            if loss < 1:
                if start_minima is None or \
                    np.abs(loss) > np.abs(start_minima) or \
                        np.abs(loss) < np.abs(start_minima) / 2:
                    start_minima = np.abs(loss)

                if np.abs(loss) > np.abs(start_minima):
                    loss = -loss

            # Update the rotation my minimum amount
            delta_r = np.deg2rad(np.sign(loss) * max(np.abs(loss), 0.1))
            r = (r + delta_r) % (2*np.pi)

            if np.abs(loss) <= np.abs(min_loss) and loss > 0:
                # Find center of match
                center = np.array([max_x + w//2, max_y + h//2])
                x, y = self.translate(center, False)
                # Update the best pose
                best_pose = np.array([x, y, r])
                min_loss = loss
                best_temp = _template
                best_res = res
                best_diff = mask

        # print(f"Best pose: {best_pose}, Score: {np.round(100 * np.sum(best_diff) / np.sum(best_temp), 5)}% match")
        # plt.imshow(np.hstack([best_res, best_temp, np.abs(best_diff- best_temp)]))
        # plt.show()
        return best_pose, np.round(np.sum(best_diff) / np.sum(best_temp), 5)

    #  Helpers

    def translate(self, coord: np.ndarray, to_index: bool = True) -> np.ndarray:
        """
        Translates a coordinate to a grid index.

        Params:
            coord (float): The coordinate value to be translated.

        Returns:
            int: The grid index corresponding to the translated coordinate.
        """
        # [0][0] -> (-shape[0]*res, -shape[1]*res)
        half_width = np.roll(self.map.shape, 1) / 2
        if to_index:
            return np.array(np.round(((coord - self.offset) / self.resolution) + half_width)).astype(int)
        else:
            return (coord - half_width) * self.resolution + self.offset

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
        self.map = cv2.resize(
            self.map, (int(new_shape[1]), int(new_shape[0])), cv2.INTER_NEAREST)

        return self

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

    # Saving

    def save(self, name: str = None) -> None:
        """
        Save the occupancy map to a file.

        Params:
            path (str): The path to save the occupancy map to.
        """
        if name is None:
            name = "occupancy_map_" + hash(self)
        np.savez(f'data/maps/{name}.npz', offset=self.offset,
                 resolution=self.resolution, map=self.map)

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
        occ = OccupancyMap(pos, [], res)
        occ.map = map
        return occ
