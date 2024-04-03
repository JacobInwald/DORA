import numpy as np
import cv2


def man_fuzz(grid: np.ndarray, dim: int = 1) -> np.ndarray:
    """
    Applies the Manhattan Fuzz algorithm to the given grid.

    The Manhattan Fuzz algorithm calculates the mean value of each cell in the grid
    by considering its neighboring cells in the Manhattan distance.

    Params:
        grid (np.ndarray): The input grid.

    Returns:
        np.ndarray: The grid with the Manhattan Fuzz algorithm applied.
    """
    half_dim = int(dim // 2)
    for i in range(grid.shape[0]):
        for j in range(grid.shape[1]):
            # if grid[i, j] >= 0.95 or grid[i, j] == 0:
            #     continue
            l = max(0, i-half_dim)
            r = min(grid.shape[0], i+half_dim)
            u = max(0, j-half_dim)
            d = min(grid.shape[1], j+half_dim)
            grid[l:r, u:d] = np.mean(grid[l:r, u:d])

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


def crop_new(arr, threshold=0.5, deviation=0.05):
    """
    Crops the given array to remove the empty space around it.

    Params:
        arr (np.ndarray): The input array.

    Returns:
        np.ndarray: The cropped array.
    """
    mask1 = arr <= (threshold - deviation)
    mask2 = arr >= (threshold + deviation)
    mask = np.logical_or(mask1, mask2)
    rows = np.flatnonzero((mask).sum(axis=1))
    cols = np.flatnonzero((mask).sum(axis=0))
    return arr[rows.min():rows.max() + 1, cols.min():cols.max() + 1]


def rotate_image(img, angle):
    size_reverse = np.array(img.shape[1::-1])  # swap x with y
    M = cv2.getRotationMatrix2D(tuple(size_reverse / 2.), angle, 1.)
    MM = np.absolute(M[:, :2])
    size_new = MM @ size_reverse
    size_new = np.array(img.shape)
    M[:, -1] += (size_new - size_reverse) / 2.
    return cv2.warpAffine(img, M, tuple(size_new.astype(int)), borderValue=0.5)


# SIMULATION
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
                    bound[i + 1] -
                    p1 if i < len(bound) - 1 else bound[0] - p1, 3
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

# END SIMULATION
