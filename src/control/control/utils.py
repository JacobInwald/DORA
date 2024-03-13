import numpy as np
import cv2

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


def crop_new(arr, threshold=0.5, deviation=0.05):
    """
    Crops the given array to remove the empty space around it.

    Params:
        arr (np.ndarray): The input array.

    Returns:
        np.ndarray: The cropped array.
    """
    mask1 = arr <= (threshold - deviation )
    mask2 = arr >= (threshold + deviation)
    mask = np.logical_or(mask1, mask2)
    rows = np.flatnonzero((mask).sum(axis=1))
    cols = np.flatnonzero((mask).sum(axis=0))
    return arr[rows.min():rows.max() + 1, cols.min():cols.max() + 1]


def rotate_image(img, angle):
    size_reverse = np.array(img.shape[1::-1]) # swap x with y
    M = cv2.getRotationMatrix2D(tuple(size_reverse / 2.), angle, 1.)
    MM = np.absolute(M[:,:2])
    size_new = MM @ size_reverse
    size_new = np.array(img.shape)
    M[:,-1] += (size_new - size_reverse) / 2.
    return cv2.warpAffine(img, M, tuple(size_new.astype(int)), borderValue=0.5)