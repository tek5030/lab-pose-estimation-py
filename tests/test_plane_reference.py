import numpy as np

from common_lab_utils import PlaneReference, Size


def test_one_pixel():
    ref = PlaneReference(Size(10, 10), Size(10, 10))
    pixel = np.array((1, 1))
    np.testing.assert_almost_equal( ref.pixel_to_world(pixel), [[1, 1, 0]])


def test_many_pixel():
    ref = PlaneReference(Size(10, 10), Size(10, 10))
    pixels = np.array([
        [0, 0],
        [1, 1],
        [2, 2]
    ])
    np.testing.assert_almost_equal( ref.pixel_to_world(pixels), [[0, 0, 0], [1, 1, 0], [2, 2, 0]])
