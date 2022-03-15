import numpy as np

from common_lab_utils import CameraModel


def test_principal_point():
    camera_model = CameraModel(np.eye(3))
    np.testing.assert_almost_equal(camera_model.principal_point, (0, 0))


def test_focal_lengths():
    camera_model = CameraModel(np.eye(3))
    np.testing.assert_almost_equal(camera_model.focal_lengths, (1, 1))
