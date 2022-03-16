import numpy as np
import pyvista as pv
from pylie import SE3
from common_lab_utils import PlaneWorldModel, PerspectiveCamera
from pose_estimators import PoseEstimate

class Scene3D:
    """Visualises the lab in 3D"""

    _do_exit = False
    _current_camera_actors = ()

    def __init__(self, world_model: PlaneWorldModel, camera_model: PerspectiveCamera):
        """Sets up the 3D viewer"""
        self._grid_length = world_model.grid_length
        self._camera_model = camera_model
        self._plotter = pv.Plotter()

        # Add scene origin and plane.
        half_width = 0.5 * world_model.world_size.width
        half_height = 0.5 * world_model.world_size.height

        point_bottom_left = [-half_width, -half_height, 0.]
        point_bottom_right = [half_width, -half_height, 0.]
        point_top_left = [-half_width, half_height, 0.]
        point_top_right = [half_width, half_height, 0.]

        rectangle = pv.Rectangle([point_bottom_left, point_bottom_right, point_top_right, point_top_left])
        rectangle.texture_map_to_plane(inplace=True)

        image_rgb = world_model.world_image[:, :, ::-1].copy()
        tex = pv.numpy_to_texture(image_rgb)

        self._plotter.add_mesh(rectangle, texture=tex, opacity=0.9)
        add_axis(self._plotter, SE3(), self._grid_length)

        # Add callback for closing window.
        def exit_callback():
            self._do_exit = True
        self._plotter.add_key_event('q', exit_callback)

        # Show window.
        self._plotter.show(title="3D visualization", interactive=True, interactive_update=True)

    def _update_current_camera_visualisation(self, undistorted_frame, estimate: PoseEstimate):
        # Remove old visualisation.
        for actor in self._current_camera_actors:
            self._plotter.remove_actor(actor, render=False)

        # Render new visualisation.
        if estimate.is_found():
            self._current_camera_actors = \
                add_frustum(self._plotter, estimate.pose_w_c, self._camera_model, undistorted_frame) + \
                add_axis(self._plotter, estimate.pose_w_c, self._grid_length)

    def update(self, undistorted_frame, estimate: PoseEstimate, time=10):
        self._update_current_camera_visualisation(undistorted_frame, estimate)
        self._plotter.update(time)
        return self._do_exit

    def show(self):
        self._plotter.show()


class ARRenderer:
    """Renders the 3D world scene in camera perspective"""

    def __init__(self, world_model: PlaneWorldModel, camera_model: PerspectiveCamera, hide_rendering=True):
        """Sets up the 3D viewer"""

        # Set up plotter.
        # Set hide_rendering=False to show a window with the 3D rendering.
        theme = pv.themes.DefaultTheme()
        theme.transparent_background = True
        self._plotter = pv.Plotter(theme=theme, off_screen=hide_rendering)

        # Set camera pose.
        self._plotter.camera.position = (0., 0., 0.)
        self._plotter.camera.focal_point = (0., 0., 1.)
        self._plotter.camera.up = (0., -1., 0.)

        # Set principal point.
        p_u = camera_model.calibration_matrix[0, 2]
        p_v = camera_model.calibration_matrix[1, 2]
        img_width = camera_model.image_size.width
        img_height = camera_model.image_size.height
        self._plotter.camera.SetWindowCenter((-2 * p_u) / img_width + 1, (2 * p_v) / img_height - 1)

        # Set focal length.
        f_v = camera_model.calibration_matrix[1, 1]
        view_angle = 180.0 / np.pi * (2.0 * np.arctan2(img_height / 2.0, f_v))
        self._plotter.camera.view_angle = view_angle

        # Add AR object.
        # Add your own objects if you want to!
        add_axis(self._plotter, SE3(), world_model.grid_length)

        # Add a light
        self._plotter.add_light(pv.Light(light_type='scene light', position=(0, 0, 5)))

        # Show window.
        self._plotter.show(title="AR visualization", window_size=[img_width, img_height],
                           interactive=False, interactive_update=True)

    def update(self, estimate: PoseEstimate):
        if not estimate.is_found():
            return None, None

        self._plotter.camera.model_transform_matrix = estimate.pose_w_c.inverse().to_matrix()

        _, ar_rendering = self._plotter.show(return_cpos=True, return_img=True, screenshot=True,
                                             interactive_update=True)
        ar_rendering_bgr = ar_rendering[:, :, 2::-1]
        foreground_mask = ar_rendering[:, :, -1] > 0

        return ar_rendering_bgr, foreground_mask


def add_axis(plotter, pose: SE3, scale=10.0):
    T = pose.to_matrix()

    point = pv.Sphere(radius=0.1 * scale)
    point.transform(T)

    x_arrow = pv.Arrow(direction=(1.0, 0.0, 0.0), scale=scale)
    x_arrow.transform(T)

    y_arrow = pv.Arrow(direction=(0.0, 1.0, 0.0), scale=scale)
    y_arrow.transform(T)

    z_arrow = pv.Arrow(direction=(0.0, 0.0, 1.0), scale=scale)
    z_arrow.transform(T)

    axis_actors = (
        plotter.add_mesh(point),
        plotter.add_mesh(x_arrow, color='red', render=False),
        plotter.add_mesh(y_arrow, color='green', render=False),
        plotter.add_mesh(z_arrow, color='blue', render=False)
    )
    return axis_actors


def add_frustum(plotter, pose_w_c, camera_model, image, scale=0.1):
    S = pose_w_c.to_matrix() @ np.diag([scale, scale, scale, 1.0])

    img_height, img_width = image.shape[:2]

    point_bottom_left = np.squeeze(camera_model.pixel_to_normalised(np.array([img_width-1., img_height-1.])))
    point_bottom_right = np.squeeze(camera_model.pixel_to_normalised(np.array([0., img_height-1.])))
    point_top_left = np.squeeze(camera_model.pixel_to_normalised(np.array([0., 0.])))
    point_top_right = np.squeeze(camera_model.pixel_to_normalised(np.array([img_width-1., 0.])))

    point_focal = np.zeros([3])

    pyramid = pv.Pyramid([point_bottom_left, point_bottom_right, point_top_left, point_top_right, point_focal])
    pyramid.transform(S)

    rectangle = pv.Rectangle([point_bottom_left, point_bottom_right, point_top_left, point_top_right])
    rectangle.texture_map_to_plane(inplace=True)
    rectangle.transform(S)

    image_flipped_rgb = image[::-1, :, ::-1].copy()
    tex = pv.numpy_to_texture(image_flipped_rgb)

    frustum_actors = (
        plotter.add_mesh(pyramid, show_edges=True, style='wireframe', render=False),
        plotter.add_mesh(rectangle, texture=tex, opacity=0.9, render=False)
    )
    return frustum_actors