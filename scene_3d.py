import numpy as np
import pyvista as pv
from pylie import SE3
from common_lab_utils import PlaneWorldModel

class Scene3D:
    """Visualises the lab in 3D"""

    _do_exit = False
    _current_camera_actors = ()

    def __init__(self, world: PlaneWorldModel):
        """Sets up the 3D viewer"""
        self._plotter = pv.Plotter()
        self._grid_size = world.grid_size

        # Add scene origin and plane.
        width, height = world.world_size
        half_width = 0.5 * width
        half_height = 0.5 * height

        point_bottom_left = [-half_width, -half_height, 0.]
        point_bottom_right = [half_width, -half_height, 0.]
        point_top_left = [-half_width, half_height, 0.]
        point_top_right = [half_width, half_height, 0.]

        rectangle = pv.Rectangle([point_bottom_left, point_bottom_right, point_top_right, point_top_left])
        rectangle.texture_map_to_plane(inplace=True)

        image_rgb = world.world_image[:, :, ::-1].copy()
        tex = pv.numpy_to_texture(image_rgb)

        self._plotter.add_mesh(rectangle, texture=tex, opacity=0.9)
        self._add_axis(SE3(), self._grid_size)

        # Add callback for closing window.
        def exit_callback():
            self._do_exit = True
        self._plotter.add_key_event('q', exit_callback)

        # Show window.
        self._plotter.show(title="3D visualization", interactive_update=True)

    def _add_axis(self, pose: SE3, scale=10.0):
        T = pose.to_matrix()

        point = pv.Sphere(radius=0.1*scale)
        point.transform(T)

        x_arrow = pv.Arrow(direction=(1.0, 0.0, 0.0), scale=scale)
        x_arrow.transform(T)

        y_arrow = pv.Arrow(direction=(0.0, 1.0, 0.0), scale=scale)
        y_arrow.transform(T)

        z_arrow = pv.Arrow(direction=(0.0, 0.0, 1.0), scale=scale)
        z_arrow.transform(T)

        axis_actors = (
            self._plotter.add_mesh(point),
            self._plotter.add_mesh(x_arrow, color='red', render=False),
            self._plotter.add_mesh(y_arrow, color='green', render=False),
            self._plotter.add_mesh(z_arrow, color='blue', render=False)
        )
        return axis_actors

    def _add_frustum(self, pose_w_c, camera_model, image, scale=0.1):
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
            self._plotter.add_mesh(pyramid, show_edges=True, style='wireframe', render=False),
            self._plotter.add_mesh(rectangle, texture=tex, opacity=0.9, render=False)
        )
        return frustum_actors

    def _update_current_camera_visualisation(self, undistorted_frame, pose_w_c: SE3, camera_model):
        # Remove old visualisation.
        for actor in self._current_camera_actors:
            self._plotter.remove_actor(actor, render=False)

        # Render new visualisation.
        self._current_camera_actors = \
            self._add_frustum(pose_w_c, camera_model, undistorted_frame) + self._add_axis(pose_w_c, self._grid_size)

    def update(self, undistorted_frame, pose_w_c: SE3, camera_model, time=10):
        self._update_current_camera_visualisation(undistorted_frame, pose_w_c, camera_model)
        self._plotter.update(time)
        return self._do_exit

    def show(self):
        self._plotter.show()
