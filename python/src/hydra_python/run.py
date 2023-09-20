"""Module containing code for running Hydra."""
from scipy.spatial.transform import Rotation as R
import numpy as np
import click


class ImageVisualizer:
    """GUI for showing images."""

    def __init__(self):
        """Initialize pyqtgraph."""
        # TODO(nathan) make a grid layout
        import pyqtgraph.multiprocess as mp

        proc = mp.QtProcess(processRequests=False)
        self._pg = proc._import("pyqtgraph")
        self._pg.setConfigOptions(imageAxisOrder="row-major")
        self._view = None

    def show(self, image, is_depth=False):
        """Show an image."""
        kwargs = {"autoLevels": False, "levels": (0, 255)} if not is_depth else {}
        if self._view is None:
            self._view = self._pg.image(image, **kwargs)
            if is_depth:
                self._view.setPredefinedGradient("viridis")
        else:
            self._view.setImage(image, **kwargs)


def hydra_output_callback(pipeline, visualizer):
    """Show graph."""
    if visualizer:
        visualizer.update_graph(pipeline.graph)


def _take_step(pipeline, data, pose, segmenter, image_viz):
    timestamp, world_t_body, q_wxyz = pose
    q_xyzw = np.roll(q_wxyz, -1)

    world_T_body = np.eye(4)
    world_T_body[:3, 3] = world_t_body
    world_T_body[:3, :3] = R.from_quat(q_xyzw).as_matrix()
    data.set_pose(timestamp, world_T_body)

    labels = segmenter(data.rgb) if segmenter else data.labels
    if image_viz:
        image_viz.show(data.colormap(labels))

    pipeline.step(timestamp, world_t_body, q_wxyz, data.depth, labels, data.rgb)


def run(
    pipeline,
    data,
    pose_source,
    segmenter=None,
    visualizer=None,
    show_images=False,
    show_progress=True,
    step_callback=hydra_output_callback,
):
    """Do stuff."""
    image_viz = ImageVisualizer() if show_images else None

    if show_progress:
        with click.progressbar(pose_source) as bar:
            for pose in bar:
                _take_step(pipeline, data, pose, segmenter, image_viz)
                if step_callback:
                    step_callback(pipeline, visualizer)
    else:
        for pose in pose_source:
            _take_step(pipeline, data, pose, segmenter, image_viz)
            if step_callback:
                step_callback(pipeline, visualizer)
