"""Demo bindings to make sure things work."""
import hydra_python as hydra
import numpy as np
import pathlib


def _get_config_path():
    return pathlib.Path(__file__).absolute().parent.parent / "config"


def _graph_str(G):
    return f"G: #nodes={G.num_nodes()}, #edges={G.num_edges()}"


def main():
    """Test stuff."""
    q = np.array([1.0, 0.0, 0.0, 0.0])
    t = np.zeros((3, 1))
    rgb = np.zeros((480, 640, 3), dtype=np.uint8)
    rgb[:, :, 0] = 255
    rgb[:, :, 1] = 20
    rgb[:, :, 2] = 127
    depth = np.ones((480, 640), dtype=np.float32)

    camera_info = {"fx": 1.0, "fy": 1.0, "cx": 320, "cy": 240}
    camera = hydra.Camera(**camera_info)

    config_path = _get_config_path()
    semantic_file = config_path / "uh2_office_segmentation.csv"

    pipeline = hydra.load_pipeline(config_path, semantic_file)
    pipeline.set_camera(camera)

    print(f"before: {_graph_str(pipeline.graph)}")
    pipeline.step(10, t, q, rgb, depth)
    print(f"after: {_graph_str(pipeline.graph)}")


if __name__ == "__main__":
    main()
