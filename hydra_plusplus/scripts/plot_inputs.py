#!/usr/bin/env python3
"""Node that runs crisp on received masks and RGBD data with paper-quality visualization."""

import pathlib

import click
import matplotlib.pyplot as plt
import numpy as np
import torch
import viser


def depth_to_pointcloud(depth, intrinsics, mask=None):
    """Convert depth image to point cloud in camera frame.

    Args:
        depth: (H, W) depth image in meters
        intrinsics: (3, 3) camera intrinsic matrix
        mask: optional (H, W) mask to filter points

    Returns:
        points: (N, 3) point cloud in camera coordinates
        colors: (N, 3) RGB colors (depth-based colormap)
    """
    if isinstance(depth, torch.Tensor):
        depth = depth.cpu().numpy()
    if isinstance(intrinsics, torch.Tensor):
        intrinsics = intrinsics.cpu().numpy()
    if mask is not None and isinstance(mask, torch.Tensor):
        mask = mask.cpu().numpy()

    h, w = depth.shape
    fx, fy = intrinsics[0, 0], intrinsics[1, 1]
    cx, cy = intrinsics[0, 2], intrinsics[1, 2]

    # Create pixel grid
    u, v = np.meshgrid(np.arange(w), np.arange(h))

    # Apply mask if provided
    if mask is not None:
        valid = (depth > 0) & (mask > 0)
    else:
        valid = depth > 0

    z = depth[valid]
    x = (u[valid] - cx) * z / fx
    y = (v[valid] - cy) * z / fy

    points = np.stack([x, y, z], axis=1)

    # Color by depth (blue=near, red=far)
    z_norm = (z - z.min()) / (z.max() - z.min() + 1e-6)
    colors = plt.cm.viridis(z_norm)[:, :3]

    return points, colors


def create_rgb_with_mask_overlay(rgb, mask, alpha=0.4):
    """Create RGB image with semi-transparent mask overlay.

    Args:
        rgb: (H, W, 3) RGB image (uint8 or float)
        mask: (H, W) binary mask
        alpha: overlay transparency

    Returns:
        overlay: (H, W, 3) RGB image with mask overlay
    """
    if isinstance(rgb, torch.Tensor):
        rgb = rgb.cpu().numpy()
    if isinstance(mask, torch.Tensor):
        mask = mask.cpu().numpy()

    # Ensure RGB is in [0, 255] range
    if rgb.max() <= 1.0:
        rgb = (rgb * 255).astype(np.uint8)
    else:
        rgb = rgb.astype(np.uint8)

    # Create overlay
    overlay = rgb.copy()
    mask_bool = mask > 0

    # Green overlay for mask region
    overlay[mask_bool, 0] = np.clip(overlay[mask_bool, 0] * (1 - alpha), 0, 255)
    overlay[mask_bool, 1] = np.clip(
        overlay[mask_bool, 1] * (1 - alpha) + 255 * alpha, 0, 255
    )
    overlay[mask_bool, 2] = np.clip(overlay[mask_bool, 2] * (1 - alpha), 0, 255)

    return overlay


def visualize(rgb, depth, mask, intrinsics, port=8080):
    """Launch viser server for interactive 3D visualization."""
    server = viser.ViserServer(port=port)
    click.secho(f"Viser started at http://localhost:{port}", fg="green", bold=True)
    depth_points, depth_colors = depth_to_pointcloud(depth, intrinsics, mask)

    server.scene.add_frame(
        "/camera",
        wxyz=(1, 0, 0, 0),
        position=(0, 0, 0),
        axes_length=0.1,
        axes_radius=0.005,
    )

    server.scene.add_point_cloud(
        "/depth_pointcloud",
        points=depth_points.astype(np.float32),
        colors=depth_colors.astype(np.float32),
        point_size=0.01,
        point_shape="circle",
    )

    click.secho("\nVisualization Controls:", fg="cyan")
    click.secho("  - Left click + drag: Rotate view", fg="white")
    click.secho("  - Right click + drag: Pan view", fg="white")
    click.secho("  - Scroll: Zoom in/out", fg="white")
    click.secho("\nPress Ctrl+C to exit...", fg="yellow")
    return server


@click.command()
@click.argument("input_path", type=click.Path(exists=True))
@click.option(
    "--use-viser/--no-use-viser", default=True, help="run viser visualization"
)
@click.option("--viser-port", default=8080, type=int, help="Port for viser server")
def main(input_path, use_viser, viser_port):
    """Run CRISP on images with paper-quality visualization."""
    input_path = pathlib.Path(input_path).expanduser().resolve()
    tensors = torch.load(input_path, weights_only=False)

    rgb = tensors["rgb"]
    depth = tensors["depth"]
    mask = tensors["mask"]
    intrinsics = tensors["intrinsics"]
    with np.printoptions(suppress=True):
        print(f"Intrinsics:\n{intrinsics.numpy()}")

    fig, ax = plt.subplots(1, 3, figsize=(15, 5))
    ax[0].imshow(rgb)
    ax[0].set_title("RGB")
    ax[1].imshow(depth)
    ax[1].set_title("Depth")
    ax[2].imshow(mask, cmap="gray")
    ax[2].set_title("Mask")
    plt.tight_layout()
    plt.show()

    if use_viser:
        visualize(rgb, depth, mask, intrinsics, port=viser_port)

        try:
            while True:
                pass
        except KeyboardInterrupt:
            click.secho("\nShutting down viser server...", fg="yellow")


if __name__ == "__main__":
    main()
