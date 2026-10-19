# ---
# jupyter:
#   jupytext:
#     formats: ipynb,py:percent
#     text_representation:
#       extension: .py
#       format_name: percent
#       format_version: '1.3'
#       jupytext_version: 1.11.5
#   kernelspec:
#     display_name: Python 3
#     language: python
#     name: python3
# ---

# %% [markdown]
# # Traversability Places Benchmark
#
# Computes per run:
# 1. Number of connected components in the traversability places layer.
# 2. Percent of the trajectory (by path length) covered by traversability places.
#
# The only input is a Hydra output directory; the trajectory is read from the agent
# layer of the same scene graph.

# %%
import pathlib

import matplotlib.pyplot as plt
import networkx as nx
import numpy as np
import pandas as pd
import spark_dsg
from spark_dsg.networkx import layer_to_networkx

# %%
# Map of run name to Hydra output directory.
RUNS = {
    # "baseline": "~/hydra_output/baseline",
    # "footprint": "~/hydra_output/footprint",
}
# Which saved graph to evaluate: "backend" or "frontend".
WHICH = "backend"
# Trajectory resampling resolution [m] for coverage computation.
STEP = 0.05


# %%
def load_graph(run_dir, which="backend"):
    """Load the saved scene graph (without mesh) from a Hydra output directory."""
    path = pathlib.Path(run_dir).expanduser() / which / "dsg.json"
    return spark_dsg.DynamicSceneGraph.load(str(path))


def num_components(G, layer=spark_dsg.DsgLayers.TRAVERSABILITY):
    """Count the connected components of a places layer (0 if the layer is empty)."""
    if not G.has_layer(layer):
        return 0

    G_nx = layer_to_networkx(G.get_layer(layer))
    return nx.number_connected_components(G_nx) if len(G_nx) > 0 else 0


def extract_trajectory(G, prefix=None):
    """Get the (N, 2) xy positions of agent poses ordered by timestamp.

    Uses the agent partition with the given prefix, or the first one found.
    """
    agents_id = G.get_layer_key(spark_dsg.DsgLayers.AGENTS).layer
    for layer in G.layer_partitions:
        if layer.id != agents_id:
            continue

        nodes = list(layer.nodes)
        if not nodes:
            continue
        if prefix is not None and nodes[0].id.category != prefix:
            continue

        nodes = sorted(nodes, key=lambda n: n.attributes.timestamp)
        return np.array([n.attributes.position[:2] for n in nodes])

    return np.zeros((0, 2))


def extract_boxes(G, layer=spark_dsg.DsgLayers.TRAVERSABILITY):
    """Get (M, 2) min and max world-frame corners of all place boundaries."""
    if not G.has_layer(layer):
        return np.zeros((0, 2)), np.zeros((0, 2))

    mins, maxs = [], []
    for node in G.get_layer(layer).nodes:
        attrs = node.attributes
        # Boundaries are stored relative to the node position.
        center = np.asarray(attrs.position)[:2]
        mins.append(center + np.asarray(attrs.boundary.min))
        maxs.append(center + np.asarray(attrs.boundary.max))

    return np.array(mins).reshape(-1, 2), np.array(maxs).reshape(-1, 2)


def path_coverage(traj, mins, maxs, step=0.05):
    """Fraction of path length of traj (N, 2) that lies inside any box [mins, maxs].

    Each segment is resampled at segment midpoints spaced at most `step` apart; each
    sample carries an equal share of the segment length. Returns NaN for empty paths.
    """
    if len(traj) < 2:
        return np.nan

    points, weights = [], []
    for start, end in zip(traj[:-1], traj[1:]):
        length = np.linalg.norm(end - start)
        if length == 0.0:
            continue
        n = int(np.ceil(length / step))
        ratios = (np.arange(n) + 0.5) / n
        points.append(start + ratios[:, None] * (end - start))
        weights.append(np.full(n, length / n))

    if not points:
        return np.nan

    points = np.concatenate(points)
    weights = np.concatenate(weights)
    if len(mins) == 0:
        return 0.0

    inside = ((points[:, None] >= mins) & (points[:, None] <= maxs)).all(-1).any(-1)
    return weights[inside].sum() / weights.sum()


def path_length(traj):
    return np.linalg.norm(np.diff(traj, axis=0), axis=1).sum() if len(traj) > 1 else 0.0


def evaluate(G, step=0.05):
    traj = extract_trajectory(G)
    mins, maxs = extract_boxes(G)
    return {
        "num_places": len(mins),
        "num_components": num_components(G),
        "path_length": path_length(traj),
        "coverage": path_coverage(traj, mins, maxs, step=step),
    }


# %% [markdown]
# ## Sanity checks

# %%
_traj = np.array([[0.0, 0.5], [1.0, 0.5]])
_box = (np.array([[0.0, 0.0]]), np.array([[1.0, 1.0]]))
assert np.isclose(path_coverage(_traj, *_box), 1.0)
assert np.isclose(path_coverage(_traj + [0.5, 0.0], *_box), 0.5)
assert np.isclose(path_coverage(_traj + [5.0, 0.0], *_box), 0.0)
assert np.isnan(path_coverage(_traj[:1], *_box))

# %% [markdown]
# ## Evaluate runs

# %%
results = pd.DataFrame(
    [{"name": name, **evaluate(load_graph(path, WHICH), STEP)} for name, path in RUNS.items()]
)
results

# %%
if len(results) > 0:
    fig, ax = plt.subplots(1, 2, figsize=(10, 4))
    results.plot.bar(x="name", y="num_components", ax=ax[0], legend=False)
    ax[0].set_ylabel("# connected components")
    results.plot.bar(x="name", y="coverage", ax=ax[1], legend=False)
    ax[1].set_ylabel("trajectory coverage")
    ax[1].set_ylim(0, 1)
    fig.tight_layout()
    plt.show()
