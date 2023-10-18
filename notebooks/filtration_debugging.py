# ---
# jupyter:
#   jupytext:
#     formats: ipynb,py:percent
#     text_representation:
#       extension: .py
#       format_name: percent
#       format_version: '1.3'
#       jupytext_version: 1.13.8
#   kernelspec:
#     display_name: Python 3 (ipykernel)
#     language: python
#     name: python3
# ---

# %%
import json
import matplotlib.pyplot as plt
import seaborn as sns
import numpy as np
import pathlib
import spark_dsg as dsg
import spark_dsg.networkx
import networkx as nx

# %matplotlib notebook

sns.set()
sns.set_style("white")

# %%
INPUT_PATH = "/home/ubuntu/office_filtrations.json"
INPUT_INDEX = -100
DIST = 0.9
MIN_SIZE = 5


# %%
def draw_places(G):
    pos = {}
    colors = []
    for node in places_nx:
        attrs = places_nx.nodes[node]
        pos[node] = attrs["position"][:2]
        colors.append(attrs["distance"])

    fig, ax = plt.subplots()
    nx.draw_networkx(
        places_nx, pos=pos, node_color=colors, with_labels=False, node_size=10, ax=ax
    )
    return fig, ax


def find_threshold(distance_lifetimes, min_dist=0.5, max_dist=1.5, min_lifetime=0.3):
    valid_lifetimes = []
    for lifetime in distance_lifetimes:
        if lifetime[1] < min_dist:
            continue  # disallow components that disappear too fast

        if lifetime[0] > max_dist:
            continue  # disallow components that start too late

        time_valid = lifetime[1] - lifetime[0]
        if time_valid > min_lifetime:
            valid_lifetimes.append(lifetime)

    return valid_lifetimes


def get_connected_components(G, dilation_distance, min_size=1):
    def node_filtration(n1):
        return G.nodes[n1]["distance"] >= dilation_distance

    def edge_filtration(n1, n2):
        if not node_filtration(n1):
            return False

        if not node_filtration(n2):
            return False

        return G[n1][n2]["weight"] >= dilation_distance

    dilated_places = nx.subgraph_view(
        G, filter_node=node_filtration, filter_edge=edge_filtration
    )
    components = nx.connected_components(dilated_places)
    return [set(c) for c in components if len(c) >= min_size]


def draw_components(G, dilation_distance, min_size):
    graph_components = get_connected_components(G, dilation_distance, min_size=min_size)
    colors = sns.color_palette("husl", len(graph_components)) + [(0.0, 0.0, 0.0)]

    pos = {}
    node_colors = []
    for node in places_nx:
        attrs = places_nx.nodes[node]
        pos[node] = attrs["position"][:2]
        best_color = len(graph_components)
        for idx, component in enumerate(graph_components):
            if node in component:
                best_color = idx
                break

        node_colors.append(colors[best_color])

    fig, ax = plt.subplots()
    nx.draw_networkx(
        places_nx, pos=pos, node_color=node_colors, with_labels=False, node_size=10
    )
    return fig, ax


# %%
with pathlib.Path(INPUT_PATH).expanduser().absolute().open("r") as fin:
    results = json.loads(fin.read())

filtration = results["results"][INPUT_INDEX][1]
distances = np.array([x[0] for x in filtration])
components = np.array([x[1] for x in filtration])

barcodes = [x[1] for x in results["barcodes"][INPUT_INDEX][1]]
barcodes = sorted(barcodes, key=lambda x: x[1] - x[0], reverse=True)

points = np.zeros((len(barcodes), 2))
for i, lifetime in enumerate(barcodes):
    points[i, 0] = lifetime[0]
    points[i, 1] = lifetime[1]

min_dist = np.amin(points)
max_dist = np.amax(points)

dsg_path = results["filepaths"][INPUT_INDEX][1]

G = dsg.DynamicSceneGraph.load(dsg_path)
places = G.get_layer(dsg.DsgLayers.PLACES)
places_nx = spark_dsg.networkx.layer_to_networkx(places)

# %%
fig, ax = draw_places(places_nx)
plt.gcf().set_size_inches([9.5, 9.5])
plt.show()

# %%
color = sns.color_palette()[0]

fig, ax = plt.subplots(1, 2)
ax[0].plot(distances, components, marker="+")
ax[0].set_xlabel("Dilation Distance [m]")
ax[0].set_ylabel("Number of Components")
ax[0].vlines(DIST, min(components), max(components), colors="k", linestyle="--")

ax[1].plot([min_dist, max_dist], [min_dist, max_dist], c=color)
ax[1].fill(
    [min_dist, max_dist, min_dist, min_dist],
    [min_dist, max_dist, max_dist, min_dist],
    c=color + (0.3,),
)
ax[1].vlines(DIST, DIST, max_dist, colors="k", linestyle="--")
ax[1].hlines(DIST, min_dist, DIST, colors="k", linestyle="--")
ax[1].scatter(
    points[:, 0],
    points[:, 1],
    color=color,
    marker="o",
    facecolors="none",
    s=20 * np.ones(points.shape[0]),
)
ax[1].set_xlabel("Birth [m]")
ax[1].set_ylabel("Death [m]")
fig.set_size_inches([9.5, 5])

plt.show()

# %%
fig, ax = draw_components(places_nx, DIST, MIN_SIZE)
fig.set_size_inches([9.5, 9.5])
plt.show()

# %%
new_lifetimes = find_threshold(barcodes)

fig, ax = plt.subplots()
ax.hlines(
    np.arange(len(new_lifetimes)),
    [x[0] for x in new_lifetimes],
    [x[1] for x in new_lifetimes],
)
fig.set_size_inches([8, 5])
plt.show()
