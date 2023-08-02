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

# %%
import networkx as nx
import networkx.readwrite as nxio
import pathlib
import json

import seaborn as sns
import matplotlib.patches as mpatches
import matplotlib.pyplot as plt

import scipy
import sklearn.cluster
import numpy as np


# %%
def get_id_parts(node_id):
    """Get human readable components from a node key."""
    key_mask = 0xFF << 56
    key = chr((node_id & key_mask) >> 56)
    idx = node_id & ~key_mask

    return key, idx


def node_matches_prefix(node_id, prefix):
    """Check if an edge matches a prefix."""
    return get_id_parts(node_id)[0] == prefix


def edge_matches_prefix(edge, prefix):
    """Check if an edge matches a prefix."""
    return node_matches_prefix(edge[0], prefix) and node_matches_prefix(edge[1], prefix)


def load_graph(dsg_dir, scene_num, places_prefix="p"):
    """Open and read graph."""
    dsg_file = f"goseek_scene{scene_num}_dsg.json"
    dsg_path = pathlib.Path(dsg_dir) / dsg_file

    with open(dsg_path, "r") as fin:
        data = json.loads(fin.read())

    G = nxio.node_link_graph(
        data,
        attrs={
            "source": "source",
            "target": "target",
            "name": "id",
            "key": "key",
            "link": "edges",
        },
    )

    for node in G:
        if node_matches_prefix(node, places_prefix):
            G.nodes[node]["min_distance"] = G.nodes[node]["attributes"]["distance"]

    for edge in G.edges():
        if edge_matches_prefix(edge, places_prefix):
            G[edge[0]][edge[1]]["min_distance"] = G[edge[0]][edge[1]]["info"]["weight"]

    return G


def get_spectral_clusters(G, k, weight_key="min_distance", use_normalized=False):
    if use_normalized:
        L = nx.normalized_laplacian_matrix(G, weight=weight_key).toarray()
    else:
        L = nx.laplacian_matrix(G, weight=weight_key).toarray()
    w, v = scipy.linalg.eigh(L)

    X = v[:, :k]

    kmeans = sklearn.cluster.KMeans(n_clusters=k)
    clusters = kmeans.fit_predict(X)
    print(f"Unseeded number of iterations: {kmeans.n_iter_}")
    return clusters


def get_connected_components(G, dilation_distance, min_size=1):
    def node_filtration(n1):
        return G.nodes[n1]["min_distance"] >= dilation_distance

    def edge_filtration(n1, n2):
        if not node_filtration(n1):
            return False

        if not node_filtration(n2):
            return False

        return G[n1][n2]["min_distance"] >= dilation_distance

    dilated_places = nx.subgraph_view(
        G, filter_node=node_filtration, filter_edge=edge_filtration
    )
    components = nx.connected_components(dilated_places)
    return [c for c in components if len(c) >= min_size]


def draw_labels(ax, G, labels, places_prefix="p", **kwargs):
    k = max(labels) + 1
    cmap = sns.color_palette("husl", k)
    colors = []
    for idx, _ in enumerate(places):
        colors.append(cmap[labels[idx]])

    positions = {node: G.nodes[node]["attributes"]["position"][:2] for node in G}

    nx.draw(G, positions, ax, node_color=colors, **kwargs)

    elements = [
        mpatches.Patch(facecolor=cmap[i], edgecolor="k", label=f"Room {i}")
        for i in range(k)
    ]
    ax.legend(handles=elements, loc="lower left")


def get_homology_seeded_clusters(
    G, distance, min_size=1, weight_key="min_distance", use_normalized=False
):
    connected_components = get_connected_components(G, distance, min_size=min_size)
    k = len(connected_components)

    if use_normalized:
        L = nx.normalized_laplacian_matrix(G, weight=weight_key).toarray()
    else:
        L = nx.laplacian_matrix(G, weight=weight_key).toarray()

    w, v = scipy.linalg.eigh(L)

    X = v[:, :k]

    node_index_mapping = {node: index for index, node in enumerate(G)}

    initial_means = np.zeros((k, k))
    for index, component in enumerate(connected_components):
        for node in component:
            initial_means[index, :] += X[node_index_mapping[node], :]
        initial_means[index, :] /= len(component)

    kmeans = sklearn.cluster.KMeans(n_clusters=k, init=initial_means)
    labels = kmeans.fit_predict(X)
    print(f"Homology seeded number of iterations: {kmeans.n_iter_}")
    return labels


# %%
node_size = 30
scene_num = 3
places_prefix = "p"
dsg_dir = "/home/ubuntu/catkin_ws/src/hydra_dsg_builder/hydra_dsg_builder/output"

G = load_graph(dsg_dir, scene_num, places_prefix=places_prefix)
places = nx.subgraph_view(
    G,
    filter_node=lambda x: node_matches_prefix(x, places_prefix),
    filter_edge=lambda x, y: edge_matches_prefix((x, y), places_prefix),
)


# %%
M = 50
min_size = 5
dilation_distances = np.linspace(0.3, 2.0, M)
component_sizes = [
    len(get_connected_components(places, d, min_size=min_size))
    for d in dilation_distances
]

fig, ax = plt.subplots(1, 2)
ax[0].plot(dilation_distances, component_sizes)
ax[0].set_xlabel("Dilation Distance [m]")
ax[0].set_ylabel("Number of Components")
ax[0].set_title("Connected Components vs Dilation")

sns.histplot(component_sizes, ax=ax[1])
ax[1].set_title("Number of Connected Components Frequency")

fig.set_size_inches([12, 4])
plt.show()

# %%
k = 12
labels = get_spectral_clusters(places, k)

fig, ax = plt.subplots()
draw_labels(ax, places, labels, node_size=node_size)
fig.set_size_inches([12, 12])
plt.show()

# %%
cutoff = np.argmax(component_sizes)
sequences = []
curr_sequence = [component_sizes[0], 1, 0]
for i in range(1, cutoff + 1):
    if curr_sequence[0] == component_sizes[i]:
        curr_sequence[1] += 1
    else:
        sequences.append(curr_sequence)
        curr_sequence = [component_sizes[i], 1, i]
sequences.append(curr_sequence)  # we don't care if this is duplicated
print(sequences)
best_sequence = max(sequences, key=lambda x: x[1])
print(best_sequence)

labels = get_homology_seeded_clusters(
    places, dilation_distances[best_sequence[2]], min_size=min_size
)

fig, ax = plt.subplots()
draw_labels(ax, places, labels, node_size=node_size)
fig.set_size_inches([12, 12])
plt.show()
