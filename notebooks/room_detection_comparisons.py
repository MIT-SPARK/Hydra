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
import networkx.algorithms.community as nx_comm
import networkx.readwrite as nxio
import matplotlib.patches as mpatches
import matplotlib.pyplot as plt
import scipy
import seaborn as sns
import numpy as np
import pathlib
import sklearn.cluster
import json


# %%
node_size = 30
scene_num = 5
dsg_dir = "/home/ubuntu/catkin_ws/src/hydra_dsg_builder/hydra_dsg_builder/output"
dsg_file = f"goseek_scene{scene_num}_dsg.json"
dsg_path = pathlib.Path(dsg_dir) / dsg_file

# %%
with open(dsg_path, "r") as fin:
    data = json.loads(fin.read())

# %%
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


# %%
def get_id_parts(node_id):
    key_mask = 0xFF << 56
    key = chr((node_id & key_mask) >> 56)
    idx = node_id & ~key_mask

    return key, idx


def get_num_nodes(graph, prefix):
    num_rooms = 0
    for node in graph:
        if get_id_parts(node)[0] == prefix:
            num_rooms += 1
    return num_rooms


# %%
print(f"Rooms: {get_num_nodes(G, 'R')}")
print(f"Places: {get_num_nodes(G, 'p')}")
print(f"Objects: {get_num_nodes(G, 'O')}")

room_parents = {}
for node in G:
    node_parts = get_id_parts(node)
    if node_parts[0] != "R":
        continue

    for other_node in G.adj[node]:
        if get_id_parts(other_node)[0] != "p":
            continue

        room_parents[other_node] = node


# %%
def is_place_node(n1):
    return get_id_parts(n1)[0] == "p"


def is_place_edge(n1, n2):
    return get_id_parts(n1)[0] == "p" and get_id_parts(n2)[0] == "p"


places = nx.subgraph_view(G, filter_node=is_place_node, filter_edge=is_place_edge)
positions = {p: np.array(places.nodes[p]["attributes"]["position"][:2]) for p in places}

# %% [markdown]
# ## Original Rooms
# Plot of top-down view of original room segmentation (from offline DSG builder)
# overlaid on places layer


# %%
num_rooms = get_num_nodes(G, "R")
cmap = sns.color_palette("husl", num_rooms)
colors = []
for p in places:
    if p in room_parents:
        colors.append(cmap[get_id_parts(room_parents[p])[1]])
    else:
        colors.append((0.0, 0.0, 0.0))

nx.draw(places, positions, node_size=node_size, node_color=colors)
plt.gcf().set_size_inches([12, 12])

elements = [
    mpatches.Patch(facecolor=cmap[i], edgecolor="k", label=f"Room {i}")
    for i in range(num_rooms)
]
elements.append(mpatches.Patch(facecolor="k", edgecolor="k", label="Unkown"))
plt.gca().legend(handles=elements, loc="lower left")
plt.show()


# %% [markdown]
# ## Modularity-Based Clustering
# See networkx greedy_modularity_communities for details


# %%
for edge in places.edges():
    G[edge[0]][edge[1]]["min_distance"] = G[edge[0]][edge[1]]["info"]["weight"]

clusters = nx_comm.greedy_modularity_communities(places, weight="min_distance")
place_clusters = {}
for index, cluster in enumerate(clusters):
    for node in cluster:
        place_clusters[node] = index

cmap = sns.color_palette("husl", len(clusters))
colors = []
for p in places:
    if p in place_clusters:
        colors.append(cmap[place_clusters[p]])
    else:
        colors.append((0.0, 0.0, 0.0))

nx.draw(places, positions, node_size=node_size, node_color=colors)
plt.gcf().set_size_inches([12, 12])

elements = [
    mpatches.Patch(facecolor=cmap[i], edgecolor="k", label=f"Room {i}")
    for i in range(len(clusters))
]
elements.append(mpatches.Patch(facecolor="k", edgecolor="k", label="Unkown"))
plt.gca().legend(handles=elements, loc="lower left")
plt.show()


# %% [markdown]
# ## Spectral Clustering
# Computes clusters via kmeans on $v$, where $$\lambda, v = eig(L)$$ and $$L = A - D$$
#
# Number of clusters is estimated by the largest gap in sorted eigenvalues,
# capped to the first N eigenvalues, where $N = 30$
#
# See https://arxiv.org/pdf/0711.0189.pdf for a thorough (but complicated tutorial)

# %%
N = 30
L = nx.laplacian_matrix(places, weight="min_distance").toarray()
w, v = scipy.linalg.eigh(L)

diffs = np.diff(w[:N])
best_idx = np.argmax(diffs)
num_clusters = best_idx + 1

plt.plot(w[:N], marker="o", label="Eigenvalues")
limits = plt.gca().get_ylim()
plt.vlines(
    num_clusters, limits[0], limits[1], color="k", linestyle="--", label="Best Cluster"
)
plt.legend()
plt.gcf().set_size_inches([10, 4])
plt.show()


# %%
num_clusters = 12
print(f"Clusters: {num_clusters}")
X = v[:, :num_clusters]
labels = sklearn.cluster.KMeans(n_clusters=num_clusters).fit_predict(X)
print(len(labels), X.shape)


# %%
cmap = sns.color_palette("husl", num_clusters)
colors = []
for idx, _ in enumerate(places):
    colors.append(cmap[labels[idx]])

nx.draw(places, positions, node_size=node_size, node_color=colors)
plt.gcf().set_size_inches([12, 12])

elements = [
    mpatches.Patch(facecolor=cmap[i], edgecolor="k", label=f"Room {i}")
    for i in range(num_clusters)
]
plt.gca().legend(handles=elements, loc="lower left")
plt.show()


# %% [markdown]
# ## Places Dilation and Connected Components
# "inflates" obstacles by some fixed distance by removing all place nodes and edges with
# distances smaller than the inflation distance. Computes the connected components for
# the new graph and will do something to associate missing places to parent rooms


# %%
def is_place_node_inflated(n1, distance):
    if get_id_parts(n1)[0] != "p":
        return False

    if G.nodes[n1]["attributes"]["distance"] < distance:
        return False

    return True


def is_place_edge_inflated(n1, n2, distance):
    if not is_place_node_inflated(n1, distance):
        return False

    if not is_place_node_inflated(n1, distance):
        return False

    return G[n1][n2]["min_distance"] > distance


inflation_distance = 1.25
inflated_places = nx.subgraph_view(
    G,
    filter_node=lambda x: is_place_node_inflated(x, inflation_distance),
    filter_edge=lambda x, y: is_place_edge_inflated(x, y, inflation_distance),
)
inflated_positions = {
    p: np.array(places.nodes[p]["attributes"]["position"][:2]) for p in places
}
nx.draw(inflated_places, inflated_positions, node_size=node_size)
plt.gcf().set_size_inches([12, 12])
plt.show()


# %%
components = nx.connected_components(inflated_places)

component_clusters = {}
num_components = 0
for idx, component in enumerate(components):
    num_components += 1
    for place in component:
        component_clusters[place] = idx

cmap = sns.color_palette("husl", num_components)
colors = []
for p in places:
    if p in component_clusters:
        colors.append(cmap[component_clusters[p]])
    else:
        colors.append((0.0, 0.0, 0.0))

nx.draw(places, positions, node_size=node_size, node_color=colors)
plt.gcf().set_size_inches([12, 12])

elements = [
    mpatches.Patch(facecolor=cmap[i], edgecolor="k", label=f"Room {i}")
    for i in range(num_components)
]
elements.append(mpatches.Patch(facecolor="k", edgecolor="k", label="Unkown"))
plt.gca().legend(handles=elements, loc="lower left")
plt.show()


# %%
def get_connected_components(G, inflation_distance, min_size=1):
    inflated_places = nx.subgraph_view(
        G,
        filter_node=lambda x: is_place_node_inflated(x, inflation_distance),
        filter_edge=lambda x, y: is_place_edge_inflated(x, y, inflation_distance),
    )
    components = nx.connected_components(inflated_places)
    return sum(len(component) >= min_size for component in components)


# %%
M = 50
min_size = 5
inflation_distances = np.linspace(0.0, 3.0, M)
component_sizes = [
    get_connected_components(G, d, min_size=min_size) for d in inflation_distances
]
plt.plot(inflation_distances, component_sizes)
plt.xlabel("Dilation Distance [m]")
plt.ylabel("Number of Components")
plt.show()

# %%
