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
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import seaborn as sns
import pandas as pd
import numpy as np
import pathlib
import random
import json

# %matplotlib notebook


# %%
def get_id_parts(node_id):
    """Get human readable components from a node key."""
    key_mask = 0xFF << 56
    key = chr((node_id & key_mask) >> 56)
    idx = node_id & ~key_mask

    return key, idx


def make_id(prefix, index):
    """Make a node key from the node parts."""
    return (ord(prefix) << 56) | (~(0xFF << 56) & index)


def node_matches_prefix(node_id, prefix):
    """Check if an edge matches a prefix."""
    return get_id_parts(node_id)[0] == prefix


def edge_matches_prefix(edge, prefix):
    """Check if an edge matches a prefix."""
    return node_matches_prefix(edge[0], prefix) and node_matches_prefix(edge[1], prefix)


def get_num_nodes(graph, prefix):
    """Get the number of nodes in a layer."""
    return sum((1 for node in graph if node_matches_prefix(node, prefix)))


def make_label(node_id):
    """Get string from node key."""
    key, idx = get_id_parts(node_id)
    return f"{key}({idx})"


def load_graph(dsg_path, places_prefix="p"):
    """Open and read graph."""
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


def get_original_colors(G):
    """Get colors directly from graph."""
    return [
        [float(v) / 255.0 for v in G.nodes[node]["attributes"]["color"]]
        for node in G
        if "color" in G.nodes[node]["attributes"]
    ]


def draw_graph(ax, G, color_func=get_original_colors, **kwargs):
    """Draw the graph."""
    positions = {node: G.nodes[node]["attributes"]["position"][:2] for node in G}
    colors = []
    if color_func:
        colors = color_func(G)

    if len(colors) > 0:
        nx.draw(G, positions, ax, node_color=colors, **kwargs)
    else:
        nx.draw(G, positions, ax, **kwargs)


# %%
node_size = 30
dsg_dir = "/home/ubuntu/catkin_ws/src/hydra_dsg_builder/hydra_dsg_builder/output"
dsg_file = "backend_dsg.json"
dsg_path = pathlib.Path(dsg_dir) / dsg_file
places_prefix = "p"
objects_prefix = "O"
rooms_prefix = "R"

G = load_graph(dsg_path, places_prefix)


# %%
print(f"Rooms: {get_num_nodes(G, 'R')}")
print(f"Places: {get_num_nodes(G, 'p')}")
print(f"Objects: {get_num_nodes(G, 'O')}")


# %%
places = nx.subgraph_view(
    G,
    filter_node=lambda x: node_matches_prefix(x, places_prefix),
    filter_edge=lambda x, y: edge_matches_prefix((x, y), places_prefix),
)
objects = nx.subgraph_view(
    G,
    filter_node=lambda x: node_matches_prefix(x, objects_prefix),
    filter_edge=lambda x, y: edge_matches_prefix((x, y), objects_prefix),
)
rooms = nx.subgraph_view(
    G,
    filter_node=lambda x: node_matches_prefix(x, rooms_prefix),
    filter_edge=lambda x, y: edge_matches_prefix((x, y), rooms_prefix),
)


place_nodes = [n for n in places]
object_nodes = [n for n in objects]


# %%
fig_width = 9.25
fig_height = 4
sns.set()

fig, ax = plt.subplots()
draw_graph(ax, rooms, node_size=node_size)
fig.set_size_inches([fig_width, fig_height])
plt.show()


# %%
for room in rooms:
    contents = []
    for n in G[room]:
        if node_matches_prefix(n, places_prefix):
            contents.append(make_label(n))

    print(f"Room {make_label(room)}: {len(contents)} children")
